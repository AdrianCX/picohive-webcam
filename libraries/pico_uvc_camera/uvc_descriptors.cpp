/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "uvc_descriptors.h"

namespace
{
    constexpr uint8_t DT_INTERFACE = 0x04;
    constexpr uint8_t DT_ENDPOINT = 0x05;
    constexpr uint8_t DT_CS_INTERFACE = 0x24;

    constexpr uint8_t CLASS_VIDEO = 0x0e;
    constexpr uint8_t SUBCLASS_CONTROL = 0x01;
    constexpr uint8_t SUBCLASS_STREAMING = 0x02;

    constexpr uint8_t VC_HEADER = 0x01;
    constexpr uint8_t VS_INPUT_HEADER = 0x01;
    constexpr uint8_t VS_FORMAT_MJPEG = 0x06;
    constexpr uint8_t VS_FRAME_MJPEG = 0x07;

    uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
    uint32_t u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
}

bool uvc_parse_config(std::span<const uint8_t> desc, UvcDeviceInfo& info)
{
    info = UvcDeviceInfo();

    enum class In { Other, Control, Streaming };
    In in = In::Other;
    uint8_t interface = 0xff;
    uint8_t alt = 0;
    bool in_mjpeg = false;

    size_t pos = 0;
    while (pos + 2 <= desc.size())
    {
        const uint8_t *d = desc.data() + pos;
        uint8_t len = d[0];
        uint8_t type = d[1];

        if ((len < 2) || (pos + len > desc.size()))
        {
            return false;
        }

        pos += len;

        if ((type == DT_INTERFACE) && (len >= 9))
        {
            interface = d[2];
            alt = d[3];
            in = In::Other;

            if (d[5] == CLASS_VIDEO)
            {
                if ((d[6] == SUBCLASS_CONTROL) && (info.control_interface == 0xff))
                {
                    info.control_interface = interface;
                }

                if (d[6] == SUBCLASS_CONTROL && interface == info.control_interface)
                {
                    in = In::Control;
                }
                else if ((d[6] == SUBCLASS_STREAMING) &&
                         ((info.streaming_interface == 0xff) || (info.streaming_interface == interface)))
                {
                    in = In::Streaming;
                }
            }
            continue;
        }

        if ((in == In::Control) && (type == DT_CS_INTERFACE) && (len >= 5) && (d[2] == VC_HEADER))
        {
            info.bcd_uvc = u16(d + 3);
            continue;
        }

        if (in != In::Streaming)
        {
            continue;
        }

        if ((type == DT_CS_INTERFACE) && (alt == 0) && (len >= 3))
        {
            uint8_t subtype = d[2];

            if ((subtype == VS_INPUT_HEADER) && (len >= 7))
            {
                if (info.mjpeg_format != 0)
                {
                    in = In::Other;
                    continue;
                }
                info.endpoint = d[6];
            }
            else if (subtype == VS_FORMAT_MJPEG && len >= 11)
            {
                in_mjpeg = (info.mjpeg_format == 0);
                if (in_mjpeg)
                {
                    info.mjpeg_format = d[3];
                    info.streaming_interface = interface;
                }
            }
            else if ((subtype == VS_FRAME_MJPEG) && in_mjpeg && (len >= 26))
            {
                if (info.frame_count < UVC_MAX_FRAMES)
                {
                    UvcFrameInfo& f = info.frames[info.frame_count++];
                    f.index = d[3];
                    f.width = u16(d + 5);
                    f.height = u16(d + 7);
                    f.max_buffer = u32(d + 17);
                    f.default_interval = u32(d + 21);
                    f.max_interval = f.default_interval;

                    uint8_t interval_type = d[25];
                    if ((interval_type == 0) && (len >= 38))
                    {
                        f.max_interval = u32(d + 30);   // continuous: min, max, step
                    }
                    else
                    {
                        for (uint8_t i = 0; (i < interval_type) && (26u + 4u * i + 4u <= len); ++i)
                        {
                            uint32_t interval = u32(d + 26 + 4 * i);
                            if (interval > f.max_interval)
                            {
                                f.max_interval = interval;
                            }
                        }
                    }
                }
            }
            else if (subtype != VS_FRAME_MJPEG)
            {
                in_mjpeg = false;
            }
            continue;
        }

        if ((type == DT_ENDPOINT) && (len >= 7) && (alt != 0) && ((d[3] & 0x03) == 0x01) && (d[2] & 0x80))
        {
            if ((info.alt_count < UVC_MAX_ALTS) && ((info.endpoint == 0) || (info.endpoint == d[2])))
            {
                uint16_t mps = u16(d + 4);
                UvcAltInfo& a = info.alts[info.alt_count++];
                a.alt = alt;
                a.endpoint = d[2];
                a.packet_bytes = (uint16_t)((mps & 0x7ff) * (1 + ((mps >> 11) & 0x3)));
            }
        }
    }

    // Alternate settings of a streaming interface that turned out not to carry MJPEG.
    uint8_t kept = 0;
    for (uint8_t i = 0; i < info.alt_count; ++i)
    {
        if (info.alts[i].endpoint == info.endpoint)
        {
            info.alts[kept++] = info.alts[i];
        }
    }
    info.alt_count = kept;

    return (info.control_interface != 0xff) && (info.mjpeg_format != 0) && (info.frame_count > 0) && (info.alt_count > 0);
}

const UvcFrameInfo *uvc_pick_frame(const UvcDeviceInfo& info, uint16_t width, uint16_t height)
{
    const UvcFrameInfo *best = nullptr;
    uint32_t wanted = (uint32_t)width * height;
    uint32_t best_diff = 0;

    for (uint8_t i = 0; i < info.frame_count; ++i)
    {
        const UvcFrameInfo& f = info.frames[i];
        uint32_t pixels = (uint32_t)f.width * f.height;
        uint32_t diff = (pixels > wanted) ? (pixels - wanted) : (wanted - pixels);

        if ((best == nullptr) || (diff < best_diff) ||
            ((diff == best_diff) && (pixels < (uint32_t)best->width * best->height)))
        {
            best = &f;
            best_diff = diff;
        }
    }

    return best;
}

const UvcAltInfo *uvc_pick_alt(const UvcDeviceInfo& info, uint32_t payload, uint16_t max_packet)
{
    const UvcAltInfo *fit = nullptr;
    const UvcAltInfo *largest = nullptr;

    for (uint8_t i = 0; i < info.alt_count; ++i)
    {
        const UvcAltInfo& a = info.alts[i];

        if (a.packet_bytes > max_packet)
        {
            continue;
        }

        if ((a.packet_bytes >= payload) && ((fit == nullptr) || (a.packet_bytes < fit->packet_bytes)))
        {
            fit = &a;
        }

        if ((largest == nullptr) || (a.packet_bytes > largest->packet_bytes))
        {
            largest = &a;
        }
    }

    return (fit != nullptr) ? fit : largest;
}
