/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <cstring>

#include "uvc_control.h"

namespace
{
    void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
    void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; ++i) { p[i] = (uint8_t)(v >> (8 * i)); } }
    uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
    uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
}

uint16_t uvc_probe_length(uint16_t bcd_uvc)
{
    if (bcd_uvc >= 0x0150)
    {
        return 48;
    }
    if (bcd_uvc >= 0x0110)
    {
        return 34;
    }
    return 26;
}

void uvc_probe_pack(const UvcProbe& p, uint8_t *out, uint16_t len)
{
    uint8_t buf[UVC_PROBE_MAX_BYTES] = {};

    put16(buf + 0, p.hint);
    buf[2] = p.format;
    buf[3] = p.frame;
    put32(buf + 4, p.interval);
    put16(buf + 8, p.key_frame_rate);
    put16(buf + 10, p.p_frame_rate);
    put16(buf + 12, p.comp_quality);
    put16(buf + 14, p.comp_window);
    put16(buf + 16, p.delay);
    put32(buf + 18, p.max_frame_size);
    put32(buf + 22, p.max_payload);
    put32(buf + 26, p.clock);
    buf[30] = p.framing_info;
    buf[31] = p.preferred_version;
    buf[32] = p.min_version;
    buf[33] = p.max_version;

    memcpy(out, buf, (len < sizeof(buf)) ? len : sizeof(buf));
}

bool uvc_probe_unpack(const uint8_t *in, uint32_t len, UvcProbe& p)
{
    if (len < 26)
    {
        return false;
    }

    p.hint = get16(in + 0);
    p.format = in[2];
    p.frame = in[3];
    p.interval = get32(in + 4);
    p.key_frame_rate = get16(in + 8);
    p.p_frame_rate = get16(in + 10);
    p.comp_quality = get16(in + 12);
    p.comp_window = get16(in + 14);
    p.delay = get16(in + 16);
    p.max_frame_size = get32(in + 18);
    p.max_payload = get32(in + 22);

    if (len >= 34)
    {
        p.clock = get32(in + 26);
        p.framing_info = in[30];
        p.preferred_version = in[31];
        p.min_version = in[32];
        p.max_version = in[33];
    }

    return true;
}

UsbSetup uvc_probe_setup(bool set, bool commit, uint8_t streaming_interface, uint16_t len)
{
    constexpr uint8_t SET_CUR = 0x01;
    constexpr uint8_t GET_CUR = 0x81;
    constexpr uint8_t VS_PROBE_CONTROL = 0x01;
    constexpr uint8_t VS_COMMIT_CONTROL = 0x02;

    UsbSetup s;
    s.bmRequestType = set ? 0x21 : 0xa1;
    s.bRequest = set ? SET_CUR : GET_CUR;
    s.wValue = (uint16_t)((commit ? VS_COMMIT_CONTROL : VS_PROBE_CONTROL) << 8);
    s.wIndex = streaming_interface;
    s.wLength = len;
    return s;
}
