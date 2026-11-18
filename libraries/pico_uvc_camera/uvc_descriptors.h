/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#pragma once

// Configuration descriptor parsing: the video interfaces, the MJPEG format
// with its frame sizes, and the isochronous alternate settings.

#include <cstdint>
#include <span>

#ifndef UVC_MAX_FRAMES
#define UVC_MAX_FRAMES 24
#endif

#ifndef UVC_MAX_ALTS
#define UVC_MAX_ALTS 16
#endif

struct UvcFrameInfo
{
    uint8_t index = 0;
    uint16_t width = 0;
    uint16_t height = 0;
    uint32_t default_interval = 0;   // 100 ns units
    uint32_t max_interval = 0;       // the slowest rate offered
    uint32_t max_buffer = 0;         // dwMaxVideoFrameBufferSize, usually width * height * 2
};

struct UvcAltInfo
{
    uint8_t alt = 0;
    uint8_t endpoint = 0;
    uint16_t packet_bytes = 0;       // per (micro)frame: size * transactions
};

struct UvcDeviceInfo
{
    uint16_t bcd_uvc = 0;
    uint8_t control_interface = 0xff;
    uint8_t streaming_interface = 0xff;
    uint8_t endpoint = 0;

    uint8_t mjpeg_format = 0;        // bFormatIndex, 0: no MJPEG
    uint8_t frame_count = 0;
    UvcFrameInfo frames[UVC_MAX_FRAMES];

    uint8_t alt_count = 0;
    UvcAltInfo alts[UVC_MAX_ALTS];
};

bool uvc_parse_config(std::span<const uint8_t> desc, UvcDeviceInfo& info);

// The MJPEG size closest to the request, by pixel count, the smaller one on a tie.
const UvcFrameInfo *uvc_pick_frame(const UvcDeviceInfo& info, uint16_t width, uint16_t height);

// The smallest alternate setting carrying payload bytes per (micro)frame, at
// most max_packet; the largest one under max_packet if none is big enough.
const UvcAltInfo *uvc_pick_alt(const UvcDeviceInfo& info, uint32_t payload, uint16_t max_packet);
