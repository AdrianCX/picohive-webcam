/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#pragma once

// The streaming parameters (probe / commit), UVC 1.0 to 1.5.

#include <cstdint>

#include "uvc_transport.h"

struct UvcProbe
{
    uint16_t hint = 0;
    uint8_t format = 0;
    uint8_t frame = 0;
    uint32_t interval = 0;            // 100 ns units
    uint16_t key_frame_rate = 0;
    uint16_t p_frame_rate = 0;
    uint16_t comp_quality = 0;
    uint16_t comp_window = 0;
    uint16_t delay = 0;
    uint32_t max_frame_size = 0;      // what the camera thinks one frame may take
    uint32_t max_payload = 0;         // bytes per (micro)frame it wants to send
    // UVC 1.1 and later
    uint32_t clock = 0;
    uint8_t framing_info = 0;
    uint8_t preferred_version = 0;
    uint8_t min_version = 0;
    uint8_t max_version = 0;
};

constexpr uint16_t UVC_PROBE_MAX_BYTES = 48;

uint16_t uvc_probe_length(uint16_t bcd_uvc);

void uvc_probe_pack(const UvcProbe& probe, uint8_t *out, uint16_t len);
bool uvc_probe_unpack(const uint8_t *in, uint32_t len, UvcProbe& probe);

// SET_CUR / GET_CUR on VS_PROBE_CONTROL (commit = false) or VS_COMMIT_CONTROL.
UsbSetup uvc_probe_setup(bool set, bool commit, uint8_t streaming_interface, uint16_t len);
