/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "command_registration.h"

struct Snap
{
    static constexpr std::string_view name = "snap";
    static constexpr std::string_view help = "snap[/<width>x<height>] - stream one JPEG from the USB webcam";

    static constexpr bool streams_output = true;

    bool on_header(CommandContext& ctx);
    bool on_send_space(CommandContext& ctx);
    void on_abort(CommandContext& ctx, const char *reason);

private:
    bool pump(CommandContext& ctx);
    void finish();

    bool m_waiting = false;
    bool m_streaming = false;
    uint32_t m_sent = 0;
    uint32_t m_total = 0;
    uint32_t m_dht_at = 0;
};

struct CameraStatus
{
    static constexpr std::string_view name = "camera";
    static constexpr std::string_view help = "camera[/sizes] - parameters offered by usb webcam";

    bool on_header(CommandContext& ctx);
};

struct UsbStatus
{
    static constexpr std::string_view name = "usb";
    static constexpr std::string_view help = "usb[/start|/reset] - USB state and counters";

    bool on_header(CommandContext& ctx);
};
