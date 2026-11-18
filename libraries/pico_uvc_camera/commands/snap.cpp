/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "snap.h"
#include "pico_helper.h"
#include "pico_logger.h"
#include "reply_buffer.h"
#include "uvc_camera.h"

bool Snap::on_header(CommandContext& ctx)
{
    UvcCamera& camera = UvcCamera::get();

    uint32_t width = 0;
    uint32_t height = 0;
    if (!ctx.args.empty())
    {
        size_t x = ctx.args.find('x');
        if ((x == std::string_view::npos) ||
            !PicoHelper::parse_number(ctx.args.substr(0, x), width) ||
            !PicoHelper::parse_number(ctx.args.substr(x + 1), height) ||
            (width == 0) || (height == 0) || (width > 4096) || (height > 4096))
        {
            return ctx.reply("no picture: snap/<width>x<height>, e.g. snap/640x480");
        }
    }

    if (!camera.request_frame((uint16_t)width, (uint16_t)height))
    {
        return ctx.reply_fmt(80, "no picture: %s", camera.last_error());
    }

    m_waiting = true;
    ctx.send_more_later();
    return true;
}

bool Snap::on_send_space(CommandContext& ctx)
{
    UvcCamera& camera = UvcCamera::get();

    if (m_waiting)
    {
        switch (camera.capture())
        {
            case UvcCamera::Capture::Pending:
                ctx.send_more_later();
                return true;

            case UvcCamera::Capture::Ready:
                break;

            default:
                m_waiting = false;
                camera.release_frame();
                ctx.sending_done();
                return ctx.reply_fmt(80, "no picture: %s", camera.last_error());
        }

        m_waiting = false;

        std::span<const uint8_t> jpeg = camera.frame();
        uint32_t dht_len = 0;
        uvc_standard_dht(dht_len);

        m_dht_at = uvc_jpeg_dht_insert_offset(jpeg.data(), jpeg.size());
        m_total = jpeg.size() + ((m_dht_at != 0) ? dht_len : 0);
        m_sent = 0;

        if (!ctx.begin_stream(name, m_total))
        {
            camera.release_frame();
            return false;
        }
        m_streaming = true;
    }

    return pump(ctx);
}

bool Snap::pump(CommandContext& ctx)
{
    std::span<const uint8_t> jpeg = UvcCamera::get().frame();
    uint32_t dht_len = 0;
    const uint8_t *dht = uvc_standard_dht(dht_len);

    while (m_sent < m_total)
    {
        uint32_t space = ctx.send_space();
        if (space == 0)
        {
            ctx.send_more_later();
            return true;
        }

        // The image as sent: jpeg[0, at), the tables, jpeg[at, end).
        const uint8_t *from;
        uint32_t left;
        if ((m_dht_at == 0) || (m_sent < m_dht_at))
        {
            from = jpeg.data() + m_sent;
            left = ((m_dht_at == 0) ? jpeg.size() : m_dht_at) - m_sent;
        }
        else if (m_sent < m_dht_at + dht_len)
        {
            from = dht + (m_sent - m_dht_at);
            left = m_dht_at + dht_len - m_sent;
        }
        else
        {
            uint32_t at = m_sent - dht_len;
            from = jpeg.data() + at;
            left = jpeg.size() - at;
        }

        uint32_t n = (left < space) ? left : space;
        if (!ctx.send_chunk(from, n))
        {
            trace("snap: send failed with %u of %u bytes left", m_total - m_sent, m_total);
            finish();
            return false;
        }
        m_sent += n;
    }

    finish();
    ctx.sending_done();
    return true;
}

void Snap::finish()
{
    if (m_streaming || m_waiting)
    {
        UvcCamera::get().release_frame();
    }
    m_streaming = false;
    m_waiting = false;
}

void Snap::on_abort(CommandContext&, const char *reason)
{
    trace("snap: stopped: %s", reason);

    if (m_waiting)
    {
        UvcCamera::get().cancel_request();
        m_waiting = false;
    }
    finish();
}

bool CameraStatus::on_header(CommandContext& ctx)
{
    ReplyBuffer text(ctx, 320);
    if (!text)
    {
        return false;
    }

    if (ctx.args == "sizes")
    {
        UvcCamera::get().list_sizes(text.data(), 320);
    }
    else
    {
        UvcCamera::get().describe(text.data(), 320);
    }
    text.set_size(strlen(text.data()));
    return ctx.reply(text);
}

bool UsbStatus::on_header(CommandContext& ctx)
{
    ReplyBuffer text(ctx, 400);
    if (!text)
    {
        return false;
    }

    uvc_transport_debug(ctx.args, text.data(), 400);
    text.set_size(strlen(text.data()));
    return ctx.reply(text);
}

MQTT_COMMAND(Snap);
WS_COMMAND(Snap);

MQTT_COMMAND(CameraStatus);
WS_COMMAND(CameraStatus);

MQTT_COMMAND(UsbStatus);
WS_COMMAND(UsbStatus);
