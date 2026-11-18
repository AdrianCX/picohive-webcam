/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

// Plays back a fixture recorded by UvcTransportLibusb (UVC_RECORD=<file>):
// the same descriptor, the same probe answer, the same packets at the rate
// they came, looped. No camera, no libusb access needed.

#include <cstdio>
#include <cstring>

#include "pico_logger.h"
#include "uvc_transport_libusb.h"

bool UvcTransportReplay::begin()
{
    FILE *f = fopen(m_path, "rb");
    if (f == nullptr)
    {
        trace("UvcTransportReplay: cannot open %s", m_path);
        return false;
    }

    uint8_t head[5];
    std::vector<uint8_t> body;

    while (fread(head, 1, sizeof(head), f) == sizeof(head))
    {
        uint32_t len = (uint32_t)head[1] | ((uint32_t)head[2] << 8) | ((uint32_t)head[3] << 16) | ((uint32_t)head[4] << 24);
        body.resize(len);
        if ((len > 0) && (fread(body.data(), 1, len, f) != len))
        {
            break;
        }

        switch (head[0])
        {
            case UvcFixture::SPEED:
                m_full_speed = (len > 0) && (body[0] != 0);
                break;
            case UvcFixture::DESCRIPTOR:
                m_descriptor = body;
                break;
            case UvcFixture::GET:
                m_gets.push_back(body);
                break;
            case UvcFixture::PACKET:
            case UvcFixture::BAD_PACKET:
                m_packets.push_back(Packet{ (uint32_t)m_data.size(), len, head[0] == UvcFixture::PACKET });
                m_data.insert(m_data.end(), body.begin(), body.end());
                break;
        }
    }

    fclose(f);

    m_loaded = !m_descriptor.empty() && !m_packets.empty();
    trace("UvcTransportReplay: %s: %u packets, %u KB, %u control answers, %s speed%s",
          m_path, (unsigned)m_packets.size(), (unsigned)(m_data.size() / 1024), (unsigned)m_gets.size(),
          m_full_speed ? "full" : "high", m_loaded ? "" : " - not usable");
    return m_loaded;
}

bool UvcTransportReplay::control(const UsbSetup& setup, uint8_t *data)
{
    m_op_len = 0;

    if (setup.bmRequestType & 0x80)
    {
        if (m_gets.empty())
        {
            m_op = UsbOp::Failed;
            return true;
        }

        const std::vector<uint8_t>& answer = m_gets[m_next_get];
        m_next_get = (m_next_get + 1) % m_gets.size();

        m_op_len = (answer.size() < setup.wLength) ? (uint32_t)answer.size() : setup.wLength;
        memcpy(data, answer.data(), m_op_len);
    }

    m_op = UsbOp::Done;
    return true;
}

UsbOp UvcTransportReplay::op_state(uint32_t *len) const
{
    if (len != nullptr)
    {
        *len = m_op_len;
    }
    return m_op;
}

bool UvcTransportReplay::start_iso(uint8_t, uint16_t, UvcPacketSink *sink)
{
    m_sink = sink;
    m_clock_us = 0;
    return true;
}

void UvcTransportReplay::poll(uint64_t now_us)
{
    if (m_sink == nullptr)
    {
        return;
    }

    if (m_clock_us == 0)
    {
        m_clock_us = now_us;
    }

    // As the bus would: one packet per frame (1 ms) or microframe (125 us).
    const uint64_t step_us = m_full_speed ? 1000 : 125;

    for (int burst = 0; (m_clock_us <= now_us) && (burst < 2000); ++burst)
    {
        const Packet& p = m_packets[m_next_packet];
        m_next_packet = (m_next_packet + 1) % m_packets.size();

        m_sink->on_iso_packet(m_data.data() + p.offset, p.len, p.ok);
        m_clock_us += step_us;
    }

    // Far behind (a stalled loop): drop the backlog, as the bus would.
    if (m_clock_us + 100000 < now_us)
    {
        m_clock_us = now_us;
    }
}
