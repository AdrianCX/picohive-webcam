/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#pragma once

// Linux backends for pico_uvc_camera:
//   UvcTransportLibusb  a real camera through libusb, uvcvideo detached while
//                       we hold it, so the UVC work is ours as on the board.
//                       UVC_RECORD=<file> also writes what it sees to a fixture.
//   UvcTransportReplay  plays such a fixture back (UVC_REPLAY=<file>).

#include <cstdint>
#include <cstdio>
#include <vector>

#include "os_loop.h"
#include "uvc_transport.h"

struct libusb_context;
struct libusb_device_handle;
struct libusb_transfer;

// The fixture format: records of [type u8][length u32 little endian][bytes].
namespace UvcFixture
{
    constexpr uint8_t SPEED = 'S';         // 1 byte: 1 full speed, 0 high speed
    constexpr uint8_t DESCRIPTOR = 'D';    // the configuration descriptor
    constexpr uint8_t GET = 'G';           // what a control IN transfer returned
    constexpr uint8_t PACKET = 'P';        // one iso packet, 0 bytes for an empty one
    constexpr uint8_t BAD_PACKET = 'B';    // an iso packet the controller failed
}

class UvcTransportLibusb : public UvcTransport, public OsPollClient
{
public:
    bool begin() override;
    void poll(uint64_t now_us) override;
    bool device_present() const override { return m_handle != nullptr; }
    bool full_speed() const override { return m_full_speed; }
    std::span<const uint8_t> config_descriptor() const override { return m_descriptor; }
    bool claim(uint8_t control_interface, uint8_t streaming_interface) override;
    bool control(const UsbSetup& setup, uint8_t *data) override;
    bool set_alt(uint8_t interface, uint8_t alt) override;
    UsbOp op_state(uint32_t *len) const override;
    bool start_iso(uint8_t endpoint, uint16_t packet_size, UvcPacketSink *sink) override;
    void stop_iso() override;
    bool iso_running() const override { return m_in_flight > 0; }
    void release() override;

    void on_poll(short revents) override;

    static void at_exit();

private:
    static constexpr int TRANSFERS = 8;
    static constexpr int PACKETS_PER_TRANSFER = 32;

    void scan(uint64_t now_us);
    bool open(void *device);
    void close_device();
    void handle_events();
    void watch_fds();
    void record(uint8_t type, const uint8_t *data, uint32_t len);

    static void control_done(libusb_transfer *transfer);
    static void iso_done(libusb_transfer *transfer);

    libusb_context *m_ctx = nullptr;
    libusb_device_handle *m_handle = nullptr;
    uint64_t m_token = 0;
    uint64_t m_next_scan_us = 0;
    bool m_full_speed = false;
    bool m_warned_access = false;
    std::vector<uint8_t> m_descriptor;
    std::vector<int> m_watched;

    int m_claimed[2] = { -1, -1 };

    UsbOp m_op = UsbOp::Idle;
    uint32_t m_op_len = 0;
    uint8_t *m_op_data = nullptr;
    libusb_transfer *m_control = nullptr;
    std::vector<uint8_t> m_control_buf;

    UvcPacketSink *m_sink = nullptr;
    libusb_transfer *m_iso[TRANSFERS] = {};
    int m_in_flight = 0;
    bool m_streaming = false;
    bool m_gone = false;

    FILE *m_record = nullptr;
    uint32_t m_recorded_packets = 0;
};

class UvcTransportReplay : public UvcTransport
{
public:
    explicit UvcTransportReplay(const char *path) : m_path(path) {}

    bool begin() override;
    void poll(uint64_t now_us) override;
    bool device_present() const override { return m_loaded; }
    bool full_speed() const override { return m_full_speed; }
    std::span<const uint8_t> config_descriptor() const override { return m_descriptor; }
    bool claim(uint8_t, uint8_t) override { return true; }
    bool control(const UsbSetup& setup, uint8_t *data) override;
    bool set_alt(uint8_t, uint8_t) override { m_op = UsbOp::Done; return true; }
    UsbOp op_state(uint32_t *len) const override;
    bool start_iso(uint8_t endpoint, uint16_t packet_size, UvcPacketSink *sink) override;
    void stop_iso() override { m_sink = nullptr; }
    bool iso_running() const override { return m_sink != nullptr; }
    void release() override { m_sink = nullptr; m_next_get = 0; }

private:
    struct Packet
    {
        uint32_t offset;
        uint32_t len;
        bool ok;
    };

    const char *m_path;
    bool m_loaded = false;
    bool m_full_speed = true;
    std::vector<uint8_t> m_descriptor;
    std::vector<std::vector<uint8_t>> m_gets;
    std::vector<uint8_t> m_data;
    std::vector<Packet> m_packets;

    UsbOp m_op = UsbOp::Idle;
    uint32_t m_op_len = 0;
    size_t m_next_get = 0;

    UvcPacketSink *m_sink = nullptr;
    size_t m_next_packet = 0;
    uint64_t m_clock_us = 0;
};
