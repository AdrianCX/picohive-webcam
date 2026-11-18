/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#pragma once

// The only part of pico_uvc_camera that differs per platform: libusb (and a
// replay of recorded packets) on linux, TinyUSB host on the board. Everything
// is non-blocking: control() and set_alt() start an operation whose end is
// seen through op_state(), iso packets arrive through the sink while poll()
// runs.

#include <cstdint>
#include <span>
#include <string_view>

struct UsbSetup
{
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

enum class UsbOp : uint8_t
{
    Idle,
    Pending,
    Done,
    Failed,
};

class UvcPacketSink
{
public:
    virtual ~UvcPacketSink() = default;

    // One isochronous packet (payload header included). ok is false for a
    // packet the controller reported damaged or missing: the frame it belongs
    // to cannot be trusted.
    virtual void on_iso_packet(const uint8_t *data, uint32_t len, bool ok) = 0;
};

class UvcTransport
{
public:
    virtual ~UvcTransport() = default;

    virtual bool begin() = 0;

    // Drives the stack: detection, completions, packets. Never blocks.
    virtual void poll(uint64_t now_us) = 0;

    // A video device is attached and its configuration descriptor was read.
    virtual bool device_present() const = 0;

    // Full speed: 1 ms frames, at most 1023 bytes per packet, as on the Pico.
    virtual bool full_speed() const = 0;

    // The configuration descriptor fetched when the device appeared.
    virtual std::span<const uint8_t> config_descriptor() const = 0;

    // Take the video control and streaming interfaces (on linux: away from uvcvideo).
    virtual bool claim(uint8_t control_interface, uint8_t streaming_interface) = 0;

    // Starts a control transfer; data holds setup.wLength bytes and must stay
    // valid until op_state() is no longer Pending.
    virtual bool control(const UsbSetup& setup, uint8_t *data) = 0;
    virtual bool set_alt(uint8_t interface, uint8_t alt) = 0;

    // The last control() or set_alt(); len is what an IN transfer returned.
    virtual UsbOp op_state(uint32_t *len = nullptr) const = 0;

    virtual bool start_iso(uint8_t endpoint, uint16_t packet_size, UvcPacketSink *sink) = 0;
    virtual void stop_iso() = 0;
    virtual bool iso_running() const = 0;

    // Gives the interfaces back (linux reattaches uvcvideo).
    virtual void release() = 0;
};

// Provided by the platform backend.
UvcTransport& uvc_platform_transport();

// The usb command: the transport's own state and knobs, as one line of text.
void uvc_transport_debug(std::string_view args, char *out, uint32_t len);
