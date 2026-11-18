/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <poll.h>
#include <stdlib.h>
#include <string.h>

#include <libusb.h>

#include "pico_logger.h"
#include "uvc_transport_libusb.h"

namespace
{
    constexpr uint64_t SCAN_US = 2000000;
    constexpr uint32_t DEFAULT_RECORD_PACKETS = 16000;

    UvcTransportLibusb *s_instance = nullptr;

    uint32_t record_limit()
    {
        const char *value = getenv("UVC_RECORD_PACKETS");
        return (value != nullptr) ? (uint32_t)strtoul(value, nullptr, 10) : DEFAULT_RECORD_PACKETS;
    }

    bool is_video_streaming(libusb_device *device)
    {
        libusb_config_descriptor *config = nullptr;
        if (libusb_get_active_config_descriptor(device, &config) != 0)
        {
            return false;
        }

        bool found = false;
        for (int i = 0; (i < config->bNumInterfaces) && !found; ++i)
        {
            const libusb_interface& intf = config->interface[i];
            for (int a = 0; a < intf.num_altsetting; ++a)
            {
                if ((intf.altsetting[a].bInterfaceClass == LIBUSB_CLASS_VIDEO) && (intf.altsetting[a].bInterfaceSubClass == 2))
                {
                    found = true;
                    break;
                }
            }
        }

        libusb_free_config_descriptor(config);
        return found;
    }

    // UVC_DEVICE=vvvv:pppp picks one camera when several are plugged in.
    bool wanted(libusb_device *device)
    {
        const char *filter = getenv("UVC_DEVICE");
        if ((filter == nullptr) || (filter[0] == 0))
        {
            return true;
        }

        libusb_device_descriptor dd;
        if (libusb_get_device_descriptor(device, &dd) != 0)
        {
            return false;
        }

        char id[16];
        snprintf(id, sizeof(id), "%04x:%04x", dd.idVendor, dd.idProduct);
        return strcasecmp(id, filter) == 0;
    }
}

UvcTransport& uvc_platform_transport()
{
    static UvcTransportLibusb libusb_transport;
    static UvcTransportReplay *replay = nullptr;

    const char *path = getenv("UVC_REPLAY");
    if ((path != nullptr) && (path[0] != 0))
    {
        if (replay == nullptr)
        {
            replay = new UvcTransportReplay(path);
        }
        return *replay;
    }

    return libusb_transport;
}

void uvc_transport_debug(std::string_view, char *out, uint32_t len)
{
    UvcTransport& t = uvc_platform_transport();
    snprintf(out, len, "linux: %s, device %s, %s speed, iso %s", getenv("UVC_REPLAY") ? "replay" : "libusb",
             t.device_present() ? "present" : "absent", t.full_speed() ? "full" : "high", t.iso_running() ? "on" : "off");
}

bool UvcTransportLibusb::begin()
{
    if (m_ctx != nullptr)
    {
        return true;
    }

    int r = libusb_init(&m_ctx);
    if (r != 0)
    {
        trace("UvcTransportLibusb::begin: libusb_init failed: %s", libusb_error_name(r));
        m_ctx = nullptr;
        return false;
    }

    m_token = OsLoop::acquire_token();
    s_instance = this;
    atexit(at_exit);

    watch_fds();
    trace("UvcTransportLibusb::begin: libusb %s up, looking for a UVC camera every %u s",
          libusb_get_version()->describe, (unsigned)(SCAN_US / 1000000));
    return true;
}

void UvcTransportLibusb::at_exit()
{
    // Gives the camera back to uvcvideo: without this it stays detached until replugged.
    if (s_instance == nullptr)
    {
        return;
    }

    if (s_instance->m_handle != nullptr)
    {
        s_instance->close_device();
    }

    // Stops libusb's own event thread too.
    for (int fd : s_instance->m_watched)
    {
        OsLoop::unwatch(fd);
    }
    s_instance->m_watched.clear();
    libusb_exit(s_instance->m_ctx);
    s_instance->m_ctx = nullptr;
    s_instance = nullptr;
}

void UvcTransportLibusb::watch_fds()
{
    for (int fd : m_watched)
    {
        OsLoop::unwatch(fd);
    }
    m_watched.clear();

    const libusb_pollfd **fds = libusb_get_pollfds(m_ctx);
    if (fds == nullptr)
    {
        return;
    }

    for (int i = 0; fds[i] != nullptr; ++i)
    {
        OsLoop::watch(m_token, fds[i]->fd, fds[i]->events, this);
        m_watched.push_back(fds[i]->fd);
    }

    libusb_free_pollfds(fds);
}

void UvcTransportLibusb::on_poll(short)
{
    handle_events();
}

void UvcTransportLibusb::handle_events()
{
    timeval zero = { 0, 0 };
    libusb_handle_events_timeout_completed(m_ctx, &zero, nullptr);
}

void UvcTransportLibusb::poll(uint64_t now_us)
{
    if (m_ctx == nullptr)
    {
        return;
    }

    handle_events();

    if (m_gone && (m_handle != nullptr))
    {
        trace("UvcTransportLibusb::poll: camera disconnected");
        close_device();
    }

    if ((m_handle == nullptr) && (now_us >= m_next_scan_us))
    {
        m_next_scan_us = now_us + SCAN_US;
        scan(now_us);
    }
}

void UvcTransportLibusb::scan(uint64_t)
{
    libusb_device **list = nullptr;
    ssize_t count = libusb_get_device_list(m_ctx, &list);

    for (ssize_t i = 0; i < count; ++i)
    {
        if (is_video_streaming(list[i]) && wanted(list[i]) && open(list[i]))
        {
            break;
        }
    }

    libusb_free_device_list(list, 1);
}

bool UvcTransportLibusb::open(void *opaque)
{
    libusb_device *device = (libusb_device *)opaque;
    libusb_device_descriptor dd;
    libusb_get_device_descriptor(device, &dd);

    libusb_device_handle *handle = nullptr;
    int r = libusb_open(device, &handle);
    if (r != 0)
    {
        if (!m_warned_access)
        {
            m_warned_access = true;
            trace("UvcTransportLibusb::open: camera %04x:%04x at /dev/bus/usb/%03u/%03u: %s%s",
                  dd.idVendor, dd.idProduct, libusb_get_bus_number(device), libusb_get_device_address(device),
                  libusb_error_name(r), (r == LIBUSB_ERROR_ACCESS) ? " (no write access, see docs/uvc-camera.md)" : "");
        }
        return false;
    }

    uint8_t head[9];
    r = libusb_get_descriptor(handle, LIBUSB_DT_CONFIG, 0, head, sizeof(head));
    if (r != (int)sizeof(head))
    {
        trace("UvcTransportLibusb::open: reading the configuration descriptor failed: %s", libusb_error_name(r));
        libusb_close(handle);
        return false;
    }

    uint16_t total = (uint16_t)(head[2] | (head[3] << 8));
    m_descriptor.assign(total, 0);
    r = libusb_get_descriptor(handle, LIBUSB_DT_CONFIG, 0, m_descriptor.data(), total);
    if (r != total)
    {
        trace("UvcTransportLibusb::open: configuration descriptor came back short (%d of %u)", r, total);
        libusb_close(handle);
        m_descriptor.clear();
        return false;
    }

    m_handle = handle;
    m_gone = false;
    m_full_speed = (libusb_get_device_speed(device) == LIBUSB_SPEED_FULL);
    m_warned_access = false;

    static const char *const SPEEDS[] = { "unknown", "low", "full", "high", "super", "super+" };
    int speed = libusb_get_device_speed(device);
    trace("UvcTransportLibusb::open: camera %04x:%04x opened, %s speed, %u byte configuration descriptor",
          dd.idVendor, dd.idProduct, ((speed >= 0) && (speed <= 5)) ? SPEEDS[speed] : "?", total);

    const char *record_path = getenv("UVC_RECORD");
    if ((record_path != nullptr) && (record_path[0] != 0) && (m_record == nullptr))
    {
        m_record = fopen(record_path, "wb");
        trace("UvcTransportLibusb::open: recording to %s %s", record_path, m_record ? "" : "failed");
        uint8_t fs = m_full_speed ? 1 : 0;
        record(UvcFixture::SPEED, &fs, 1);
        record(UvcFixture::DESCRIPTOR, m_descriptor.data(), total);
    }

    watch_fds();
    return true;
}

void UvcTransportLibusb::record(uint8_t type, const uint8_t *data, uint32_t len)
{
    if (m_record == nullptr)
    {
        return;
    }

    uint8_t head[5] = { type, (uint8_t)len, (uint8_t)(len >> 8), (uint8_t)(len >> 16), (uint8_t)(len >> 24) };
    fwrite(head, 1, sizeof(head), m_record);
    if (len > 0)
    {
        fwrite(data, 1, len, m_record);
    }

    if ((type == UvcFixture::PACKET) || (type == UvcFixture::BAD_PACKET))
    {
        if (++m_recorded_packets >= record_limit())
        {
            trace("UvcTransportLibusb::record: recorded %u packets, recording closed", m_recorded_packets);
            fclose(m_record);
            m_record = nullptr;
        }
    }
}

bool UvcTransportLibusb::claim(uint8_t control_interface, uint8_t streaming_interface)
{
    if (m_handle == nullptr)
    {
        return false;
    }

    libusb_set_auto_detach_kernel_driver(m_handle, 1);

    uint8_t interfaces[2] = { control_interface, streaming_interface };
    for (int i = 0; i < 2; ++i)
    {
        if (m_claimed[i] == interfaces[i])
        {
            continue;
        }

        int r = libusb_claim_interface(m_handle, interfaces[i]);
        if (r != 0)
        {
            trace("UvcTransportLibusb::claim: claiming interface %u failed: %s%s", interfaces[i], libusb_error_name(r),
                  (r == LIBUSB_ERROR_BUSY) ? " (another program has the camera open)" : "");
            return false;
        }
        m_claimed[i] = interfaces[i];
    }

    return true;
}

void UvcTransportLibusb::control_done(libusb_transfer *transfer)
{
    UvcTransportLibusb *self = (UvcTransportLibusb *)transfer->user_data;

    if (transfer->status == LIBUSB_TRANSFER_COMPLETED)
    {
        const libusb_control_setup *setup = libusb_control_transfer_get_setup(transfer);
        self->m_op_len = transfer->actual_length;

        if ((setup->bmRequestType & 0x80) && (self->m_op_data != nullptr))
        {
            memcpy(self->m_op_data, libusb_control_transfer_get_data(transfer), transfer->actual_length);
            self->record(UvcFixture::GET, self->m_op_data, transfer->actual_length);
        }
        self->m_op = UsbOp::Done;
    }
    else
    {
        trace("UvcTransportLibusb::control_done: control transfer status %d", transfer->status);
        if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
        {
            self->m_gone = true;
        }
        self->m_op = UsbOp::Failed;
    }
}

bool UvcTransportLibusb::control(const UsbSetup& setup, uint8_t *data)
{
    if ((m_handle == nullptr) || (m_op == UsbOp::Pending))
    {
        return false;
    }

    if (m_control == nullptr)
    {
        m_control = libusb_alloc_transfer(0);
    }

    m_control_buf.assign(LIBUSB_CONTROL_SETUP_SIZE + setup.wLength, 0);
    libusb_fill_control_setup(m_control_buf.data(), setup.bmRequestType, setup.bRequest, setup.wValue, setup.wIndex, setup.wLength);

    if (!(setup.bmRequestType & 0x80) && (setup.wLength > 0))
    {
        memcpy(m_control_buf.data() + LIBUSB_CONTROL_SETUP_SIZE, data, setup.wLength);
    }

    m_op_data = data;
    m_op_len = 0;
    libusb_fill_control_transfer(m_control, m_handle, m_control_buf.data(), control_done, this, 1000);

    int r = libusb_submit_transfer(m_control);
    if (r != 0)
    {
        trace("UvcTransportLibusb::control: submitting a control transfer failed: %s", libusb_error_name(r));
        m_op = UsbOp::Failed;
        return false;
    }

    m_op = UsbOp::Pending;
    return true;
}

bool UvcTransportLibusb::set_alt(uint8_t interface, uint8_t alt)
{
    if (m_handle == nullptr)
    {
        return false;
    }

    int r = libusb_set_interface_alt_setting(m_handle, interface, alt);
    if (r != 0)
    {
        trace("UvcTransportLibusb::set_alt: alt %u on interface %u failed: %s", alt, interface, libusb_error_name(r));
        if (r == LIBUSB_ERROR_NO_DEVICE)
        {
            m_gone = true;
        }
    }

    m_op = (r == 0) ? UsbOp::Done : UsbOp::Failed;
    return true;
}

UsbOp UvcTransportLibusb::op_state(uint32_t *len) const
{
    if (len != nullptr)
    {
        *len = m_op_len;
    }
    return m_op;
}

void UvcTransportLibusb::iso_done(libusb_transfer *transfer)
{
    UvcTransportLibusb *self = (UvcTransportLibusb *)transfer->user_data;

    if ((transfer->status == LIBUSB_TRANSFER_COMPLETED) && self->m_streaming && (self->m_sink != nullptr))
    {
        for (int i = 0; i < transfer->num_iso_packets; ++i)
        {
            const libusb_iso_packet_descriptor& p = transfer->iso_packet_desc[i];
            const uint8_t *data = libusb_get_iso_packet_buffer_simple(transfer, i);
            bool ok = (p.status == LIBUSB_TRANSFER_COMPLETED);

            self->m_sink->on_iso_packet(data, ok ? p.actual_length : 0, ok);
            self->record(ok ? UvcFixture::PACKET : UvcFixture::BAD_PACKET, data, ok ? p.actual_length : 0);
        }
    }
    else if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE)
    {
        self->m_gone = true;
    }

    if (self->m_streaming && !self->m_gone && (libusb_submit_transfer(transfer) == 0))
    {
        return;
    }

    for (int i = 0; i < TRANSFERS; ++i)
    {
        if (self->m_iso[i] == transfer)
        {
            self->m_iso[i] = nullptr;
        }
    }

    free(transfer->buffer);
    libusb_free_transfer(transfer);
    --self->m_in_flight;
}

bool UvcTransportLibusb::start_iso(uint8_t endpoint, uint16_t packet_size, UvcPacketSink *sink)
{
    if ((m_handle == nullptr) || (m_in_flight > 0))
    {
        return false;
    }

    m_sink = sink;
    m_streaming = true;

    for (int i = 0; i < TRANSFERS; ++i)
    {
        libusb_transfer *t = libusb_alloc_transfer(PACKETS_PER_TRANSFER);
        uint8_t *buffer = (uint8_t *)malloc((size_t)packet_size * PACKETS_PER_TRANSFER);

        libusb_fill_iso_transfer(t, m_handle, endpoint, buffer, packet_size * PACKETS_PER_TRANSFER,
                                 PACKETS_PER_TRANSFER, iso_done, this, 1000);
        libusb_set_iso_packet_lengths(t, packet_size);

        int r = libusb_submit_transfer(t);
        if (r != 0)
        {
            trace("UvcTransportLibusb::start_iso: submitting iso transfer %d failed: %s", i, libusb_error_name(r));
            free(buffer);
            libusb_free_transfer(t);
            break;
        }

        m_iso[i] = t;
        ++m_in_flight;
    }

    if (m_in_flight == 0)
    {
        m_streaming = false;
        return false;
    }

    trace("UvcTransportLibusb::start_iso: %d iso transfers of %d x %u bytes in flight (%u KB)",
          m_in_flight, PACKETS_PER_TRANSFER, packet_size, (unsigned)(m_in_flight * PACKETS_PER_TRANSFER * packet_size / 1024));
    return true;
}

void UvcTransportLibusb::stop_iso()
{
    m_streaming = false;

    for (int i = 0; i < TRANSFERS; ++i)
    {
        if (m_iso[i] != nullptr)
        {
            libusb_cancel_transfer(m_iso[i]);
        }
    }
}

void UvcTransportLibusb::release()
{
    if (m_handle == nullptr)
    {
        return;
    }

    stop_iso();

    if ((m_control != nullptr) && (m_op == UsbOp::Pending))
    {
        libusb_cancel_transfer(m_control);
    }

    for (int i = 0; (i < 50) && ((m_in_flight > 0) || (m_op == UsbOp::Pending)); ++i)
    {
        timeval tv = { 0, 2000 };
        libusb_handle_events_timeout_completed(m_ctx, &tv, nullptr);
    }

    for (int i = 1; i >= 0; --i)
    {
        if (m_claimed[i] >= 0)
        {
            if (!m_gone && (i == 1))
            {
                libusb_set_interface_alt_setting(m_handle, m_claimed[i], 0);
            }
            libusb_release_interface(m_handle, m_claimed[i]);
            m_claimed[i] = -1;
        }
    }
}

void UvcTransportLibusb::close_device()
{
    release();

    if (m_control != nullptr)
    {
        libusb_free_transfer(m_control);
        m_control = nullptr;
    }

    libusb_close(m_handle);
    m_handle = nullptr;
    m_descriptor.clear();
    m_op = UsbOp::Idle;
    m_gone = false;

    if (m_record != nullptr)
    {
        fclose(m_record);
        m_record = nullptr;
    }

    if (s_instance != nullptr)
    {
        watch_fds();
    }
}
