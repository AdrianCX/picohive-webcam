/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <cstdio>
#include <cstring>
#include <string_view>

#include "hardware/irq.h"
#include "hardware/regs/usb.h"
#include "hardware/resets.h"
#include "hardware/structs/usb.h"
#include "hardware/sync.h"
#include "pico.h"

#include "pico_logger.h"
#include "uvc_transport.h"

namespace
{
    constexpr uint32_t SIE_BASE = USB_SIE_CTRL_SOF_EN_BITS | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS |
                                  USB_SIE_CTRL_PULLDOWN_EN_BITS | USB_SIE_CTRL_EP0_INT_1BUF_BITS;

    constexpr uint32_t TYPE_CONTROL = 0;
    constexpr uint32_t TYPE_ISO = 1;

    constexpr uint32_t EPX_DATA_OFFSET = 0x180;
    constexpr uint8_t DEVICE_ADDRESS = 1;

    constexpr uint64_t AUTOSTART_US = 30000000;
    constexpr uint64_t STAGE_TIMEOUT_US = 500000;
    constexpr uint64_t STATS_US = 10000000;
    constexpr uint32_t CHANGES_LOGGED = 10;
    constexpr uint32_t RING_BYTES = 16384;
    constexpr uint16_t LOST = 0xffff;

    struct Shared
    {
        volatile bool setup_done;
        volatile bool buf_done;
        volatile uint16_t buf_len;
        volatile bool stalled;
        volatile bool timed_out;
        volatile bool error;
        volatile uint32_t error_status;
        volatile bool conn_changed;
        volatile bool iso_on;
        volatile bool iso_busy;
        volatile uint8_t iso_ep;
        volatile uint16_t iso_mps;
        volatile uint32_t iso_kick_frame;
        volatile bool iso_lost;
        volatile uint32_t packets;
        volatile uint32_t empty_packets;
        volatile uint32_t iso_timeouts;
        volatile uint32_t iso_errors;
        volatile uint32_t iso_stuck;
        volatile uint32_t ring_drops;
        volatile uint32_t sofs;
        volatile uint32_t irqs;
        volatile uint32_t unexpected;
        volatile uint32_t last_unexpected;
        volatile uint16_t max_len;
        volatile uint64_t bytes;
        volatile uint32_t ring_head;
        volatile uint32_t ring_tail;
        volatile uint32_t ring_peak;
    };

    Shared s;
    uint8_t s_ring[RING_BYTES];

    inline uint8_t *epx_data() { return (uint8_t *)usbh_dpram + EPX_DATA_OFFSET; }

    inline uint32_t ring_used() { return s.ring_tail - s.ring_head; }

    void __not_in_flash_func(ring_put)(const uint8_t *data, uint32_t len)
    {
        uint32_t tail = s.ring_tail;
        for (uint32_t i = 0; i < len; ++i)
        {
            s_ring[(tail + i) & (RING_BYTES - 1)] = data[i];
        }
        s.ring_tail = tail + len;
    }

    void __not_in_flash_func(ring_packet)(const uint8_t *data, uint16_t len)
    {
        if (len == 0)
        {
            s.empty_packets = s.empty_packets + 1;
            return;
        }

        uint32_t need = 2u + len + (s.iso_lost ? 2u : 0u);
        if (RING_BYTES - ring_used() < need)
        {
            s.ring_drops = s.ring_drops + 1;
            s.iso_lost = true;
            return;
        }

        if (s.iso_lost)
        {
            uint8_t marker[2] = { 0xff, 0xff };
            ring_put(marker, 2);
            s.iso_lost = false;
        }

        uint8_t head[2] = { (uint8_t)len, (uint8_t)(len >> 8) };
        ring_put(head, 2);
        ring_put(data, len);

        s.packets = s.packets + 1;
        s.bytes = s.bytes + len;
        if (len > s.max_len)
        {
            s.max_len = len;
        }
        if (ring_used() > s.ring_peak)
        {
            s.ring_peak = ring_used();
        }
    }

    void __not_in_flash_func(start_epx)(uint32_t sie_flags)
    {
        usb_hw->sie_ctrl = SIE_BASE | sie_flags;
        busy_wait_at_least_cycles(12);
        usb_hw->sie_ctrl = SIE_BASE | sie_flags | USB_SIE_CTRL_START_TRANS_BITS;
    }

    void __not_in_flash_func(iso_kick)()
    {
        usbh_dpram->epx_ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER |
                               (TYPE_ISO << EP_CTRL_BUFFER_TYPE_LSB) | EPX_DATA_OFFSET;
        usbh_dpram->epx_buf_ctrl = s.iso_mps | USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LAST | USB_BUF_CTRL_DATA0_PID;
        usb_hw->dev_addr_ctrl = DEVICE_ADDRESS | ((uint32_t)(s.iso_ep & 0x0f) << USB_ADDR_ENDP_ENDPOINT_LSB);
        s.iso_busy = true;
        s.iso_kick_frame = usb_hw->sof_rd;
        start_epx(USB_SIE_CTRL_RECEIVE_DATA_BITS);
    }

    void __not_in_flash_func(usb_irq)()
    {
        uint32_t status = usb_hw->ints;
        uint32_t handled = 0;
        s.irqs = s.irqs + 1;

        if (status & USB_INTS_HOST_CONN_DIS_BITS)
        {
            handled |= USB_INTS_HOST_CONN_DIS_BITS;
            hw_clear_bits(&usb_hw->sie_status, USB_SIE_STATUS_SPEED_BITS);
            s.conn_changed = true;
            s.iso_on = false;
            s.iso_busy = false;
        }

        if (status & USB_INTS_STALL_BITS)
        {
            handled |= USB_INTS_STALL_BITS;
            hw_clear_bits(&usb_hw->sie_status, USB_SIE_STATUS_STALL_REC_BITS);
            s.stalled = true;
        }

        if (status & USB_INTS_BUFF_STATUS_BITS)
        {
            handled |= USB_INTS_BUFF_STATUS_BITS;
            uint32_t bufs = usb_hw->buf_status;

            if (bufs & 0x1)
            {
                hw_clear_bits(&usb_hw->buf_status, 0x1);
                uint16_t len = usbh_dpram->epx_buf_ctrl & USB_BUF_CTRL_LEN_MASK;

                if (s.iso_busy)
                {
                    ring_packet(epx_data(), len);
                    s.iso_busy = false;
                }
                else
                {
                    s.buf_len = len;
                    s.buf_done = true;
                }
            }

            uint32_t other = bufs & ~0x1u;
            if (other)
            {
                hw_clear_bits(&usb_hw->buf_status, other);
                s.unexpected = s.unexpected + 1;
                s.last_unexpected = other;
            }
        }

        if (status & USB_INTS_TRANS_COMPLETE_BITS)
        {
            handled |= USB_INTS_TRANS_COMPLETE_BITS;
            if (usb_hw->sie_ctrl & USB_SIE_CTRL_SEND_SETUP_BITS)
            {
                s.setup_done = true;
            }
            hw_clear_bits(&usb_hw->sie_status, USB_SIE_STATUS_TRANS_COMPLETE_BITS);
        }

        constexpr uint32_t ERRORS = USB_INTS_ERROR_RX_TIMEOUT_BITS | USB_INTS_ERROR_DATA_SEQ_BITS | USB_INTS_ERROR_CRC_BITS |
                                    USB_INTS_ERROR_BIT_STUFF_BITS | USB_INTS_ERROR_RX_OVERFLOW_BITS;
        if (status & ERRORS)
        {
            handled |= status & ERRORS;
            uint32_t sie = usb_hw->sie_status;
            hw_clear_bits(&usb_hw->sie_status, USB_SIE_STATUS_RX_TIMEOUT_BITS | USB_SIE_STATUS_DATA_SEQ_ERROR_BITS |
                                                   USB_SIE_STATUS_CRC_ERROR_BITS | USB_SIE_STATUS_BIT_STUFF_ERROR_BITS |
                                                   USB_SIE_STATUS_RX_OVERFLOW_BITS);

            if (s.iso_on || s.iso_busy)
            {
                if (status & USB_INTS_ERROR_RX_TIMEOUT_BITS)
                {
                    s.iso_timeouts = s.iso_timeouts + 1;
                }
                else
                {
                    s.iso_errors = s.iso_errors + 1;
                }
                s.iso_lost = true;
                s.iso_busy = false;
            }
            else
            {
                if (status & USB_INTS_ERROR_RX_TIMEOUT_BITS)
                {
                    s.timed_out = true;
                }
                else
                {
                    s.error = true;
                }
                s.error_status = sie;
            }
        }

        if (status & USB_INTS_HOST_SOF_BITS)
        {
            handled |= USB_INTS_HOST_SOF_BITS;
            uint32_t frame = usb_hw->sof_rd;
            s.sofs = s.sofs + 1;

            if (s.iso_on)
            {
                if (s.iso_busy && (((frame - s.iso_kick_frame) & 0x7ff) > 3))
                {
                    s.iso_stuck = s.iso_stuck + 1;
                    hw_set_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_STOP_TRANS_BITS);
                    s.iso_busy = false;
                    s.iso_lost = true;
                }
                if (!s.iso_busy)
                {
                    iso_kick();
                }
            }
        }

        uint32_t left = status & ~handled;
        if (left)
        {
            hw_clear_bits(&usb_hw->inte, left);
            s.unexpected = s.unexpected + 1;
            s.last_unexpected = left;
        }
    }

    class UvcTransportRp2350 : public UvcTransport
    {
    public:
        bool begin() override;
        void poll(uint64_t now_us) override;
        bool device_present() const override { return m_bus == Bus::Ready; }
        bool full_speed() const override { return m_speed == 2; }
        std::span<const uint8_t> config_descriptor() const override { return std::span<const uint8_t>(m_config, m_config_len); }
        bool claim(uint8_t, uint8_t) override { return m_bus == Bus::Ready; }
        bool control(const UsbSetup& setup, uint8_t *data) override;
        bool set_alt(uint8_t interface, uint8_t alt) override;
        UsbOp op_state(uint32_t *len) const override;
        bool start_iso(uint8_t endpoint, uint16_t packet_size, UvcPacketSink *sink) override;
        void stop_iso() override;
        bool iso_running() const override { return s.iso_on || s.iso_busy; }
        void release() override { stop_iso(); }

        void debug(std::string_view args, char *out, uint32_t len);

    private:
        enum class Bus : uint8_t
        {
            Off, WaitConnect, Debounce, ResetBus, Settle, GetDevice8, SetAddress, AddressWait,
            GetDevice18, GetConfig9, GetConfig, SetConfig, Ready, Failed,
        };

        enum class Stage : uint8_t { Idle, Setup, Data, Status, Done, Failed };

        void start_host();
        void enter(Bus bus);
        void bus_step();
        void drain();
        void stats();

        void ctl_begin(const UsbSetup& setup, uint8_t *data);
        void ctl_step();
        void ctl_fail(const char *why);
        void ctl_data_packet();
        void ctl_status();
        bool ctl_busy() const { return (m_stage == Stage::Setup) || (m_stage == Stage::Data) || (m_stage == Stage::Status); }

        static const char *bus_name(Bus bus);

        Bus m_bus = Bus::Off;
        uint64_t m_now = 0;
        uint64_t m_bus_us = 0;
        uint64_t m_boot_us = 0;
        bool m_autostart = true;
        bool m_started = false;
        uint8_t m_speed = 0;
        uint8_t m_ep0_mps = 8;
        uint8_t m_address = 0;
        uint8_t m_enum_failures = 0;
        uint32_t m_ignored_changes = 0;

        Stage m_stage = Stage::Idle;
        UsbSetup m_setup = {};
        uint8_t *m_data = nullptr;
        uint16_t m_done = 0;
        uint16_t m_chunk = 0;
        bool m_pid1 = true;
        uint64_t m_stage_us = 0;
        bool m_quiet = false;

        uint8_t m_device[18] = {};
        uint8_t m_config[4096] = {};
        uint16_t m_config_len = 0;

        UvcPacketSink *m_sink = nullptr;
        uint32_t m_changes = 0;
        uint64_t m_changes_us = 0;
        uint64_t m_stats_us = 0;
        uint32_t m_last_packets = 0;
        uint64_t m_last_bytes = 0;
    };

    UvcTransportRp2350 s_transport;
}

UvcTransport& uvc_platform_transport()
{
    return s_transport;
}

void uvc_transport_debug(std::string_view args, char *out, uint32_t len)
{
    s_transport.debug(args, out, len);
}

const char *UvcTransportRp2350::bus_name(Bus bus)
{
    static const char *const NAMES[] = {
        "off", "wait-connect", "debounce", "reset-bus", "settle", "get-device-8", "set-address", "address-wait",
        "get-device-18", "get-config-9", "get-config", "set-config", "ready", "failed",
    };
    return NAMES[(int)bus];
}

bool UvcTransportRp2350::begin()
{
    restart_reason_t last = get_last_restart_reason();
    m_autostart = (last != RESTART_FAIL) && (last != RESTART_WATCHDOG) && (last != RESTART_HW_WATCHDOG) && (last != RESTART_FAULT);

    trace("UvcTransportRp2350::begin: m_autostart[%d]", m_autostart);
    return true;
}

void UvcTransportRp2350::start_host()
{
    if (m_started)
    {
        return;
    }
    m_started = true;

    memset((void *)&s, 0, sizeof(s));
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    memset((void *)usbh_dpram, 0, sizeof(*usbh_dpram));

    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS | USB_MAIN_CTRL_HOST_NDEVICE_BITS;
    usb_hw->sie_ctrl = SIE_BASE;
    usb_hw->inte = USB_INTE_BUFF_STATUS_BITS | USB_INTE_HOST_CONN_DIS_BITS | USB_INTE_STALL_BITS |
                   USB_INTE_TRANS_COMPLETE_BITS | USB_INTE_ERROR_RX_TIMEOUT_BITS | USB_INTE_ERROR_DATA_SEQ_BITS |
                   USB_INTE_ERROR_CRC_BITS | USB_INTE_ERROR_BIT_STUFF_BITS | USB_INTE_ERROR_RX_OVERFLOW_BITS;

    irq_add_shared_handler(USBCTRL_IRQ, usb_irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(USBCTRL_IRQ, true);

    trace("UvcTransportRp2350::start_host: main_ctrl=0x%08x sie_status=0x%08x, waiting for device", (unsigned)usb_hw->main_ctrl, (unsigned)usb_hw->sie_status);
    enter(Bus::WaitConnect);
}

void UvcTransportRp2350::enter(Bus bus)
{
    if (!m_quiet || (bus == Bus::Ready) || (bus == Bus::Failed) || (bus == Bus::WaitConnect))
    {
        trace("UvcTransportRp2350::enter: %s -> %s", bus_name(m_bus), bus_name(bus));
    }
    m_bus = bus;
    m_bus_us = m_now;
}

void UvcTransportRp2350::poll(uint64_t now_us)
{
    m_now = now_us;
    if (m_boot_us == 0)
    {
        m_boot_us = now_us;
    }

    if (!m_started)
    {
        if (m_autostart && (now_us - m_boot_us > AUTOSTART_US))
        {
            start_host();
        }
        return;
    }

    if (s.conn_changed)
    {
        s.conn_changed = false;
        uint8_t speed = (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) >> USB_SIE_STATUS_SPEED_LSB;

        if ((m_bus == Bus::ResetBus) || (m_bus == Bus::Settle) || (m_bus == Bus::Debounce))
        {
            ++m_ignored_changes;
        }
        else
        {
            if (m_now - m_changes_us > 60000000)
            {
                if (m_changes > CHANGES_LOGGED)
                {
                    trace("UvcTransportRp2350::poll: %u connection changes", (unsigned)(m_changes - CHANGES_LOGGED));
                }
                m_changes = 0;
                m_changes_us = m_now;
            }
            if (++m_changes <= CHANGES_LOGGED)
            {
                trace("UvcTransportRp2350::poll: connection change: speed %u (%s), sie_status=0x%08x%s", speed, (speed == 2) ? "full" : (speed == 1) ? "low" : "disconnected", (unsigned)usb_hw->sie_status, (m_changes == CHANGES_LOGGED) ? " (more this minute)" : "");
            }

            stop_iso();
            m_stage = Stage::Idle;
            m_address = 0;
            m_speed = speed;
            if (speed == 0)
            {
                enter(Bus::WaitConnect);
            }
            else
            {
                m_enum_failures = 0;
                enter(Bus::Debounce);
            }
        }
    }

    if (ctl_busy())
    {
        ctl_step();
    }

    bus_step();
    drain();
    stats();
}

void UvcTransportRp2350::bus_step()
{
    uint64_t in_state = m_now - m_bus_us;

    switch (m_bus)
    {
        case Bus::Off:
        case Bus::Ready:
            break;

        case Bus::WaitConnect:
        {
            uint8_t speed = (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) >> USB_SIE_STATUS_SPEED_LSB;
            if ((speed != 0) && (in_state > 200000))
            {
                trace("UvcTransportRp2350::bus_step: device already there (speed %u)", speed);
                m_speed = speed;
                enter(Bus::Debounce);
            }
            break;
        }

        case Bus::Debounce:
        {
            if (in_state > 100000)
            {
                m_ignored_changes = 0;
                enter(Bus::ResetBus);
                hw_set_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_RESET_BUS_BITS);
            }
            break;
        }
        case Bus::ResetBus:
        {
            if (in_state > 50000)
            {
                trace("UvcTransportRp2350::bus_step: bus reset done, sie_ctrl=0x%08x sie_status=0x%08x, %u connection changes during it", (unsigned)usb_hw->sie_ctrl, (unsigned)usb_hw->sie_status, (unsigned)m_ignored_changes);
                usb_hw->sie_ctrl = SIE_BASE;
                enter(Bus::Settle);
            }
            break;
        }
        case Bus::Settle:
        {
            if (in_state > 20000)
            {
                s.conn_changed = false;
                uint8_t speed = (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) >> USB_SIE_STATUS_SPEED_LSB;
                if (speed == 0)
                {
                    trace("UvcTransportRp2350::bus_step: no device after the bus reset (sie_status=0x%08x)", (unsigned)usb_hw->sie_status);
                    enter(Bus::WaitConnect);
                    break;
                }
                m_speed = speed;
                m_address = 0;
                m_ep0_mps = 8;
                UsbSetup get = { 0x80, 0x06, 0x0100, 0, 8 };
                ctl_begin(get, m_device);
                enter(Bus::GetDevice8);
            }
            break;
        }
        case Bus::GetDevice8:
        {
            if (m_stage == Stage::Done)
            {
                m_ep0_mps = m_device[7] ? m_device[7] : 8;
                trace("UvcTransportRp2350::bus_step: device descriptor head: usb %x.%02x class %02x ep0 %u bytes", m_device[3], m_device[2], m_device[4], m_ep0_mps);
                UsbSetup set_address = { 0x00, 0x05, DEVICE_ADDRESS, 0, 0 };
                ctl_begin(set_address, nullptr);
                enter(Bus::SetAddress);
            }
            break;
        }
        case Bus::SetAddress:
        {
            if (m_stage == Stage::Done)
            {
                m_address = DEVICE_ADDRESS;
                enter(Bus::AddressWait);
            }
            break;
        }
        case Bus::AddressWait:
        {
            if (in_state > 10000)
            {
                UsbSetup get = { 0x80, 0x06, 0x0100, 0, 18 };
                ctl_begin(get, m_device);
                enter(Bus::GetDevice18);
            }
            break;
        }
        case Bus::GetDevice18:
        {
            if (m_stage == Stage::Done)
            {
                trace("UvcTransportRp2350::bus_step: device %02x%02x:%02x%02x, release %x.%02x, %u configuration(s)", m_device[9], m_device[8], m_device[11], m_device[10], m_device[13], m_device[12], m_device[17]);
                UsbSetup get = { 0x80, 0x06, 0x0200, 0, 9 };
                ctl_begin(get, m_config);
                enter(Bus::GetConfig9);
            }
            break;
        }
        case Bus::GetConfig9:
        {
            if (m_stage == Stage::Done)
            {
                uint16_t total = (uint16_t)(m_config[2] | (m_config[3] << 8));
                if (total > sizeof(m_config))
                {
                    trace("UvcTransportRp2350::bus_step: configuration descriptor is %u bytes, more than %u", total, (unsigned)sizeof(m_config));
                    enter(Bus::Failed);
                    break;
                }
                UsbSetup get = { 0x80, 0x06, 0x0200, 0, total };
                ctl_begin(get, m_config);
                enter(Bus::GetConfig);
            }
            break;
        }
        case Bus::GetConfig:
        {
            if (m_stage == Stage::Done)
            {
                m_config_len = m_done;
                trace("UvcTransportRp2350::bus_step: configuration descriptor, %u bytes, %u interfaces, max power %u mA", m_config_len, m_config[4], m_config[8] * 2);
                UsbSetup set = { 0x00, 0x09, m_config[5], 0, 0 };
                ctl_begin(set, nullptr);
                enter(Bus::SetConfig);
            }
            break;
        }
        case Bus::SetConfig:
        {
            if (m_stage == Stage::Done)
            {
                m_stage = Stage::Idle;
                enter(Bus::Ready);
            }
            break;
        }
        case Bus::Failed:
        {
            if (in_state > 5000000)
            {
                uint8_t speed = (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS) >> USB_SIE_STATUS_SPEED_LSB;
                if ((speed != 0) && (m_enum_failures < 5))
                {
                    trace("UvcTransportRp2350::bus_step: trying enumeration again (%u failures)", m_enum_failures);
                    enter(Bus::Debounce);
                }
                else
                {
                    m_bus_us = m_now;
                }
            }
            break;
        }
        default:
        {
            trace("UvcTransportRp2350::bus_step: unknown state=%d", m_bus);
            break;
        }
    }

    if ((m_stage == Stage::Failed) && (m_bus > Bus::Settle) && (m_bus < Bus::Ready))
    {
        ++m_enum_failures;
        m_stage = Stage::Idle;
        enter(Bus::Failed);
    }
}

void UvcTransportRp2350::ctl_begin(const UsbSetup& setup, uint8_t *data)
{
    m_setup = setup;
    m_data = data;
    m_done = 0;
    s.setup_done = false;
    s.buf_done = false;
    s.stalled = false;
    s.timed_out = false;
    s.error = false;

    uint8_t raw[8] = {
        setup.bmRequestType, setup.bRequest,
        (uint8_t)setup.wValue, (uint8_t)(setup.wValue >> 8),
        (uint8_t)setup.wIndex, (uint8_t)(setup.wIndex >> 8),
        (uint8_t)setup.wLength, (uint8_t)(setup.wLength >> 8),
    };
    for (int i = 0; i < 8; ++i)
    {
        usbh_dpram->setup_packet[i] = raw[i];
    }

    usbh_dpram->epx_ctrl = EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER | (TYPE_CONTROL << EP_CTRL_BUFFER_TYPE_LSB) | EPX_DATA_OFFSET;
    usb_hw->dev_addr_ctrl = m_address;

    m_stage = Stage::Setup;
    m_stage_us = m_now;
    start_epx(USB_SIE_CTRL_SEND_SETUP_BITS);
}

void UvcTransportRp2350::ctl_data_packet()
{
    bool in = m_setup.bmRequestType & 0x80;
    uint16_t left = m_setup.wLength - m_done;
    m_chunk = (left < m_ep0_mps) ? left : m_ep0_mps;

    uint32_t buf = m_chunk | USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LAST | (m_pid1 ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID);
    if (!in)
    {
        memcpy(epx_data(), m_data + m_done, m_chunk);
        buf |= USB_BUF_CTRL_FULL;
    }

    s.buf_done = false;
    usbh_dpram->epx_buf_ctrl = buf;
    m_stage = Stage::Data;
    m_stage_us = m_now;
    start_epx(in ? USB_SIE_CTRL_RECEIVE_DATA_BITS : USB_SIE_CTRL_SEND_DATA_BITS);
}

void UvcTransportRp2350::ctl_status()
{
    bool in = (m_setup.bmRequestType & 0x80) && (m_setup.wLength > 0);
    uint32_t buf = USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LAST | USB_BUF_CTRL_DATA1_PID | (in ? USB_BUF_CTRL_FULL : 0);

    s.buf_done = false;
    usbh_dpram->epx_buf_ctrl = buf;
    m_stage = Stage::Status;
    m_stage_us = m_now;
    start_epx(in ? USB_SIE_CTRL_SEND_DATA_BITS : USB_SIE_CTRL_RECEIVE_DATA_BITS);
}

void UvcTransportRp2350::ctl_fail(const char *why)
{
    trace("usbh: control %02x %02x %04x %04x len %u failed in %s: %s (sie_status=0x%08x, %u bytes in)",
          m_setup.bmRequestType, m_setup.bRequest, m_setup.wValue, m_setup.wIndex, m_setup.wLength,
          (m_stage == Stage::Setup) ? "setup" : (m_stage == Stage::Data) ? "data" : "status",
          why, (unsigned)(s.error_status ? s.error_status : usb_hw->sie_status), m_done);

    hw_set_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_STOP_TRANS_BITS);
    s.error_status = 0;
    m_stage = Stage::Failed;
}

void UvcTransportRp2350::ctl_step()
{
    if (s.stalled)
    {
        ctl_fail("stall");
        return;
    }
    if (s.timed_out)
    {
        ctl_fail("rx timeout");
        return;
    }
    if (s.error)
    {
        ctl_fail("bus error");
        return;
    }
    if (m_now - m_stage_us > STAGE_TIMEOUT_US)
    {
        ctl_fail("no answer in 500 ms");
        return;
    }

    switch (m_stage)
    {
        case Stage::Setup:
        {
            if (s.setup_done)
            {
                m_pid1 = true;
                if (m_setup.wLength > 0)
                {
                    ctl_data_packet();
                }
                else
                {
                    ctl_status();
                }
            }
            break;
        }
        case Stage::Data:
        {
            if (s.buf_done)
            {
                bool in = m_setup.bmRequestType & 0x80;
                uint16_t got = in ? s.buf_len : m_chunk;
                if (in)
                {
                    memcpy(m_data + m_done, epx_data(), got);
                }
                m_done += got;
                m_pid1 = !m_pid1;

                if ((m_done >= m_setup.wLength) || (in && (got < m_ep0_mps)))
                {
                    ctl_status();
                }
                else
                {
                    ctl_data_packet();
                }
            }
            break;
        }
        case Stage::Status:
        {
            if (s.buf_done)
            {
                m_stage = Stage::Done;
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

bool UvcTransportRp2350::control(const UsbSetup& setup, uint8_t *data)
{
    if ((m_bus != Bus::Ready) || ctl_busy() || iso_running())
    {
        trace("UvcTransportRp2350::control: control refused: bus %s, %s, iso %s", bus_name(m_bus), ctl_busy() ? "busy" : "idle", iso_running() ? "on" : "off");
        return false;
    }
    ctl_begin(setup, data);
    return true;
}

bool UvcTransportRp2350::set_alt(uint8_t interface, uint8_t alt)
{
    trace("UvcTransportRp2350::set_alt: SET_INTERFACE %u alt %u", interface, alt);
    UsbSetup set = { 0x01, 0x0b, alt, interface, 0 };
    return control(set, nullptr);
}

UsbOp UvcTransportRp2350::op_state(uint32_t *len) const
{
    if (len != nullptr)
    {
        *len = m_done;
    }
    switch (m_stage)
    {
        case Stage::Done: return UsbOp::Done;
        case Stage::Failed: return UsbOp::Failed;
        case Stage::Idle: return UsbOp::Idle;
        default: return UsbOp::Pending;
    }
}

bool UvcTransportRp2350::start_iso(uint8_t endpoint, uint16_t packet_size, UvcPacketSink *sink)
{
    if ((m_bus != Bus::Ready) || ctl_busy() || iso_running())
    {
        return false;
    }

    m_sink = sink;
    uint32_t irq = save_and_disable_interrupts();
    s.ring_head = 0;
    s.ring_tail = 0;
    s.iso_lost = false;
    s.iso_ep = endpoint;
    s.iso_mps = (packet_size > 1023) ? 1023 : packet_size;
    restore_interrupts(irq);

    m_stats_us = m_now;
    m_last_packets = s.packets;
    m_last_bytes = s.bytes;

    s.iso_busy = false;
    s.iso_on = true;
    hw_set_bits(&usb_hw->inte, USB_INTE_HOST_SOF_BITS);

    trace("UvcTransportRp2350::start_iso: iso IN ep 0x%02x, %u bytes per frame", endpoint, s.iso_mps);
    return true;
}

void UvcTransportRp2350::stop_iso()
{
    if (!s.iso_on && !s.iso_busy)
    {
        return;
    }

    s.iso_on = false;
    hw_clear_bits(&usb_hw->inte, USB_INTE_HOST_SOF_BITS);
    if (s.iso_busy)
    {
        hw_set_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_STOP_TRANS_BITS);
        s.iso_busy = false;
    }

    drain();
    m_sink = nullptr;
    trace("UvcTransportRp2350::stop_iso: iso stopped: %u packets, %u KB, max %u B, %u empty, %u timeouts, %u errors, %u stuck, %u ring drops, ring peak %u",
          (unsigned)s.packets, (unsigned)(s.bytes / 1024), s.max_len, (unsigned)s.empty_packets, (unsigned)s.iso_timeouts,
          (unsigned)s.iso_errors, (unsigned)s.iso_stuck, (unsigned)s.ring_drops, (unsigned)s.ring_peak);
}

void UvcTransportRp2350::drain()
{
    static uint8_t packet[1024];

    while (ring_used() >= 2)
    {
        uint32_t head = s.ring_head;
        uint16_t len = (uint16_t)(s_ring[head & (RING_BYTES - 1)] | (s_ring[(head + 1) & (RING_BYTES - 1)] << 8));

        if (len == LOST)
        {
            s.ring_head = head + 2;
            if (m_sink != nullptr)
            {
                m_sink->on_iso_packet(nullptr, 0, false);
            }
            continue;
        }

        if (ring_used() < 2u + len)
        {
            break;
        }

        for (uint32_t i = 0; i < len; ++i)
        {
            packet[i] = s_ring[(head + 2 + i) & (RING_BYTES - 1)];
        }
        s.ring_head = head + 2 + len;

        if (m_sink != nullptr)
        {
            m_sink->on_iso_packet(packet, len, true);
        }
    }
}

void UvcTransportRp2350::stats()
{
    if (!iso_running() || (m_now - m_stats_us < STATS_US))
    {
        return;
    }

    uint32_t packets = s.packets - m_last_packets;
    uint64_t bytes = s.bytes - m_last_bytes;
    uint32_t ms = (uint32_t)((m_now - m_stats_us) / 1000);

    trace("UvcTransportRp2350::stats: iso %u packets/s, %u KB/s, max %u B, empty %u, timeouts %u, errors %u, stuck %u, ring drops %u, ring peak %u, sofs %u, irqs %u, unexpected %u (0x%x)",
          (unsigned)(packets * 1000 / ms), (unsigned)(bytes * 1000 / ms / 1024), s.max_len, (unsigned)s.empty_packets,
          (unsigned)s.iso_timeouts, (unsigned)s.iso_errors, (unsigned)s.iso_stuck, (unsigned)s.ring_drops,
          (unsigned)s.ring_peak, (unsigned)s.sofs, (unsigned)s.irqs, (unsigned)s.unexpected, (unsigned)s.last_unexpected);

    m_stats_us = m_now;
    m_last_packets = s.packets;
    m_last_bytes = s.bytes;
}

void UvcTransportRp2350::debug(std::string_view args, char *out, uint32_t len)
{
    if (args == "start")
    {
        start_host();
    }
    else if (args == "reset")
    {
        if (m_started)
        {
            stop_iso();
            m_stage = Stage::Idle;
            enter(Bus::Debounce);
        }
    }
    else if (!args.empty())
    {
        snprintf(out, len, "usb[/start|/reset]");
        return;
    }

    snprintf(out, len,
             "host %s, bus %s, speed %u, device %02x%02x:%02x%02x, config %u B, iso %s, packets %u (%u KB, max %u), "
             "empty %u, timeouts %u, errors %u, stuck %u, ring drops %u, irqs %u, unexpected %u (0x%x), sie_status 0x%08x",
             m_started ? "on" : (m_autostart ? "starting soon" : "off (usb/start)"), bus_name(m_bus), m_speed,
             m_device[9], m_device[8], m_device[11], m_device[10], m_config_len,
             iso_running() ? "on" : "off",
             (unsigned)s.packets, (unsigned)(s.bytes / 1024), s.max_len, (unsigned)s.empty_packets, (unsigned)s.iso_timeouts,
             (unsigned)s.iso_errors, (unsigned)s.iso_stuck, (unsigned)s.ring_drops, (unsigned)s.irqs, (unsigned)s.unexpected,
             (unsigned)s.last_unexpected, m_started ? (unsigned)usb_hw->sie_status : 0u);
}
