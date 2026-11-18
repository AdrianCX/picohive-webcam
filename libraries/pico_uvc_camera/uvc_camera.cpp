/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "memory_stats.h"
#include "pico_logger.h"
#include "uvc_camera.h"

namespace
{
    constexpr uint64_t RETRY_US = 5000000;
    constexpr uint64_t LOG_WINDOW_US = 10000000;
    constexpr uint16_t FULL_SPEED_MAX_PACKET = 1023;
    constexpr uint16_t HIGH_SPEED_MAX_PACKET = 3072;

    uint32_t ms_between(uint64_t from_us, uint64_t to_us) { return (uint32_t)((to_us - from_us) / 1000); }
}

UvcCamera& UvcCamera::get()
{
    static UvcCamera instance;
    return instance;
}

bool UvcCamera::begin(const UvcCameraConfig& config)
{
    m_config = config;
    m_transport = &uvc_platform_transport();
    m_assembler.set_buffer(nullptr, 0);
    m_started = m_transport->begin();

    trace("UvcCamera::begin: want %ux%u, frame buffer taken while streaming (up to %u bytes), transport %s",
          m_config.width, m_config.height, (unsigned)UVC_FRAME_BUFFER_BYTES, m_started ? "up" : "failed");

    return m_started;
}

UvcCamera::State UvcCamera::state() const
{
    switch (m_step)
    {
        case Step::NoCamera: return State::NoCamera;
        case Step::Idle: return State::Idle;
        case Step::Warm: return State::Warm;
        case Step::Capturing: return State::Capturing;
        case Step::FrameReady: return State::FrameReady;
        case Step::Error: return State::Error;
        case Step::StopIso:
        case Step::AltZero: return State::Idle;
        default: return State::Starting;
    }
}

void UvcCamera::enter(Step step)
{
    m_step = step;
    m_step_us = m_now_us;
}

void UvcCamera::set_error(const char *why)
{
    snprintf(m_error, sizeof(m_error), "%s", why);
}

bool UvcCamera::request_frame(uint16_t width, uint16_t height)
{
    if (!m_started || (m_step == Step::NoCamera) || (m_step == Step::Error))
    {
        set_error(m_started ? "no camera" : "camera not started");
        return false;
    }

    if ((m_capture == Capture::Pending) || (m_capture == Capture::Ready))
    {
        set_error("busy");
        return false;
    }

    if (m_info.frame_count > 0)
    {
        m_want_frame = uvc_pick_frame(m_info, width ? width : m_config.width, height ? height : m_config.height);
    }

    m_requested = true;
    m_capture = Capture::Pending;
    m_request_us = m_now_us;
    m_error[0] = 0;

    if ((m_step == Step::Warm) && (m_want_frame != m_frame_info))
    {
        trace("UvcCamera: switching %ux%u -> %ux%u", m_frame_info->width, m_frame_info->height, m_want_frame->width, m_want_frame->height);
        m_transport->stop_iso();
        enter(Step::StopIso);
    }
    else if (m_step == Step::Warm)
    {
        m_assembler.arm();
        enter(Step::Capturing);
    }
    else if ((m_step == Step::Idle) || (m_step == Step::StopIso) || (m_step == Step::AltZero))
    {
        if (m_step == Step::Idle)
        {
            start_stream();
        }
    }

    return true;
}

void UvcCamera::cancel_request()
{
    if (m_capture == Capture::Pending)
    {
        m_requested = false;
        m_capture = Capture::None;
        m_assembler.disarm();

        if (m_step == Step::Capturing)
        {
            m_last_use_us = m_now_us;
            enter(Step::Warm);
        }
    }
}

std::span<const uint8_t> UvcCamera::frame() const
{
    if (m_capture != Capture::Ready)
    {
        return {};
    }
    return std::span<const uint8_t>(m_assembler.data(), m_assembler.size());
}

void UvcCamera::release_frame()
{
    if (m_capture == Capture::Ready)
    {
        m_assembler.release();
    }

    if (m_capture != Capture::Pending)
    {
        m_capture = Capture::None;
    }

    if (m_step == Step::FrameReady)
    {
        m_last_use_us = m_now_us;
        enter(Step::Warm);
    }

    if (m_free_pending)
    {
        free_buffer();
    }
}

void UvcCamera::fail_request(const char *why)
{
    set_error(why);
    if (log_failure())
    {
        trace("UvcCamera: capture failed: %s", why);
    }

    m_assembler.disarm();
    m_requested = false;

    if (m_capture == Capture::Pending)
    {
        m_capture = Capture::Failed;
        ++m_failures;
    }
}

bool UvcCamera::log_capture()
{
    if (m_window_us == 0)
    {
        m_window_us = m_now_us;
        return true;
    }

    ++m_window_captures;
    m_window_ms += m_last_capture_ms;
    m_window_bytes += m_assembler.size();
    if (m_assembler.size() > m_window_largest)
    {
        m_window_largest = m_assembler.size();
    }
    return false;
}

bool UvcCamera::log_failure()
{
    if (m_window_us == 0)
    {
        m_window_us = m_now_us;
        return true;
    }

    ++m_window_failures;
    return false;
}

void UvcCamera::log_window()
{
    if ((m_window_us == 0) || ((m_now_us - m_window_us) < LOG_WINDOW_US))
    {
        return;
    }

    if ((m_window_captures == 0) && (m_window_failures == 0))
    {
        m_window_us = 0;
        return;
    }

    const UvcStreamStats& s = m_assembler.stats();
    uint32_t captures = (m_window_captures > 0) ? m_window_captures : 1;
    trace("UvcCamera: last %u s: %u more frames (avg %u ms, avg %u bytes, largest %u), %u failed (last: %s); dropped so far %u error, %u overflow, %u invalid",
          (unsigned)((m_now_us - m_window_us) / 1000000), (unsigned)m_window_captures, (unsigned)(m_window_ms / captures),
          (unsigned)(m_window_bytes / captures), (unsigned)m_window_largest, (unsigned)m_window_failures,
          (m_window_failures > 0) ? m_error : "-", (unsigned)s.dropped_error, (unsigned)s.dropped_overflow, (unsigned)s.dropped_invalid);

    m_window_us = m_now_us;
    m_window_captures = 0;
    m_window_failures = 0;
    m_window_ms = 0;
    m_window_bytes = 0;
    m_window_largest = 0;
}

uint32_t UvcCamera::buffer_size_for(uint32_t largest_frame)
{
    uint64_t want = (uint64_t)largest_frame * (100 + UVC_FRAME_BUFFER_MARGIN_PERCENT) / 100;
    want = (want + 1023) & ~(uint64_t)1023;
    if (want < UVC_FRAME_BUFFER_MIN_BYTES)
    {
        want = UVC_FRAME_BUFFER_MIN_BYTES;
    }
    if (want > UVC_FRAME_BUFFER_BYTES)
    {
        want = UVC_FRAME_BUFFER_BYTES;
    }
    return (uint32_t)want;
}

bool UvcCamera::take_buffer(uint32_t bytes)
{
    if ((m_buffer != nullptr) && (bytes <= m_buffer_bytes))
    {
        return true;
    }

    uint32_t free_heap = getFreeHeap();
    uint32_t largest = getLargestFreeBlock();
    if ((free_heap < bytes + UVC_HEAP_RESERVE_BYTES) || (largest < bytes))
    {
        if (!m_grow_logged)
        {
            trace("UvcCamera: no room for a %u byte frame buffer: %u bytes free, largest block %u, %u kept free",
                  (unsigned)bytes, (unsigned)free_heap, (unsigned)largest, (unsigned)UVC_HEAP_RESERVE_BYTES);
            m_grow_logged = true;
        }
        return false;
    }

    uint8_t *buffer = static_cast<uint8_t *>(malloc(bytes));
    if (buffer == nullptr)
    {
        return false;
    }

    m_assembler.replace_buffer(buffer, bytes);

    free(m_buffer);
    m_buffer = buffer;
    m_buffer_bytes = bytes;
    return true;
}

void UvcCamera::check_buffer()
{
    const UvcStreamStats& s = m_assembler.stats();
    bool overflowed = (s.dropped_overflow != m_overflow_seen);
    m_overflow_seen = s.dropped_overflow;

    if (!overflowed && (m_assembler.measured_frames() < UVC_FRAME_BUFFER_CHECK_FRAMES))
    {
        return;
    }

    uint32_t largest = m_assembler.measured_largest();
    m_assembler.reset_measure();

    uint32_t want = buffer_size_for(largest);
    if (want > m_buffer_bytes)
    {
        uint32_t step = buffer_size_for(m_buffer_bytes);
        if (want < step)
        {
            want = step;
        }

        uint32_t old = m_buffer_bytes;
        if (take_buffer(want))
        {
            trace("UvcCamera: frame buffer %u -> %u bytes (a frame of %u bytes)", (unsigned)old, (unsigned)m_buffer_bytes, (unsigned)largest);
        }
        return;
    }

    if (overflowed && !m_grow_logged)
    {
        trace("UvcCamera: a frame of %u bytes does not fit the %u byte buffer, which can't grow (up to %u)",
              (unsigned)largest, (unsigned)m_buffer_bytes, (unsigned)UVC_FRAME_BUFFER_BYTES);
        m_grow_logged = true;
    }
}

void UvcCamera::free_buffer()
{
    if (m_capture == Capture::Ready)
    {
        m_free_pending = true;
        return;
    }

    m_assembler.replace_buffer(nullptr, 0);
    free(m_buffer);
    m_buffer = nullptr;
    m_buffer_bytes = 0;
    m_free_pending = false;
}

void UvcCamera::fail_step(const char *why)
{
    trace("UvcCamera: %s, retrying in %u s", why, (unsigned)(RETRY_US / 1000000));

    if (m_capture == Capture::Pending)
    {
        fail_request(why);
    }
    set_error(why);

    teardown();
    m_retry_us = m_now_us + RETRY_US;
    enter(Step::Error);
}

void UvcCamera::teardown()
{
    if (m_transport->iso_running())
    {
        m_transport->stop_iso();
    }
    m_transport->release();
    m_assembler.reset();
    free_buffer();
    m_frame_info = nullptr;
    m_alt = nullptr;
}

void UvcCamera::open_device()
{
    std::span<const uint8_t> desc = m_transport->config_descriptor();

    if (!uvc_parse_config(desc, m_info))
    {
        fail_step("no MJPEG video streaming in the descriptors");
        return;
    }

    m_frame_info = uvc_pick_frame(m_info, m_config.width, m_config.height);
    if ((m_want_frame == nullptr) || (m_want_frame < m_info.frames) || (m_want_frame >= m_info.frames + m_info.frame_count))
    {
        m_want_frame = m_frame_info;
    }
    m_probe_len = uvc_probe_length(m_info.bcd_uvc);

    trace("UvcCamera: UVC %x.%02x, control if %u, streaming if %u ep 0x%02x, %u MJPEG sizes, %u alt settings, %s speed",
          m_info.bcd_uvc >> 8, m_info.bcd_uvc & 0xff, m_info.control_interface, m_info.streaming_interface,
          m_info.endpoint, m_info.frame_count, m_info.alt_count, m_transport->full_speed() ? "full" : "high");

    for (uint8_t i = 0; i < m_info.alt_count; ++i)
    {
        trace("UvcCamera:   alt %u: %u bytes per (micro)frame", m_info.alts[i].alt, m_info.alts[i].packet_bytes);
    }

    if (!m_transport->claim(m_info.control_interface, m_info.streaming_interface))
    {
        fail_step("cannot claim the video interfaces");
        return;
    }

    trace("UvcCamera: using %ux%u (frame %u), default interval %u us",
          m_frame_info->width, m_frame_info->height, m_frame_info->index, (unsigned)(m_frame_info->default_interval / 10));

    enter(Step::Idle);

    if (m_requested)
    {
        start_stream();
    }
}

void UvcCamera::start_stream()
{
    m_stream_start_us = m_now_us;
    m_frame_info = m_want_frame ? m_want_frame : m_frame_info;

    m_probe = UvcProbe();
    m_probe.hint = 0x0001;
    m_probe.format = m_info.mjpeg_format;
    m_probe.frame = m_frame_info->index;
    m_probe.interval = m_frame_info->default_interval;

    if (m_config.fps != 0)
    {
        uint32_t interval = 10000000u / m_config.fps;
        if (interval > m_frame_info->max_interval)
        {
            interval = m_frame_info->max_interval;
        }
        if (interval > m_probe.interval)
        {
            m_probe.interval = interval;
        }
    }

    memset(m_probe_bytes, 0, sizeof(m_probe_bytes));
    uvc_probe_pack(m_probe, m_probe_bytes, m_probe_len);

    UsbSetup setup = uvc_probe_setup(true, false, m_info.streaming_interface, m_probe_len);
    if (!m_transport->control(setup, m_probe_bytes))
    {
        fail_step("probe SET_CUR would not start");
        return;
    }

    enter(Step::ProbeSet);
}

bool UvcCamera::wait_op(const char *what)
{
    UsbOp op = m_transport->op_state();

    if (op == UsbOp::Done)
    {
        return true;
    }

    if (op == UsbOp::Failed)
    {
        char why[48];
        snprintf(why, sizeof(why), "%s failed", what);
        fail_step(why);
        return false;
    }

    if (ms_between(m_step_us, m_now_us) > m_config.timeout_ms)
    {
        char why[48];
        snprintf(why, sizeof(why), "%s timed out", what);
        fail_step(why);
    }

    return false;
}

void UvcCamera::update(uint64_t now_us)
{
    m_now_us = now_us;

    if (!m_started)
    {
        return;
    }

    m_transport->poll(now_us);
    log_window();

    if (!m_transport->device_present())
    {
        if ((m_step != Step::NoCamera) && (m_step != Step::Error))
        {
            trace("UvcCamera: camera gone");
            if (m_capture == Capture::Pending)
            {
                fail_request("camera unplugged");
            }
            teardown();
        }
        else if ((m_step == Step::NoCamera) && (m_capture == Capture::Pending))
        {
            fail_request("no camera");
        }

        if (m_step != Step::NoCamera)
        {
            enter(Step::NoCamera);
        }
        return;
    }

    advance();

    if ((m_capture == Capture::Pending) && (ms_between(m_request_us, m_now_us) > m_config.timeout_ms))
    {
        fail_request("no good frame in time");

        if (m_step == Step::Capturing)
        {
            m_last_use_us = m_now_us;
            enter(Step::Warm);
        }
    }
}

void UvcCamera::advance()
{
    uint32_t len = 0;

    switch (m_step)
    {
        case Step::NoCamera:
        {
            open_device();
            break;
        }
        case Step::Error:
        {
            if (m_now_us >= m_retry_us)
            {
                enter(Step::NoCamera);
            }
            break;
        }
        case Step::Idle:
        {
            break;
        }
        case Step::ProbeSet:
        {
            if (wait_op("probe SET_CUR"))
            {
                memset(m_probe_bytes, 0, sizeof(m_probe_bytes));
                if (!m_transport->control(uvc_probe_setup(false, false, m_info.streaming_interface, m_probe_len), m_probe_bytes))
                {
                    fail_step("probe GET_CUR would not start");
                    break;
                }
                enter(Step::ProbeGet);
            }
            break;
        }
        case Step::ProbeGet:
        {
            if (wait_op("probe GET_CUR"))
            {
                m_transport->op_state(&len);
                if (!uvc_probe_unpack(m_probe_bytes, len, m_probe))
                {
                    fail_step("probe GET_CUR came back short");
                    break;
                }

                trace("UvcCamera: negotiated frame %u interval %u us, max frame %u bytes, payload %u bytes",
                      m_probe.frame, (unsigned)(m_probe.interval / 10), (unsigned)m_probe.max_frame_size, (unsigned)m_probe.max_payload);

                if (!m_transport->control(uvc_probe_setup(true, true, m_info.streaming_interface, m_probe_len), m_probe_bytes))
                {
                    fail_step("commit SET_CUR would not start");
                    break;
                }
                enter(Step::CommitSet);
            }
            break;
        }
        case Step::CommitSet:
        {
            if (wait_op("commit SET_CUR"))
            {
                uint16_t cap = m_config.max_packet;
                if (cap == 0)
                {
                    cap = m_transport->full_speed() ? FULL_SPEED_MAX_PACKET : HIGH_SPEED_MAX_PACKET;
                }

                m_alt = uvc_pick_alt(m_info, m_probe.max_payload, cap);
                if (m_alt == nullptr)
                {
                    fail_step("no alternate setting fits");
                    break;
                }

                if (m_alt->packet_bytes < m_probe.max_payload)
                {
                    trace("UvcCamera: camera wants %u bytes per packet, the best fit is %u: expect lost frames",
                          (unsigned)m_probe.max_payload, m_alt->packet_bytes);
                }

                if (!m_transport->set_alt(m_info.streaming_interface, m_alt->alt))
                {
                    fail_step("set alt would not start");
                    break;
                }
                enter(Step::SetAlt);
            }
            break;
        }
        case Step::SetAlt:
        {
            if (wait_op("set alt"))
            {
                m_assembler.reset();
                m_assembler.reset_measure();
                if (!m_transport->start_iso(m_alt->endpoint, m_alt->packet_bytes, &m_assembler))
                {
                    fail_step("isochronous transfers would not start");
                    break;
                }

                trace("UvcCamera: streaming on alt %u (%u bytes), %u ms after start",
                      m_alt->alt, m_alt->packet_bytes, ms_between(m_stream_start_us, m_now_us));

                m_warmup_from = m_assembler.stats().frames_seen;
                enter(Step::Warmup);
            }
            break;
        }
        case Step::Warmup:
        {
            if (m_assembler.stats().frames_seen - m_warmup_from >= m_config.warmup_frames)
            {
                m_cold_start_ms = ms_between(m_stream_start_us, m_now_us);
                trace("UvcCamera: warm after %u frames, %u ms from cold", m_config.warmup_frames, m_cold_start_ms);

                uint32_t largest = m_assembler.measured_largest();
                m_grow_logged = false;
                if (!take_buffer(buffer_size_for(largest)))
                {
                    if (m_capture == Capture::Pending)
                    {
                        fail_request("not enough memory for a frame");
                    }
                    set_error("not enough memory for a frame");
                    m_transport->stop_iso();
                    enter(Step::StopIso);
                    break;
                }
                trace("UvcCamera: frame buffer %u bytes (largest of %u warm-up frames: %u)",
                      (unsigned)m_buffer_bytes, (unsigned)m_assembler.measured_frames(), (unsigned)largest);
                m_assembler.reset_measure();
                m_overflow_seen = m_assembler.stats().dropped_overflow;

                m_last_use_us = m_now_us;
                if (m_requested && (m_want_frame != m_frame_info))
                {
                    m_transport->stop_iso();
                    enter(Step::StopIso);
                }
                else if (m_requested)
                {
                    m_assembler.arm();
                    enter(Step::Capturing);
                }
                else
                {
                    enter(Step::Warm);
                }
            }
            else if (ms_between(m_step_us, m_now_us) > m_config.timeout_ms)
            {
                fail_step("no frames from the camera");
            }
            break;
        }
        case Step::Warm:
        {
            check_buffer();
            if ((ms_between(m_last_use_us, m_now_us) >= m_config.keep_warm_ms) && !m_requested)
            {
                m_transport->stop_iso();
                enter(Step::StopIso);
            }
            break;
        }
        case Step::Capturing:
        {
            check_buffer();
            if (m_assembler.done())
            {
                m_last_capture_ms = ms_between(m_request_us, m_now_us);
                ++m_captures;
                m_requested = false;
                m_capture = Capture::Ready;

                if (log_capture())
                {
                    const UvcStreamStats& s = m_assembler.stats();
                    trace("UvcCamera: %ux%u frame of %u bytes, %u ms after the request (dropped so far: %u error, %u overflow, %u invalid; largest %u)",
                          m_frame_info->width, m_frame_info->height, m_assembler.size(), m_last_capture_ms, (unsigned)s.dropped_error,
                          (unsigned)s.dropped_overflow, (unsigned)s.dropped_invalid, (unsigned)s.largest_frame);
                }

                enter(Step::FrameReady);
            }
            break;
        }
        case Step::FrameReady:
        {
            break;
        }
        case Step::StopIso:
        {
            if (!m_transport->iso_running())
            {
                if (!m_transport->set_alt(m_info.streaming_interface, 0))
                {
                    fail_step("set alt 0 would not start");
                    break;
                }
                enter(Step::AltZero);
            }
            else if (ms_between(m_step_us, m_now_us) > m_config.timeout_ms)
            {
                fail_step("isochronous transfers would not stop");
            }
            break;
        }
        case Step::AltZero:
        {
            if (wait_op("set alt 0"))
            {
                trace("UvcCamera: stream stopped, idle, frame buffer of %u bytes given back", (unsigned)m_buffer_bytes);
                free_buffer();
                enter(Step::Idle);
                if (m_requested)
                {
                    start_stream();
                }
            }
            break;
        }
    }
}

void UvcCamera::describe(char *out, uint32_t len) const
{
    static const char *const NAMES[] = { "no camera", "idle", "starting", "warm", "capturing", "frame ready", "error" };
    const UvcStreamStats& s = m_assembler.stats();

    snprintf(out, len,
             "%s%s%s, %ux%u, alt %u (%u B), buffer %u B, captures %u, failures %u, cold start %u ms, last capture %u ms, "
             "frames %u, largest %u B, dropped %u/%u/%u, packets %u (%u bad)",
             NAMES[(int)state()], m_error[0] ? ": " : "", m_error,
             m_frame_info ? m_frame_info->width : 0, m_frame_info ? m_frame_info->height : 0,
             m_alt ? m_alt->alt : 0, m_alt ? m_alt->packet_bytes : 0, (unsigned)m_buffer_bytes,
             (unsigned)m_captures, (unsigned)m_failures, (unsigned)m_cold_start_ms, (unsigned)m_last_capture_ms,
             (unsigned)s.frames_seen, (unsigned)s.largest_frame,
             (unsigned)s.dropped_error, (unsigned)s.dropped_overflow, (unsigned)s.dropped_invalid,
             (unsigned)s.packets, (unsigned)s.bad_packets);
}

void UvcCamera::list_sizes(char *out, uint32_t len) const
{
    uint32_t used = (uint32_t)snprintf(out, len, "sizes:");

    for (uint8_t i = 0; (i < m_info.frame_count) && (used < len); ++i)
    {
        used += (uint32_t)snprintf(out + used, len - used, " %ux%u", m_info.frames[i].width, m_info.frames[i].height);
    }
}
