/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#pragma once

// One JPEG from a USB webcam on request. Detection, negotiation, streaming,
// reassembly, warm-up and recovery are all inside; everything advances from
// update(), which never blocks.
//
//   on_boot():   UvcCamera::get().begin();
//   on_update(): UvcCamera::get().update(now_us);
//
//   request_frame(), then once capture() says Ready: frame(), release_frame().

#include <cstdint>
#include <span>

#include "general_config.h"
#include "uvc_control.h"
#include "uvc_descriptors.h"
#include "uvc_stream.h"
#include "uvc_transport.h"

// The one frame kept is on the heap only while the camera streams: sized at
// the largest warm-up frame plus UVC_FRAME_BUFFER_MARGIN_PERCENT, taken again
// bigger when frames keep coming larger (checked every
// UVC_FRAME_BUFFER_CHECK_FRAMES frames, or at once when one does not fit),
// freed when the stream stops. 320x240 or 640x480 MJPEG from a webcam is 10-30 KB.
#ifndef UVC_FRAME_BUFFER_BYTES
#define UVC_FRAME_BUFFER_BYTES (64 * 1024)          // the most it may grow to
#endif

#ifndef UVC_FRAME_BUFFER_MARGIN_PERCENT
#define UVC_FRAME_BUFFER_MARGIN_PERCENT 10
#endif

#ifndef UVC_FRAME_BUFFER_CHECK_FRAMES
#define UVC_FRAME_BUFFER_CHECK_FRAMES 8
#endif

#ifndef UVC_FRAME_BUFFER_MIN_BYTES
#define UVC_FRAME_BUFFER_MIN_BYTES (8 * 1024)
#endif

// Heap left for everything else (TLS, the network) when taking the buffer.
#ifndef UVC_HEAP_RESERVE_BYTES
#define UVC_HEAP_RESERVE_BYTES (48 * 1024)
#endif

struct UvcCameraConfig
{
    uint16_t width = 320;            // the closest MJPEG size the camera offers
    uint16_t height = 240;
    uint8_t fps = 0;                 // 0: the camera's default rate for that size
    uint8_t warmup_frames = 15;      // dropped after a cold start (auto exposure: 8 was still dark on a C922)
    uint32_t keep_warm_ms = 30000;   // keep streaming this long after a capture, 0 = stop at once
    uint32_t timeout_ms = 3000;      // give up on a request after this
    uint16_t max_packet = 0;         // cap on bytes per (micro)frame, 0: 1023 at full speed, any at high speed
};

class UvcCamera
{
public:
    static UvcCamera& get();

    enum class State : uint8_t { NoCamera, Idle, Starting, Warm, Capturing, FrameReady, Error };
    enum class Capture : uint8_t { None, Pending, Ready, Failed };

    bool begin(const UvcCameraConfig& config = UvcCameraConfig());
    void update(uint64_t now_us);

    // false: no camera, or a request is already running / its frame not yet released.
    // With a size: the closest MJPEG size the camera offers (0 x 0: the configured one).
    // A size other than the one streaming restarts the stream at that size.
    bool request_frame(uint16_t width = 0, uint16_t height = 0);
    void cancel_request();

    State state() const;
    Capture capture() const { return m_capture; }
    bool frame_ready() const { return m_capture == Capture::Ready; }
    std::span<const uint8_t> frame() const;
    void release_frame();

    const char *last_error() const { return m_error; }

    // Single-line status for a command reply / the log.
    void describe(char *out, uint32_t len) const;

    // "sizes: 160x120 320x240 ...", the MJPEG sizes the camera offers ("sizes:" with no camera).
    void list_sizes(char *out, uint32_t len) const;

    // The size the last frame was taken at.
    uint16_t width() const { return m_frame_info ? m_frame_info->width : 0; }
    uint16_t height() const { return m_frame_info ? m_frame_info->height : 0; }

    const UvcStreamStats& stream_stats() const { return m_assembler.stats(); }

    // The frame buffer's size now, 0 while not streaming.
    uint32_t buffer_bytes() const { return m_buffer_bytes; }

private:
    enum class Step : uint8_t
    {
        NoCamera,
        Idle,
        ProbeSet,
        ProbeGet,
        CommitSet,
        SetAlt,
        Warmup,
        Warm,
        Capturing,
        FrameReady,
        StopIso,
        AltZero,
        Error,
    };

    void open_device();
    void start_stream();
    void advance();
    bool wait_op(const char *what);
    void fail_step(const char *why);
    void fail_request(const char *why);
    void teardown();
    void set_error(const char *why);
    void enter(Step step);

    // Back to back captures (the page's live view) would log one line each:
    // the first of a run is traced in full, the rest are counted and summed
    // up once per LOG_WINDOW_US. Failures the same.
    bool log_capture();
    bool log_failure();
    void log_window();

    // The frame buffer: taken when warm-up ends, taken again bigger while
    // streaming (check_buffer), given back when the stream stops (once the
    // frame held is released).
    static uint32_t buffer_size_for(uint32_t largest_frame);
    bool take_buffer(uint32_t largest_frame);
    void check_buffer();
    void free_buffer();

    UvcTransport *m_transport = nullptr;
    UvcCameraConfig m_config;
    UvcDeviceInfo m_info;
    const UvcFrameInfo *m_frame_info = nullptr;     // what is streaming
    const UvcFrameInfo *m_want_frame = nullptr;     // what the next capture wants
    const UvcAltInfo *m_alt = nullptr;
    UvcProbe m_probe;
    uint8_t m_probe_bytes[UVC_PROBE_MAX_BYTES] = {};
    uint16_t m_probe_len = 26;

    UvcFrameAssembler m_assembler;

    Step m_step = Step::NoCamera;
    Capture m_capture = Capture::None;
    bool m_requested = false;
    bool m_started = false;

    uint64_t m_now_us = 0;
    uint64_t m_step_us = 0;
    uint64_t m_request_us = 0;
    uint64_t m_stream_start_us = 0;
    uint64_t m_last_use_us = 0;
    uint64_t m_retry_us = 0;
    uint32_t m_warmup_from = 0;

    // Measurements, for the log and describe().
    uint32_t m_cold_start_ms = 0;
    uint32_t m_last_capture_ms = 0;
    uint32_t m_captures = 0;
    uint32_t m_failures = 0;

    // The current log window (0: none open), see log_capture().
    uint64_t m_window_us = 0;
    uint32_t m_window_captures = 0;
    uint32_t m_window_failures = 0;
    uint64_t m_window_ms = 0;
    uint64_t m_window_bytes = 0;
    uint32_t m_window_largest = 0;

    uint8_t *m_buffer = nullptr;
    uint32_t m_buffer_bytes = 0;
    bool m_free_pending = false;       // stopped while a frame was held
    bool m_grow_logged = false;        // "can't grow" said once per stream
    uint32_t m_overflow_seen = 0;

    char m_error[64] = {};
};
