/* SPDX-License-Identifier: MPL-2.0

Copyright (c) 2026 Adrian Cruceru - https://github.com/AdrianCX/picohive-firmware

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#pragma once

// Payload headers and frame reassembly. Packets stream in all the time the
// camera is on; only an armed assembler copies, and only from the start of a
// frame (after an EOF or an FID flip) to its end, into one caller-owned buffer.
// A frame with a damaged packet, the ERR bit, an overflow or no FF D8 ... FF D9
// is dropped and the next one taken.

#include <cstdint>

#include "uvc_transport.h"

struct UvcStreamStats
{
    uint32_t packets = 0;
    uint32_t empty_packets = 0;
    uint32_t bad_packets = 0;        // controller error or broken header
    uint32_t frames_seen = 0;        // boundaries, armed or not
    uint32_t frames_taken = 0;
    uint32_t dropped_error = 0;      // ERR bit or a damaged packet
    uint32_t dropped_overflow = 0;
    uint32_t dropped_invalid = 0;    // not a JPEG from start to end
    uint32_t largest_frame = 0;      // of the frames copied, kept or not
    uint64_t bytes = 0;
};

class UvcFrameAssembler : public UvcPacketSink
{
public:
    void set_buffer(uint8_t *buffer, uint32_t capacity);

    // Another buffer while streaming: the frame boundaries are kept, so the next
    // frame lands in it; a frame being copied right now is given up.
    void replace_buffer(uint8_t *buffer, uint32_t capacity);

    void arm();
    void disarm();
    void reset();

    bool armed() const { return m_armed; }
    bool done() const { return m_done; }
    uint32_t size() const { return m_done ? m_size : 0; }
    const uint8_t *data() const { return m_buffer; }

    // Frees the buffer for the next arm().
    void release() { m_done = false; m_size = 0; }

    // Every frame's size, copied or not (warm-up frames, or one too big for
    // the buffer): the largest since reset_measure(), to size a buffer by.
    uint32_t measured_largest() const { return m_measured_largest; }
    uint32_t measured_frames() const { return m_measured_frames; }
    void reset_measure() { m_measured_largest = 0; m_measured_frames = 0; }

    const UvcStreamStats& stats() const { return m_stats; }

    void on_iso_packet(const uint8_t *data, uint32_t len, bool ok) override;

private:
    void finish();
    void end_of_frame();

    uint8_t *m_buffer = nullptr;
    uint32_t m_capacity = 0;

    bool m_armed = false;
    bool m_done = false;
    bool m_copying = false;
    bool m_damaged = false;
    bool m_overflow = false;
    bool m_at_start = false;         // the next packet with data begins a frame
    int m_fid = -1;
    uint32_t m_size = 0;

    uint32_t m_frame_bytes = 0;      // of the frame going by, copied or not
    uint32_t m_measured_largest = 0;
    uint32_t m_measured_frames = 0;

    UvcStreamStats m_stats;
};

// MJPEG from webcams often leaves the Huffman tables out (they are the
// standard ones). Where they go: the offset to insert uvc_standard_dht() at,
// or 0 when the image has its own (or is not a JPEG).
uint32_t uvc_jpeg_dht_insert_offset(const uint8_t *jpeg, uint32_t len);
const uint8_t *uvc_standard_dht(uint32_t& len);
