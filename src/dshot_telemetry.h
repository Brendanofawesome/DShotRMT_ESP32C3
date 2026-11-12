/**
 * @file dshot_telemetry.h
 * @brief Run-length, NRZI and GCR telemetry decoding.
 * @license MIT
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <driver/rmt_types.h>

namespace dshot_detail
{

inline bool decodeTelemetry(
    const rmt_symbol_word_t *symbols,
    size_t count,
    uint32_t resolution_hz,
    uint32_t bitrate,
    uint16_t &payload)
{
    if (!symbols || count < 2 || !resolution_hz || !bitrate)
        return false;

    const uint32_t telemetry_bitrate = bitrate * 5 / 4;

    uint32_t line_bits = 0;
    unsigned bits = 0;
    bool started = false;
    bool ended = false;

    for (size_t i = 0; i < count && bits < 21 && !ended; ++i)
    {
        for (unsigned half = 0; half < 2 && bits < 21; ++half)
        {
            const uint32_t duration =
                half ? symbols[i].duration1 : symbols[i].duration0;

            const unsigned level =
                half ? symbols[i].level1 : symbols[i].level0;

            if (!duration)
            {
                ended = true;
                break;
            }

            // RX inversion makes idle zero and the first response bit one.
            if (!started)
            {
                if (level == 0)
                    continue;

                started = true;
            }

            unsigned run =
                (static_cast<uint64_t>(duration) * telemetry_bitrate
                 + resolution_hz / 2) / resolution_hz;

            if (!run)
                return false;

            const unsigned remaining = 21 - bits;

            // Interior runs have at most three bits. A final zero run
            // may merge into the idle interval.
            if (run > 3 && !(level == 0 && remaining <= 3))
                return false;

            if (run > remaining)
            {
                if (level != 0)
                    return false;

                run = remaining;
            }

            line_bits <<= run;

            if (level)
                line_bits |= (1u << run) - 1u;

            bits += run;
        }
    }

    // Hardware may omit a final zero run that merges into idle.
    if (!started || bits < 18)
        return false;

    line_bits <<= 21 - bits;

    const uint32_t gcr =
        (line_bits ^ (line_bits >> 1)) & 0xFFFFFu;

    static constexpr uint8_t decode[32] = {
        255, 255, 255, 255, 255, 255, 255, 255,
        255,   9,  10,  11, 255,  13,  14,  15,
        255, 255,   2,   3, 255,   5,   6,   7,
        255,   0,   8,   1, 255,   4,  12, 255
    };

    uint16_t frame = 0;

    for (unsigned i = 0; i < 4; ++i)
    {
        const uint8_t nibble = decode[(gcr >> (5 * i)) & 31];

        if (nibble == 255)
            return false;

        frame |= static_cast<uint16_t>(nibble) << (4 * i);
    }

    const uint16_t data = frame >> 4;

    const uint8_t checksum =
        (~(data ^ (data >> 4) ^ (data >> 8))) & 15;

    if ((frame & 15) != checksum)
        return false;

    payload = data;
    return true;
}

inline bool electricalRpm(uint16_t payload, uint32_t &erpm)
{
    if (payload == 0xFFF)
    {
        erpm = 0;
        return true;
    }

    const uint32_t period_us =
        (payload & 0x1FFu) << (payload >> 9);

    if (!period_us)
        return false;

    erpm = (60000000u + period_us / 2) / period_us;
    return true;
}

} // namespace dshot_detail