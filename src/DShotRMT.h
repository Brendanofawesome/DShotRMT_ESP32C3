/**
 * @file DShotRMT.h
 * @brief DShot generation and bidirectional telemetry using ESP-IDF 5 RMT.
 * @license MIT
 */
#pragma once

#include <atomic>
#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include <freertos/task.h>

#include "dshot_definitions.h"
#include "dshot_init.h"

static constexpr uint8_t DSHOTRMT_MAJOR_VERSION = 0;
static constexpr uint8_t DSHOTRMT_MINOR_VERSION = 9;
static constexpr uint8_t DSHOTRMT_PATCH_VERSION = 6;
static constexpr auto DSHOT_THROTTLE_FAILSAFE = 0;




template<typename T>
class DShotSharedValue
{
public:
    explicit DShotSharedValue(T initial = T{}) noexcept
        : _value(initial)
    {
    }

    DShotSharedValue(const DShotSharedValue &) = delete;
    DShotSharedValue &operator=(const DShotSharedValue &) = delete;

    // The memory-order argument preserves existing call syntax.
    // Synchronization is supplied by the critical section.
    inline __attribute__((always_inline))
    T load(
        std::memory_order = std::memory_order_seq_cst) const noexcept
    {
        portENTER_CRITICAL_SAFE(&_lock);
        const T result = _value;
        portEXIT_CRITICAL_SAFE(&_lock);
        return result;
    }

    inline __attribute__((always_inline))
    void store(
        T value,
        std::memory_order = std::memory_order_seq_cst) noexcept
    {
        portENTER_CRITICAL_SAFE(&_lock);
        _value = value;
        portEXIT_CRITICAL_SAFE(&_lock);
    }

    inline __attribute__((always_inline))
    T exchange(
        T value,
        std::memory_order = std::memory_order_seq_cst) noexcept
    {
        portENTER_CRITICAL_SAFE(&_lock);
        const T previous = _value;
        _value = value;
        portEXIT_CRITICAL_SAFE(&_lock);
        return previous;
    }

private:
    mutable portMUX_TYPE _lock = portMUX_INITIALIZER_UNLOCKED;
    T _value;
};

// Call each instance from one task, with interrupts enabled.
// Sending is asynchronous. The next send waits for the previous frame's slot.
class DShotRMT
{
public:
    DShotRMT(
        gpio_num_t gpio,
        dshot_mode_t mode = DSHOT300,
        bool is_bidirectional = false,
        uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    DShotRMT(
        uint16_t pin_nr,
        dshot_mode_t mode,
        bool is_bidirectional = false,
        uint16_t magnet_count = DEFAULT_MOTOR_MAGNET_COUNT);

    ~DShotRMT();

    DShotRMT(const DShotRMT &) = delete;
    DShotRMT &operator=(const DShotRMT &) = delete;

    dshot_result_t begin();

    dshot_result_t sendThrottle(uint16_t throttle);
    dshot_result_t sendThrottlePercent(float percent);

    dshot_result_t sendCommand(uint16_t command_value);
    dshot_result_t sendCommand(dshotCommands_e command);

    dshot_result_t sendCommand(
        dshotCommands_e command,
        uint16_t repeat_count,
        uint16_t delay_us);

    dshot_result_t sendCustomCommand(
        uint16_t command_value,
        uint16_t repeat_count,
        uint16_t delay_us);

    dshot_result_t getTelemetry();
    dshot_result_t setMotorSpinDirection(bool reversed);
    dshot_result_t saveESCSettings();

    dshot_mode_t getMode() const { return _mode; }
    bool isBidirectional() const { return _is_bidirectional; }
    uint16_t getThrottleValue() const { return _last_throttle; }
    uint16_t getEncodedFrameValue() const { return _encoded_frame_value; }

    esp_err_t getLastError() const
    {
        const esp_err_t pending = _rx_start_error.load();
        return pending != ESP_OK ? pending : _last_error.load();
    }

private:
    struct RxFrame
    {
        size_t num_symbols;
        rmt_symbol_word_t symbols[RMT_RX_BUFFER_SYMBOLS];
    };

    static bool IRAM_ATTR _on_tx_done(
        rmt_channel_handle_t channel,
        const rmt_tx_done_event_data_t *edata,
        void *user_data);

    static bool IRAM_ATTR _on_rx_done(
        rmt_channel_handle_t channel,
        const rmt_rx_done_event_data_t *edata,
        void *user_data);

    void _preCalculateRMTTicks();

    dshot_packet_t _buildDShotPacket(uint16_t value) const;
    uint16_t _buildDShotFrameValue(const dshot_packet_t &packet) const;
    uint16_t _calculateCRC(uint16_t data) const;

    dshot_result_t _sendRawDshotFrame(uint16_t value);
    dshot_result_t _sendPacket(const dshot_packet_t &packet);
    void _encodeTxFrame();
    dshot_result_t _waitForFrameSlot();

    dshot_result_t _sendRepeatedCommand(
        uint16_t value,
        uint16_t repeat_count,
        uint16_t delay_us);

    void _cleanupRmtResources();

    gpio_num_t _gpio;
    dshot_mode_t _mode;
    bool _is_bidirectional;
    uint16_t _motor_magnet_count;

    rmt_ticks_t _rmt_ticks = {};
    rmt_channel_handle_t _rmt_tx_channel = nullptr;
    rmt_channel_handle_t _rmt_rx_channel = nullptr;
    rmt_encoder_handle_t _dshot_encoder = nullptr;

    bool _started = false;
    bool _tx_enabled = false;
    bool _rx_enabled = false;

    // Persistent storage for asynchronous transactions.
    rmt_symbol_word_t _tx_symbols[DSHOT_BITS_PER_FRAME] = {};
    rmt_symbol_word_t _rx_symbols[RMT_RX_BUFFER_SYMBOLS] = {};
    rmt_receive_config_t _rx_config = {};
    QueueHandle_t _rx_queue = nullptr;

    // Word-sized atomics are lock-free on ESP32-C3.
    DShotSharedValue<uint32_t> _tx_busy{false};
    DShotSharedValue<uint32_t> _rx_busy{false};
    DShotSharedValue<uint32_t> _rx_started_us{0};
    DShotSharedValue<esp_err_t> _rx_start_error{ESP_OK};
    DShotSharedValue<esp_err_t> _last_error{ESP_OK};

    uint32_t _bitrate = 0;
    uint64_t _frame_timer_us = 0;
    uint64_t _last_transmission_time_us = 0;
    bool _has_transmitted = false;

    uint16_t _last_throttle = 0;
    uint16_t _encoded_frame_value = 0;

    uint32_t _last_erpm = 0;
    dshot_telemetry_data_t _telemetry_data = {};
    uint16_t _telemetry_fields = 0;
};

#include "dshot_utils.h"