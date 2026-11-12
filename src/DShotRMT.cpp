/**
 * @file DShotRMT.cpp
 * @brief DShot generation and bidirectional telemetry for ESP-IDF 5.
 * @license MIT
 */
#include "DShotRMT.h"
#include "dshot_telemetry.h"

#include <cmath>
#include <esp_timer.h>
#include <hal/gpio_ll.h>

DShotRMT::DShotRMT(
    gpio_num_t gpio,
    dshot_mode_t mode,
    bool is_bidirectional,
    uint16_t magnet_count)
    : _gpio(gpio),
      _mode(mode),
      _is_bidirectional(is_bidirectional),
      _motor_magnet_count(magnet_count)
{
    _preCalculateRMTTicks();
}

DShotRMT::DShotRMT(
    uint16_t pin_nr,
    dshot_mode_t mode,
    bool is_bidirectional,
    uint16_t magnet_count)
    : DShotRMT(
          static_cast<gpio_num_t>(pin_nr),
          mode,
          is_bidirectional,
          magnet_count)
{
}

DShotRMT::~DShotRMT()
{
    _cleanupRmtResources();
}

dshot_result_t DShotRMT::begin()
{
    if (_started)
        return {true, DSHOT_INIT_SUCCESS};

    // Dispose of resources left by an exceptional recovery failure.
    if (_rmt_tx_channel || _rmt_rx_channel || _rx_queue)
        _cleanupRmtResources();

    if (!_bitrate || !GPIO_IS_VALID_OUTPUT_GPIO(_gpio))
    {
        _last_error.store(ESP_ERR_INVALID_ARG);
        return {false, DSHOT_INIT_FAILED};
    }

    if (_is_bidirectional &&
        (_motor_magnet_count < 2 || (_motor_magnet_count & 1)))
    {
        _last_error.store(ESP_ERR_INVALID_ARG);
        return {false, DSHOT_INVALID_MAGNET_COUNT};
    }

    // RX first; TX must be the final owner of the GPIO output route.
    if (_is_bidirectional)
    {
        _rx_queue = xQueueCreate(1, sizeof(RxFrame));

        if (!_rx_queue)
        {
            _last_error.store(ESP_ERR_NO_MEM);
            return {false, DSHOT_RX_INIT_FAILED};
        }

        rmt_rx_event_callbacks_t callbacks = {};
        callbacks.on_recv_done = _on_rx_done;

        const auto result = init_rmt_rx_channel(
            _gpio,
            &_rmt_rx_channel,
            &callbacks,
            this);

        if (!result.success)
        {
            _last_error.store(ESP_FAIL);
            _cleanupRmtResources();
            return result;
        }

        _rx_enabled = true;

        _rx_config.signal_range_min_ns = DSHOT_PULSE_MIN_NS;
        _rx_config.signal_range_max_ns = DSHOT_PULSE_MAX_NS;
    }

    auto result = init_rmt_tx_channel(
        _gpio,
        &_rmt_tx_channel,
        _is_bidirectional);

    if (!result.success)
    {
        _last_error.store(ESP_FAIL);
        _cleanupRmtResources();
        return result;
    }

    _tx_enabled = true;

    rmt_tx_event_callbacks_t callbacks = {};
    callbacks.on_trans_done = _on_tx_done;

    const esp_err_t callback_error =
        rmt_tx_register_event_callbacks(
            _rmt_tx_channel,
            &callbacks,
            this);

    if (callback_error != ESP_OK)
    {
        _last_error.store(callback_error);
        _cleanupRmtResources();
        return {false, DSHOT_CALLBACK_REGISTERING_FAILED};
    }

    result = init_dshot_encoder(
        &_dshot_encoder,
        _rmt_ticks,
        1,
        0);

    if (!result.success)
    {
        _last_error.store(ESP_FAIL);
        _cleanupRmtResources();
        return result;
    }

    if (_is_bidirectional)
        gpio_pullup_en(_gpio);

    _last_error.store(ESP_OK);
    _started = true;

    return {true, DSHOT_INIT_SUCCESS};
}

void DShotRMT::_preCalculateRMTTicks()
{
    switch (_mode)
    {
    case DSHOT150:
        _bitrate = 150000;
        break;

    case DSHOT300:
        _bitrate = 300000;
        break;

    case DSHOT600:
        _bitrate = 600000;
        break;

    case DSHOT1200:
        _bitrate = 1200000;
        break;

    default:
        return;
    }

    // Match the pulse timings of the uploaded working driver at 10 MHz.
    static constexpr uint16_t bit_ticks[] = {0, 64, 32, 16, 8};
    _rmt_ticks.bit_length_ticks =
        static_cast<uint32_t>(bit_ticks[_mode]) * DSHOT_RMT_RESOLUTION / 10000000u;
    _rmt_ticks.t1h_ticks = _rmt_ticks.bit_length_ticks * 3u / 4u;
    _rmt_ticks.t0h_ticks = _rmt_ticks.bit_length_ticks * 3u / 8u;

    _rmt_ticks.t1l_ticks =
        _rmt_ticks.bit_length_ticks - _rmt_ticks.t1h_ticks;

    _rmt_ticks.t0l_ticks =
        _rmt_ticks.bit_length_ticks - _rmt_ticks.t0h_ticks;

    const uint32_t tx_us =
        (1000000u * 16 + _bitrate - 1) / _bitrate;

    const uint32_t telemetry_bitrate = _bitrate * 5 / 4;

    const uint32_t rx_us =
        (1000000u * 21 + telemetry_bitrate - 1) /
        telemetry_bitrate;

    _frame_timer_us = tx_us + DSHOT_PADDING_US;

    if (_is_bidirectional)
    {
        _frame_timer_us +=
            30 + rx_us + DSHOT_PULSE_MAX_NS / 1000;
    }
}

dshot_result_t DShotRMT::sendThrottle(uint16_t throttle)
{
    if (throttle != 0 &&
        (throttle < DSHOT_THROTTLE_MIN ||
         throttle > DSHOT_THROTTLE_MAX))
    {
        return {false, DSHOT_THROTTLE_NOT_IN_RANGE};
    }

    // A periodic stop refresh sends one frame.
    const auto result = _sendRawDshotFrame(throttle);

    if (result.success)
        _last_throttle = throttle;

    return result;
}

dshot_result_t DShotRMT::sendThrottlePercent(float percent)
{
    if (!std::isfinite(percent) ||
        percent < 0.0f ||
        percent > 100.0f)
    {
        return {false, DSHOT_PERCENT_NOT_IN_RANGE};
    }

    if (percent == 0.0f)
        return sendThrottle(0);

    const uint16_t throttle =
        DSHOT_THROTTLE_MIN +
        static_cast<uint16_t>(
            (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) *
                percent / 100.0f +
            0.5f);

    return sendThrottle(throttle);
}

dshot_result_t DShotRMT::sendCommand(uint16_t command)
{
    if (command > DSHOT_CMD_MAX)
        return {false, DSHOT_COMMAND_NOT_VALID};

    return sendCommand(static_cast<dshotCommands_e>(command));
}

dshot_result_t DShotRMT::sendCommand(dshotCommands_e command)
{
    uint16_t repeats = 1;

    switch (command)
    {
    case DSHOT_CMD_SPIN_DIRECTION_1:
    case DSHOT_CMD_SPIN_DIRECTION_2:
    case DSHOT_CMD_3D_MODE_OFF:
    case DSHOT_CMD_3D_MODE_ON:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE:
    case DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE:
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
        repeats = SETTINGS_COMMAND_REPEATS;
        break;

    default:
        break;
    }

    return sendCommand(command, repeats, 1000);
}

dshot_result_t DShotRMT::sendCommand(
    dshotCommands_e command,
    uint16_t repeats,
    uint16_t delay_us)
{
    return sendCustomCommand(
        static_cast<uint16_t>(command),
        repeats,
        delay_us);
}

dshot_result_t DShotRMT::sendCustomCommand(
    uint16_t command,
    uint16_t repeats,
    uint16_t delay_us)
{
    if (command > DSHOT_CMD_MAX || !repeats)
        return {false, DSHOT_INVALID_COMMAND};

    return _sendRepeatedCommand(command, repeats, delay_us);
}

dshot_result_t DShotRMT::setMotorSpinDirection(bool reversed)
{
    return sendCommand(
        reversed
            ? DSHOT_CMD_SPIN_DIRECTION_REVERSED
            : DSHOT_CMD_SPIN_DIRECTION_NORMAL);
}

dshot_result_t DShotRMT::saveESCSettings()
{
    return sendCommand(DSHOT_CMD_SAVE_SETTINGS);
}

dshot_result_t DShotRMT::_sendRepeatedCommand(
    uint16_t value,
    uint16_t repeats,
    uint16_t delay_us)
{
    for (uint16_t i = 0; i < repeats; ++i)
    {
        const auto result = _sendRawDshotFrame(value);

        if (!result.success)
            return result;

        if (i + 1 < repeats)
            delayMicroseconds(delay_us);
    }

    const auto result = _waitForFrameSlot();

    if (!result.success)
        return result;

    if (value >= DSHOT_CMD_BEACON1 &&
        value <= DSHOT_CMD_BEACON5)
    {
        delay(100);
    }
    else if (value == DSHOT_CMD_ESC_INFO)
    {
        delay(12);
    }
    else if (value == DSHOT_CMD_SAVE_SETTINGS)
    {
        delay(35);
    }

    if (value == DSHOT_CMD_MOTOR_STOP)
        _last_throttle = 0;

    return {true, DSHOT_COMMAND_SUCCESS};
}

uint16_t DShotRMT::_calculateCRC(uint16_t data) const
{
    const uint16_t crc =
        (data ^ (data >> 4) ^ (data >> 8)) & 15;

    return _is_bidirectional ? (crc ^ 15) : crc;
}

dshot_packet_t DShotRMT::_buildDShotPacket(uint16_t value) const
{
    dshot_packet_t packet = {};
    packet.throttle_value = value;

    // Commands use the telemetry-request bit. Normal bidirectional RPM
    // replies are requested by the inverted checksum.
    packet.telemetric_request =
        value > 0 && value < DSHOT_THROTTLE_MIN;

    packet.checksum = _calculateCRC(
        (value << 1) | packet.telemetric_request);

    return packet;
}

uint16_t DShotRMT::_buildDShotFrameValue(
    const dshot_packet_t &packet) const
{
    return (packet.throttle_value << 5) |
           (packet.telemetric_request << 4) |
           packet.checksum;
}

dshot_result_t DShotRMT::_sendRawDshotFrame(uint16_t value)
{
    return _sendPacket(_buildDShotPacket(value));
}

dshot_result_t DShotRMT::_waitForFrameSlot()
{
    if (!_started)
        return {false, DSHOT_INIT_FAILED};

    const int64_t deadline =
        esp_timer_get_time() + DSHOT_OPERATION_TIMEOUT_US;

    while (
        _tx_busy.load(std::memory_order_acquire) ||
        (_has_transmitted &&
         static_cast<uint64_t>(esp_timer_get_time()) -
                 _last_transmission_time_us <
             _frame_timer_us) ||
        (_rx_busy.load(std::memory_order_acquire) &&
         static_cast<uint32_t>(esp_timer_get_time()) -
                 _rx_started_us.load(std::memory_order_acquire) <
             _frame_timer_us))
    {
        if (esp_timer_get_time() >= deadline)
        {
            _last_error.store(ESP_ERR_TIMEOUT);
            return {false, _tx_busy.load()
                ? DSHOT_TRANSMISSION_FAILED : DSHOT_RECEIVER_FAILED};
        }

        delayMicroseconds(1);
    }

    const esp_err_t rx_error =
        _rx_start_error.exchange(ESP_OK);

    if (rx_error != ESP_OK)
    {
        _last_error.store(rx_error);
        return {false, DSHOT_RECEIVER_FAILED};
    }

    if (_rx_busy.load(std::memory_order_acquire))
    {
        // The response window expired. A missing telemetry reply must not
        // prevent the next throttle/stop frame from being transmitted.
        esp_err_t err = rmt_disable(_rmt_rx_channel);

        if (err != ESP_OK)
        {
            _last_error.store(err);
            return {false, DSHOT_RECEIVER_FAILED};
        }

        _rx_enabled = false;
        _rx_busy.store(false, std::memory_order_release);
        err = rmt_enable(_rmt_rx_channel);

        if (err != ESP_OK)
        {
            _last_error.store(err);
            _started = false;
            return {false, DSHOT_RECEIVER_FAILED};
        }

        _rx_enabled = true;
    }

    const esp_err_t err =
        rmt_tx_wait_all_done(_rmt_tx_channel, 0);

    if (err != ESP_OK)
    {
        _last_error.store(err);
        return {false, DSHOT_TRANSMISSION_FAILED};
    }

    return {true, DSHOT_NONE};
}

dshot_result_t DShotRMT::_sendPacket(
    const dshot_packet_t &packet)
{
    const auto ready = _waitForFrameSlot();

    if (!ready.success)
        return ready;

    _encoded_frame_value = _buildDShotFrameValue(packet);

    _encodeTxFrame();

    if (_is_bidirectional)
        gpio_ll_od_disable(&GPIO, _gpio);

    rmt_transmit_config_t config = {};

    // This is the level before GPIO-matrix inversion.
    // In bidirectional mode it produces physical HIGH.
    config.flags.eot_level = 0;

    _last_transmission_time_us = esp_timer_get_time();
    _has_transmitted = true;

    _last_error.store(ESP_OK);
    _tx_busy.store(true, std::memory_order_release);

    const esp_err_t err = rmt_transmit(
        _rmt_tx_channel,
        _dshot_encoder,
        _tx_symbols,
        sizeof(_tx_symbols),
        &config);

    if (err != ESP_OK)
    {
        _tx_busy.store(false);
        _last_error.store(err);

        if (_is_bidirectional)
            gpio_ll_od_enable(&GPIO, _gpio);

        return {false, DSHOT_TRANSMISSION_FAILED};
    }

    return {true, DSHOT_TRANSMISSION_SUCCESS};
}

void DShotRMT::_encodeTxFrame()
{
    for (unsigned i = 0; i < DSHOT_BITS_PER_FRAME; ++i)
    {
        const bool one = (_encoded_frame_value & (0x8000u >> i)) != 0;
        _tx_symbols[i].val = 0;
        _tx_symbols[i].duration0 = one ? _rmt_ticks.t1h_ticks : _rmt_ticks.t0h_ticks;
        _tx_symbols[i].level0 = 1;
        _tx_symbols[i].duration1 = one ? _rmt_ticks.t1l_ticks : _rmt_ticks.t0l_ticks;
        _tx_symbols[i].level1 = 0;
    }
}

bool IRAM_ATTR DShotRMT::_on_tx_done(
    rmt_channel_handle_t,
    const rmt_tx_done_event_data_t *,
    void *user_data)
{
    auto *self = static_cast<DShotRMT *>(user_data);

    if (self->_is_bidirectional)
    {
        // TX ended at physical HIGH. Open drain now releases the line.
        gpio_ll_od_enable(&GPIO, self->_gpio);

        self->_rx_started_us.store(
            static_cast<uint32_t>(esp_timer_get_time()), std::memory_order_release);
        self->_rx_busy.store(true, std::memory_order_release);

        // RX remains enabled between frames. rmt_receive() is ISR-safe.
        const esp_err_t err = rmt_receive(
            self->_rmt_rx_channel,
            self->_rx_symbols,
            sizeof(self->_rx_symbols),
            &self->_rx_config);

        if (err != ESP_OK)
        {
            self->_rx_start_error.store(err);
            self->_rx_busy.store(false, std::memory_order_release);
        }
    }

    self->_tx_busy.store(false, std::memory_order_release);
    return false;
}

bool IRAM_ATTR DShotRMT::_on_rx_done(
    rmt_channel_handle_t,
    const rmt_rx_done_event_data_t *edata,
    void *user_data)
{
    auto *self = static_cast<DShotRMT *>(user_data);
    BaseType_t wake = pdFALSE;

    if (edata &&
        edata->received_symbols &&
        edata->num_symbols > 1 &&
        edata->num_symbols <= RMT_RX_BUFFER_SYMBOLS)
    {
        RxFrame frame;
        frame.num_symbols = edata->num_symbols;

        // Copy the capture so subsequent receives cannot overwrite it.
        for (size_t i = 0; i < RMT_RX_BUFFER_SYMBOLS; ++i)
        {
            frame.symbols[i].val =
                i < frame.num_symbols
                    ? edata->received_symbols[i].val
                    : 0;
        }

        xQueueOverwriteFromISR(self->_rx_queue, &frame, &wake);
    }

    self->_rx_busy.store(false, std::memory_order_release);
    return wake == pdTRUE;
}

dshot_result_t DShotRMT::getTelemetry()
{
    dshot_result_t result = {false, DSHOT_TELEMETRY_FAILED};

    if (!_is_bidirectional)
    {
        result.result_code = DSHOT_BIDIR_NOT_ENABLED;
        return result;
    }

    if (!_started || !_rx_queue)
    {
        result.result_code = DSHOT_INIT_FAILED;
        return result;
    }

    RxFrame frame;

    if (xQueueReceive(_rx_queue, &frame, 0) != pdTRUE)
        return result;

    uint16_t payload;

    if (!dshot_detail::decodeTelemetry(
            frame.symbols,
            frame.num_symbols,
            DSHOT_RMT_RESOLUTION,
            _bitrate,
            payload))
    {
        return result;
    }

    if ((payload & 0x100) || (payload & 0xF00) == 0)
    {
        if (!dshot_detail::electricalRpm(payload, _last_erpm))
            return result;

        _telemetry_data.rpm = _last_erpm;
        _telemetry_fields |= DSHOT_TELEM_HAS_RPM;

        result.telemetry_type = 1;
        result.result_code = DSHOT_TELEMETRY_SUCCESS;
    }
    else
    {
        result.telemetry_type = payload >> 8;
        result.telemetry_value = payload & 255;

        switch (result.telemetry_type)
        {
        case 2:
            _telemetry_data.temperature = result.telemetry_value;
            _telemetry_fields |= DSHOT_TELEM_HAS_TEMPERATURE;
            break;

        case 4:
            _telemetry_data.voltage =
                result.telemetry_value * 250u;
            _telemetry_fields |= DSHOT_TELEM_HAS_VOLTAGE;
            break;

        case 6:
            _telemetry_data.current =
                result.telemetry_value * 1000u;
            _telemetry_fields |= DSHOT_TELEM_HAS_CURRENT;
            break;

        case 14:
            _telemetry_data.errors = result.telemetry_value;
            _telemetry_fields |= DSHOT_TELEM_HAS_STATUS;
            break;

        default:
            // Debug/stress samples are returned through type/value.
            break;
        }

        result.result_code = DSHOT_TELEMETRY_DATA_AVAILABLE;
    }

    result.success = true;
    result.erpm = _last_erpm;

    result.motor_rpm =
        static_cast<uint64_t>(_last_erpm) * 2 /
        _motor_magnet_count;

    result.telemetry_data = _telemetry_data;
    result.telemetry_fields = _telemetry_fields;

    result.telemetry_available =
        (_telemetry_fields & ~DSHOT_TELEM_HAS_RPM) != 0;

    return result;
}

void DShotRMT::_cleanupRmtResources()
{
    _started = false;

    // Stop TX first because its callback can start RX.
    if (_rmt_tx_channel)
    {
        if (_tx_enabled)
        {
            rmt_tx_wait_all_done(_rmt_tx_channel, 20);
            rmt_disable(_rmt_tx_channel);
            _tx_enabled = false;
        }

        rmt_del_channel(_rmt_tx_channel);
        _rmt_tx_channel = nullptr;
    }

    if (_rmt_rx_channel)
    {
        if (_rx_enabled)
        {
            rmt_disable(_rmt_rx_channel);
            _rx_enabled = false;
        }

        rmt_del_channel(_rmt_rx_channel);
        _rmt_rx_channel = nullptr;
    }

    if (_dshot_encoder)
    {
        rmt_del_encoder(_dshot_encoder);
        _dshot_encoder = nullptr;
    }

    if (_rx_queue)
    {
        vQueueDelete(_rx_queue);
        _rx_queue = nullptr;
    }

    _tx_busy.store(false);
    _rx_busy.store(false);
    _rx_start_error.store(ESP_OK);
    _has_transmitted = false;
}