/**
 * @file dshot_init.cpp
 * @brief RMT allocation helpers.
 * @license MIT
 */
#include "dshot_init.h"

dshot_result_t init_rmt_tx_channel(
    gpio_num_t gpio,
    rmt_channel_handle_t *out_channel,
    bool is_bidirectional)
{
    rmt_tx_channel_config_t config = {};
    config.gpio_num = gpio;
    config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    config.resolution_hz = DSHOT_RMT_RESOLUTION;
    config.mem_block_symbols = RMT_TX_BUFFER_SYMBOLS;
    config.trans_queue_depth = RMT_QUEUE_DEPTH;

    config.flags.invert_out = is_bidirectional;
    config.flags.io_loop_back = is_bidirectional;
    config.flags.io_od_mode = is_bidirectional;

    *out_channel = nullptr;

    if (rmt_new_tx_channel(&config, out_channel) != ESP_OK)
        return {false, DSHOT_TX_INIT_FAILED};

    if (rmt_enable(*out_channel) != ESP_OK)
    {
        // Enable failed: the channel remains in INIT.
        rmt_del_channel(*out_channel);
        *out_channel = nullptr;
        return {false, DSHOT_TX_INIT_FAILED};
    }

    return {true, DSHOT_TX_INIT_SUCCESS};
}

dshot_result_t init_rmt_rx_channel(
    gpio_num_t gpio,
    rmt_channel_handle_t *out_channel,
    rmt_rx_event_callbacks_t *callbacks,
    void *user_data)
{
    rmt_rx_channel_config_t config = {};
    config.gpio_num = gpio;
    config.clk_src = DSHOT_CLOCK_SRC_DEFAULT;
    config.resolution_hz = DSHOT_RMT_RESOLUTION;
    config.mem_block_symbols = RMT_RX_BUFFER_SYMBOLS;

    // Physical idle HIGH becomes logical zero.
    config.flags.invert_in = true;
    config.flags.io_loop_back = true;

    *out_channel = nullptr;

    if (rmt_new_rx_channel(&config, out_channel) != ESP_OK)
        return {false, DSHOT_RX_INIT_FAILED};

    if (rmt_rx_register_event_callbacks(
            *out_channel, callbacks, user_data) != ESP_OK)
    {
        rmt_del_channel(*out_channel);
        *out_channel = nullptr;
        return {false, DSHOT_CALLBACK_REGISTERING_FAILED};
    }

    if (rmt_enable(*out_channel) != ESP_OK)
    {
        rmt_del_channel(*out_channel);
        *out_channel = nullptr;
        return {false, DSHOT_RX_INIT_FAILED};
    }

    return {true, DSHOT_RX_INIT_SUCCESS};
}

dshot_result_t init_dshot_encoder(
    rmt_encoder_handle_t *out_encoder,
    const rmt_ticks_t &ticks,
    uint16_t pulse_level,
    uint16_t idle_level)
{
    // Symbols are now generated explicitly in DShotRMT::_encodeTxFrame().
    (void)ticks;
    (void)pulse_level;
    (void)idle_level;
    rmt_copy_encoder_config_t config = {};

    if (rmt_new_copy_encoder(&config, out_encoder) != ESP_OK)
        return {false, DSHOT_ENCODER_INIT_FAILED};

    return {true, DSHOT_ENCODER_INIT_SUCCESS};
}