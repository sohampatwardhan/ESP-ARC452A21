#include "daikin_ir.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "daikin_frame.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "daikin_ir";

enum {
    DAIKIN_SYMBOL_COUNT = 292,
};

static const rmt_symbol_word_t PREAMBLE_SYMBOLS[] = {
    {.level0 = 1, .duration0 = 531, .level1 = 0, .duration1 = 330},
    {.level0 = 1, .duration0 = 510, .level1 = 0, .duration1 = 356},
    {.level0 = 1, .duration0 = 507, .level1 = 0, .duration1 = 359},
    {.level0 = 1, .duration0 = 504, .level1 = 0, .duration1 = 362},
    {.level0 = 1, .duration0 = 500, .level1 = 0, .duration1 = 389},
    {.level0 = 1, .duration0 = 474, .level1 = 0, .duration1 = 25076},
};

static const rmt_symbol_word_t SECTION_LEADER_SYMBOL = {
    .level0 = 1,
    .duration0 = 3462,
    .level1 = 0,
    .duration1 = 1728,
};

static const rmt_symbol_word_t SECTION_GAP_SYMBOL = {
    .level0 = 1,
    .duration0 = 437,
    .level1 = 0,
    .duration1 = 2743,
};

static const rmt_symbol_word_t BIT_ZERO_SYMBOL = {
    .level0 = 1,
    .duration0 = 438,
    .level1 = 0,
    .duration1 = 428,
};

static const rmt_symbol_word_t BIT_ONE_SYMBOL = {
    .level0 = 1,
    .duration0 = 438,
    .level1 = 0,
    .duration1 = 1294,
};

static const rmt_symbol_word_t TERMINAL_MARK_SYMBOL = {
    .level0 = 1,
    .duration0 = 439,
    .level1 = 0,
    .duration1 = 0,
};

static rmt_channel_handle_t s_tx_channel;
static rmt_encoder_handle_t s_copy_encoder;
static uint8_t s_repeat_count = 1;
static uint32_t s_repeat_gap_ms;

static void log_section_3(const daikin_frame_t *frame)
{
    printf("DAIKIN_TX_SECTION_3");
    for (size_t i = 0; i < DAIKIN_FRAME_SECTION_3_LEN; ++i) {
        printf(",%02X", frame->section_3[i]);
    }
    printf("\n");
}

static void append_symbol(rmt_symbol_word_t symbols[DAIKIN_SYMBOL_COUNT],
                          size_t *count,
                          rmt_symbol_word_t symbol)
{
    symbols[*count] = symbol;
    ++(*count);
}

static void append_bytes_lsb(rmt_symbol_word_t symbols[DAIKIN_SYMBOL_COUNT],
                             size_t *count,
                             const uint8_t *bytes,
                             size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        for (uint8_t bit = 0; bit < 8; ++bit) {
            append_symbol(symbols, count,
                          (bytes[i] & (1U << bit)) ? BIT_ONE_SYMBOL : BIT_ZERO_SYMBOL);
        }
    }
}

static esp_err_t build_symbols(const daikin_frame_t *frame,
                               rmt_symbol_word_t symbols[DAIKIN_SYMBOL_COUNT],
                               size_t *symbol_count)
{
    size_t count = 0;
    for (size_t i = 0; i < sizeof(PREAMBLE_SYMBOLS) / sizeof(PREAMBLE_SYMBOLS[0]); ++i) {
        append_symbol(symbols, &count, PREAMBLE_SYMBOLS[i]);
    }

    append_symbol(symbols, &count, SECTION_LEADER_SYMBOL);
    append_bytes_lsb(symbols, &count, frame->section_1, DAIKIN_FRAME_SECTION_1_LEN);

    append_symbol(symbols, &count, SECTION_GAP_SYMBOL);
    append_symbol(symbols, &count, SECTION_LEADER_SYMBOL);
    append_bytes_lsb(symbols, &count, frame->section_2, DAIKIN_FRAME_SECTION_2_LEN);

    append_symbol(symbols, &count, SECTION_GAP_SYMBOL);
    append_symbol(symbols, &count, SECTION_LEADER_SYMBOL);
    append_bytes_lsb(symbols, &count, frame->section_3, DAIKIN_FRAME_SECTION_3_LEN);

    append_symbol(symbols, &count, TERMINAL_MARK_SYMBOL);

    *symbol_count = count;
    ESP_RETURN_ON_FALSE(count == DAIKIN_SYMBOL_COUNT, ESP_ERR_INVALID_SIZE, TAG,
                        "internal symbol count mismatch");
    return ESP_OK;
}

esp_err_t daikin_ir_init(const daikin_ir_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is required");

    rmt_tx_channel_config_t tx_config = {
        .gpio_num = config->tx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config->resolution_hz,
        .mem_block_symbols = 128,
        .trans_queue_depth = 4,
        .flags.invert_out = config->invert_out,
        .flags.with_dma = false,
    };

    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_config, &s_tx_channel), TAG,
                        "failed to create RMT TX channel");

    rmt_carrier_config_t carrier_config = {
        .frequency_hz = config->carrier_hz,
        .duty_cycle = config->carrier_duty_cycle,
        .flags.polarity_active_low = config->carrier_active_low,
    };

    ESP_RETURN_ON_ERROR(rmt_apply_carrier(s_tx_channel, &carrier_config), TAG,
                        "failed to apply RMT carrier");

    rmt_copy_encoder_config_t encoder_config = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&encoder_config, &s_copy_encoder), TAG,
                        "failed to create RMT copy encoder");

    ESP_RETURN_ON_ERROR(rmt_enable(s_tx_channel), TAG, "failed to enable RMT TX channel");

    s_repeat_count = config->repeat_count == 0 ? 1 : config->repeat_count;
    s_repeat_gap_ms = config->repeat_gap_ms;

    ESP_LOGI(TAG,
             "RMT TX ready on GPIO %d with %" PRIu32
             " Hz carrier, invert_out=%d carrier_active_low=%d repeat=%u gap=%" PRIu32 " ms",
             config->tx_gpio,
             config->carrier_hz,
             config->invert_out,
             config->carrier_active_low,
             (unsigned)s_repeat_count,
             s_repeat_gap_ms);
    return ESP_OK;
}

esp_err_t daikin_ir_send_state(const daikin_state_t *state)
{
    ESP_RETURN_ON_FALSE(state != NULL, ESP_ERR_INVALID_ARG, TAG, "state is required");
    ESP_RETURN_ON_FALSE(s_tx_channel != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "RMT TX channel is not initialized");
    ESP_RETURN_ON_FALSE(s_copy_encoder != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "RMT copy encoder is not initialized");

    daikin_frame_t frame;
    ESP_RETURN_ON_ERROR(daikin_frame_build(state, &frame), TAG,
                        "failed to build Daikin payload");
    log_section_3(&frame);

    rmt_symbol_word_t symbols[DAIKIN_SYMBOL_COUNT];
    size_t symbol_count = 0;
    ESP_RETURN_ON_ERROR(build_symbols(&frame, symbols, &symbol_count), TAG,
                        "failed to build RMT symbols");

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };

    ESP_LOGI(TAG, "Sending Daikin frame: power=%s mode=%d temp=%u F fan=%d symbols=%u",
             state->power ? "on" : "off",
             state->mode,
             state->target_fahrenheit,
             state->fan,
             (unsigned)symbol_count);

    for (uint8_t i = 0; i < s_repeat_count; ++i) {
        ESP_RETURN_ON_ERROR(rmt_transmit(s_tx_channel, s_copy_encoder, symbols,
                                         symbol_count * sizeof(symbols[0]),
                                         &transmit_config),
                            TAG, "failed to transmit Daikin frame");
        ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(s_tx_channel, 1000), TAG,
                            "timed out waiting for Daikin frame transmit");

        if (i + 1 < s_repeat_count && s_repeat_gap_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(s_repeat_gap_ms));
        }
    }

    return ESP_OK;
}
