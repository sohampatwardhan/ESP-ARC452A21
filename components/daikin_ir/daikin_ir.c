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

typedef struct {
    const char *name;
    rmt_symbol_word_t preamble[6];
    rmt_symbol_word_t section_leader;
    rmt_symbol_word_t section_gap;
    rmt_symbol_word_t bit_zero;
    rmt_symbol_word_t bit_one;
    rmt_symbol_word_t terminal_mark;
} daikin_timing_config_t;

static const daikin_timing_config_t TIMING_PROFILES[] = {
    [DAIKIN_TIMING_PROFILE_CAPTURED] = {
        .name = "captured",
        .preamble = {
            {.level0 = 1, .duration0 = 531, .level1 = 0, .duration1 = 330},
            {.level0 = 1, .duration0 = 510, .level1 = 0, .duration1 = 356},
            {.level0 = 1, .duration0 = 507, .level1 = 0, .duration1 = 359},
            {.level0 = 1, .duration0 = 504, .level1 = 0, .duration1 = 362},
            {.level0 = 1, .duration0 = 500, .level1 = 0, .duration1 = 389},
            {.level0 = 1, .duration0 = 474, .level1 = 0, .duration1 = 25076},
        },
        .section_leader = {.level0 = 1, .duration0 = 3462, .level1 = 0, .duration1 = 1728},
        .section_gap = {.level0 = 1, .duration0 = 437, .level1 = 0, .duration1 = 2743},
        .bit_zero = {.level0 = 1, .duration0 = 438, .level1 = 0, .duration1 = 428},
        .bit_one = {.level0 = 1, .duration0 = 438, .level1 = 0, .duration1 = 1294},
        .terminal_mark = {.level0 = 1, .duration0 = 439, .level1 = 0, .duration1 = 0},
    },
    [DAIKIN_TIMING_PROFILE_NOMINAL] = {
        .name = "nominal",
        .preamble = {
            {.level0 = 1, .duration0 = 531, .level1 = 0, .duration1 = 330},
            {.level0 = 1, .duration0 = 510, .level1 = 0, .duration1 = 356},
            {.level0 = 1, .duration0 = 507, .level1 = 0, .duration1 = 359},
            {.level0 = 1, .duration0 = 504, .level1 = 0, .duration1 = 362},
            {.level0 = 1, .duration0 = 500, .level1 = 0, .duration1 = 389},
            {.level0 = 1, .duration0 = 474, .level1 = 0, .duration1 = 25076},
        },
        .section_leader = {.level0 = 1, .duration0 = 3360, .level1 = 0, .duration1 = 1760},
        .section_gap = {.level0 = 1, .duration0 = 360, .level1 = 0, .duration1 = 32300},
        .bit_zero = {.level0 = 1, .duration0 = 360, .level1 = 0, .duration1 = 520},
        .bit_one = {.level0 = 1, .duration0 = 360, .level1 = 0, .duration1 = 1370},
        .terminal_mark = {.level0 = 1, .duration0 = 360, .level1 = 0, .duration1 = 0},
    },
};

static rmt_channel_handle_t s_tx_channel;
static rmt_encoder_handle_t s_copy_encoder;
static gpio_num_t s_tx_gpio = GPIO_NUM_NC;
static bool s_invert_out;
static uint8_t s_repeat_count = 1;
static uint32_t s_repeat_gap_ms;
static daikin_timing_profile_t s_timing_profile = DAIKIN_TIMING_PROFILE_NOMINAL;

static const daikin_timing_config_t *current_timing(void)
{
    return &TIMING_PROFILES[s_timing_profile];
}

static void log_section(const char *label, const uint8_t *section, size_t len)
{
    printf("DAIKIN_TX_%s", label);
    for (size_t i = 0; i < len; ++i) {
        printf(",%02X", section[i]);
    }
    printf("\n");
}

static void log_frame(const daikin_frame_t *frame)
{
    log_section("SECTION_1", frame->section_1, DAIKIN_FRAME_SECTION_1_LEN);
    log_section("SECTION_2", frame->section_2, DAIKIN_FRAME_SECTION_2_LEN);
    log_section("SECTION_3", frame->section_3, DAIKIN_FRAME_SECTION_3_LEN);
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
                             size_t len,
                             const daikin_timing_config_t *timing)
{
    for (size_t i = 0; i < len; ++i) {
        for (uint8_t bit = 0; bit < 8; ++bit) {
            append_symbol(symbols, count,
                          (bytes[i] & (1U << bit)) ? timing->bit_one : timing->bit_zero);
        }
    }
}

static esp_err_t build_symbols(const daikin_frame_t *frame,
                               rmt_symbol_word_t symbols[DAIKIN_SYMBOL_COUNT],
                               size_t *symbol_count)
{
    const daikin_timing_config_t *timing = current_timing();
    size_t count = 0;
    for (size_t i = 0; i < sizeof(timing->preamble) / sizeof(timing->preamble[0]); ++i) {
        append_symbol(symbols, &count, timing->preamble[i]);
    }

    append_symbol(symbols, &count, timing->section_leader);
    append_bytes_lsb(symbols, &count, frame->section_1, DAIKIN_FRAME_SECTION_1_LEN, timing);

    append_symbol(symbols, &count, timing->section_gap);
    append_symbol(symbols, &count, timing->section_leader);
    append_bytes_lsb(symbols, &count, frame->section_2, DAIKIN_FRAME_SECTION_2_LEN, timing);

    append_symbol(symbols, &count, timing->section_gap);
    append_symbol(symbols, &count, timing->section_leader);
    append_bytes_lsb(symbols, &count, frame->section_3, DAIKIN_FRAME_SECTION_3_LEN, timing);

    append_symbol(symbols, &count, timing->terminal_mark);

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

    s_tx_gpio = config->tx_gpio;
    s_invert_out = config->invert_out;
    s_repeat_count = config->repeat_count == 0 ? 1 : config->repeat_count;
    s_repeat_gap_ms = config->repeat_gap_ms;

    ESP_LOGI(TAG,
             "RMT TX ready on GPIO %d with %" PRIu32
             " Hz carrier, invert_out=%d carrier_active_low=%d repeat=%u gap=%" PRIu32
             " ms timing=%s",
             config->tx_gpio,
             config->carrier_hz,
             config->invert_out,
             config->carrier_active_low,
             (unsigned)s_repeat_count,
             s_repeat_gap_ms,
             current_timing()->name);
    return ESP_OK;
}

esp_err_t daikin_ir_set_invert_out(bool invert_out)
{
    ESP_RETURN_ON_FALSE(s_tx_channel != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "RMT TX channel is not initialized");
    ESP_RETURN_ON_FALSE(s_tx_gpio != GPIO_NUM_NC, ESP_ERR_INVALID_STATE, TAG,
                        "RMT TX GPIO is not initialized");

    if (s_invert_out == invert_out) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(s_tx_channel, 1000), TAG,
                        "timed out waiting for pending Daikin transmit");
    ESP_RETURN_ON_ERROR(rmt_disable(s_tx_channel), TAG,
                        "failed to disable RMT TX channel");

    esp_err_t switch_err = rmt_tx_switch_gpio(s_tx_channel, s_tx_gpio, invert_out);
    esp_err_t enable_err = rmt_enable(s_tx_channel);
    ESP_RETURN_ON_ERROR(enable_err, TAG, "failed to re-enable RMT TX channel");
    ESP_RETURN_ON_ERROR(switch_err, TAG, "failed to switch RMT TX polarity");

    s_invert_out = invert_out;
    ESP_LOGI(TAG, "RMT TX output polarity changed: invert_out=%d", s_invert_out);
    return ESP_OK;
}

void daikin_ir_set_repeat(uint8_t repeat_count, uint32_t repeat_gap_ms)
{
    s_repeat_count = repeat_count == 0 ? 1 : repeat_count;
    s_repeat_gap_ms = repeat_gap_ms;
    ESP_LOGI(TAG, "RMT TX repeat changed: repeat=%u gap=%" PRIu32 " ms",
             (unsigned)s_repeat_count,
             s_repeat_gap_ms);
}

esp_err_t daikin_ir_set_timing_profile(daikin_timing_profile_t profile)
{
    ESP_RETURN_ON_FALSE(profile >= DAIKIN_TIMING_PROFILE_CAPTURED &&
                            profile <= DAIKIN_TIMING_PROFILE_NOMINAL,
                        ESP_ERR_INVALID_ARG, TAG, "invalid timing profile");

    s_timing_profile = profile;
    ESP_LOGI(TAG, "RMT TX timing profile changed: %s", current_timing()->name);
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
    log_frame(&frame);

    rmt_symbol_word_t symbols[DAIKIN_SYMBOL_COUNT];
    size_t symbol_count = 0;
    ESP_RETURN_ON_ERROR(build_symbols(&frame, symbols, &symbol_count), TAG,
                        "failed to build RMT symbols");

    rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };

    ESP_LOGI(TAG, "Sending Daikin frame: power=%s mode=%d temp=%u %c fan=%d symbols=%u timing=%s",
             state->power ? "on" : "off",
             state->mode,
             state->use_fahrenheit ? state->target_fahrenheit : state->target_celsius,
             state->use_fahrenheit ? 'F' : 'C',
             state->fan,
             (unsigned)symbol_count,
             current_timing()->name);

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
