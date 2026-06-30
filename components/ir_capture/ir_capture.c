#include "ir_capture.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/rmt_rx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "ir_capture";

static rmt_channel_handle_t s_rx_channel;
static QueueHandle_t s_receive_queue;
static rmt_receive_config_t s_receive_config;
static size_t s_symbols_capacity;
static rmt_symbol_word_t s_symbols[IR_CAPTURE_MAX_SYMBOLS];

enum {
    DAIKIN_CAPTURE_SYMBOL_COUNT = 292,
    DAIKIN_SECTION_1_LEN = 8,
    DAIKIN_SECTION_2_LEN = 8,
    DAIKIN_SECTION_3_LEN = 19,
    DAIKIN_BIT_ONE_SPACE_US = 800,
};

static bool ir_capture_on_receive_done(rmt_channel_handle_t channel,
                                       const rmt_rx_done_event_data_t *edata,
                                       void *user_data)
{
    (void)channel;

    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

static uint8_t ir_capture_decode_bit(const rmt_symbol_word_t *symbol)
{
    return symbol->duration1 > DAIKIN_BIT_ONE_SPACE_US ? 1 : 0;
}

static void ir_capture_decode_bytes_lsb(const rmt_symbol_word_t *symbols,
                                        size_t *position,
                                        uint8_t *bytes,
                                        size_t len)
{
    for (size_t byte_index = 0; byte_index < len; ++byte_index) {
        uint8_t value = 0;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            value |= (uint8_t)(ir_capture_decode_bit(&symbols[*position]) << bit);
            ++(*position);
        }
        bytes[byte_index] = value;
    }
}

static uint8_t ir_capture_checksum(const uint8_t *bytes, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = (uint8_t)(sum + bytes[i]);
    }
    return sum;
}

static void ir_capture_print_bytes(const char *label, const uint8_t *bytes, size_t len)
{
    printf("DAIKIN_CAPTURE_%s", label);
    for (size_t i = 0; i < len; ++i) {
        printf(",%02X", bytes[i]);
    }
    printf("\n");
}

static void ir_capture_log_daikin_decode(const rmt_symbol_word_t *symbols, size_t count)
{
    if (count != DAIKIN_CAPTURE_SYMBOL_COUNT) {
        return;
    }

    uint8_t section_1[DAIKIN_SECTION_1_LEN];
    uint8_t section_2[DAIKIN_SECTION_2_LEN];
    uint8_t section_3[DAIKIN_SECTION_3_LEN];
    size_t position = 6;

    ++position;
    ir_capture_decode_bytes_lsb(symbols, &position, section_1, sizeof(section_1));
    ++position;
    ++position;
    ir_capture_decode_bytes_lsb(symbols, &position, section_2, sizeof(section_2));
    ++position;
    ++position;
    ir_capture_decode_bytes_lsb(symbols, &position, section_3, sizeof(section_3));

    ir_capture_print_bytes("SECTION_1", section_1, sizeof(section_1));
    ir_capture_print_bytes("SECTION_2", section_2, sizeof(section_2));
    ir_capture_print_bytes("SECTION_3", section_3, sizeof(section_3));
    printf("DAIKIN_CAPTURE_FIELDS,mode_power=%02X,temp=%02X,fan_vswing=%02X,hswing=%02X,quiet=%02X,sensor=%02X,checksum=%s\n",
           section_3[5],
           section_3[6],
           section_3[8],
           section_3[9],
           section_3[13],
           section_3[16],
           ir_capture_checksum(section_3, sizeof(section_3) - 1) == section_3[18] ? "ok" : "bad");
}

static void ir_capture_log_symbols(const rmt_symbol_word_t *symbols, size_t count)
{
    ESP_LOGI(TAG, "Captured %u RMT symbols", (unsigned)count);
    printf("IR_CAPTURE_BEGIN,%u,%u\n", (unsigned)count, (unsigned)s_symbols_capacity);
    ir_capture_log_daikin_decode(symbols, count);

    for (size_t i = 0; i < count; ++i) {
        ESP_LOGD(TAG, "%03u: %u,%u,%u,%u", (unsigned)i,
                 symbols[i].level0, symbols[i].duration0,
                 symbols[i].level1, symbols[i].duration1);
        printf("IR_CAPTURE_SYMBOL,%u,%u,%u,%u,%u\n", (unsigned)i,
               symbols[i].level0, symbols[i].duration0,
               symbols[i].level1, symbols[i].duration1);
    }

    printf("IR_CAPTURE_END\n");
    fflush(stdout);

    if (count >= s_symbols_capacity) {
        ESP_LOGW(TAG, "Capture filled the %u-symbol buffer; frame may be truncated",
                 (unsigned)s_symbols_capacity);
    }
}

esp_err_t ir_capture_init(const ir_capture_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is required");
    ESP_RETURN_ON_FALSE(config->symbols_capacity <= IR_CAPTURE_MAX_SYMBOLS,
                        ESP_ERR_INVALID_ARG, TAG,
                        "symbols_capacity exceeds static buffer");
    ESP_RETURN_ON_FALSE(config->mem_block_symbols <= config->symbols_capacity,
                        ESP_ERR_INVALID_ARG, TAG,
                        "mem_block_symbols cannot exceed symbols_capacity");

    rmt_rx_channel_config_t rx_config = {
        .gpio_num = config->rx_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config->resolution_hz,
        .mem_block_symbols = config->mem_block_symbols,
        .flags.invert_in = false,
        .flags.with_dma = false,
    };

    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_config, &s_rx_channel), TAG,
                        "failed to create RMT RX channel");

    s_receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    ESP_RETURN_ON_FALSE(s_receive_queue != NULL, ESP_ERR_NO_MEM, TAG,
                        "failed to create receive queue");

    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = ir_capture_on_receive_done,
    };
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(s_rx_channel, &callbacks,
                                                        s_receive_queue),
                        TAG, "failed to register RX callbacks");

    ESP_RETURN_ON_ERROR(rmt_enable(s_rx_channel), TAG, "failed to enable RMT RX channel");

    s_receive_config = (rmt_receive_config_t) {
        .signal_range_min_ns = config->signal_range_min_ns,
        .signal_range_max_ns = config->signal_range_max_ns,
    };
    s_symbols_capacity = config->symbols_capacity;

    ESP_LOGI(TAG, "RMT RX ready on GPIO %d at %" PRIu32
             " Hz, hardware block %u symbols, capture buffer %u symbols",
             config->rx_gpio, config->resolution_hz,
             (unsigned)config->mem_block_symbols,
             (unsigned)config->symbols_capacity);
    printf("IR_CAPTURE_CONFIG,1,%u,%u\n",
           (unsigned)config->symbols_capacity,
           (unsigned)config->mem_block_symbols);
    fflush(stdout);
    return ESP_OK;
}

esp_err_t ir_capture_start_once(void)
{
    ESP_RETURN_ON_FALSE(s_rx_channel != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "RMT RX channel is not initialized");

    memset(s_symbols, 0, sizeof(s_symbols));
    return rmt_receive(s_rx_channel, s_symbols,
                       s_symbols_capacity * sizeof(s_symbols[0]),
                       &s_receive_config);
}

esp_err_t ir_capture_run_forever(void)
{
    ESP_RETURN_ON_FALSE(s_rx_channel != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "RMT RX channel is not initialized");
    ESP_RETURN_ON_FALSE(s_receive_queue != NULL, ESP_ERR_INVALID_STATE, TAG,
                        "receive queue is not initialized");

    ESP_RETURN_ON_ERROR(ir_capture_start_once(), TAG, "failed to start first receive");

    while (true) {
        rmt_rx_done_event_data_t rx_data;
        if (xQueueReceive(s_receive_queue, &rx_data, portMAX_DELAY) == pdTRUE) {
            ir_capture_log_symbols(rx_data.received_symbols, rx_data.num_symbols);
            ESP_RETURN_ON_ERROR(ir_capture_start_once(), TAG, "failed to restart receive");
        }
    }

    return ESP_OK;
}
