#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t rx_gpio;
    uint32_t resolution_hz;
    size_t mem_block_symbols;
    size_t symbols_capacity;
    uint32_t signal_range_min_ns;
    uint32_t signal_range_max_ns;
} ir_capture_config_t;

#define IR_CAPTURE_MAX_SYMBOLS 2048
#define IR_CAPTURE_MEM_BLOCK_SYMBOLS 512

#define IR_CAPTURE_DEFAULT_CONFIG(gpio)                     \
    {                                                       \
        .rx_gpio = (gpio),                                  \
        .resolution_hz = 1000000,                           \
        .mem_block_symbols = IR_CAPTURE_MEM_BLOCK_SYMBOLS,  \
        .symbols_capacity = IR_CAPTURE_MAX_SYMBOLS,         \
        .signal_range_min_ns = 1250,                        \
        .signal_range_max_ns = 50000000,                    \
    }

esp_err_t ir_capture_init(const ir_capture_config_t *config);
esp_err_t ir_capture_start_once(void);
esp_err_t ir_capture_run_forever(void);
