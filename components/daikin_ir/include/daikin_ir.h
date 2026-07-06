#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#define DAIKIN_MIN_TEMP_C 18
#define DAIKIN_MAX_TEMP_C 32
#define DAIKIN_MIN_TEMP_F 64
#define DAIKIN_MAX_TEMP_F 90

typedef enum {
    DAIKIN_MODE_AUTO,
    DAIKIN_MODE_DRY,
    DAIKIN_MODE_COOL,
    DAIKIN_MODE_HEAT,
    DAIKIN_MODE_FAN,
} daikin_mode_t;

typedef enum {
    DAIKIN_FAN_SPEED_1,
    DAIKIN_FAN_SPEED_2,
    DAIKIN_FAN_SPEED_3,
    DAIKIN_FAN_SPEED_4,
    DAIKIN_FAN_SPEED_5,
    DAIKIN_FAN_AUTO,
    DAIKIN_FAN_NIGHT,
} daikin_fan_t;

typedef enum {
    DAIKIN_SENSOR_OFF,
    DAIKIN_SENSOR_COMFORT,
    DAIKIN_SENSOR_INTELLIGENT_EYE,
    DAIKIN_SENSOR_COMFORT_AND_INTELLIGENT_EYE,
} daikin_sensor_t;

typedef enum {
    DAIKIN_TIMING_PROFILE_CAPTURED,
    DAIKIN_TIMING_PROFILE_NOMINAL,
} daikin_timing_profile_t;

typedef struct {
    bool power;
    daikin_mode_t mode;
    bool use_fahrenheit;
    uint8_t target_fahrenheit;
    uint8_t target_celsius;
    daikin_fan_t fan;
    bool swing_vertical;
    bool swing_horizontal;
    bool quiet;
    daikin_sensor_t sensor;
} daikin_state_t;

typedef struct {
    gpio_num_t tx_gpio;
    uint32_t resolution_hz;
    uint32_t carrier_hz;
    float carrier_duty_cycle;
    bool invert_out;
    bool carrier_active_low;
    uint8_t repeat_count;
    uint32_t repeat_gap_ms;
} daikin_ir_config_t;

#define DAIKIN_IR_DEFAULT_CONFIG(gpio)       \
    {                                        \
        .tx_gpio = (gpio),                   \
        .resolution_hz = 1000000,            \
        .carrier_hz = 38000,                 \
        .carrier_duty_cycle = 0.33f,         \
        .invert_out = false,                 \
        .carrier_active_low = false,         \
        .repeat_count = 1,                   \
        .repeat_gap_ms = 80,                 \
    }

esp_err_t daikin_ir_init(const daikin_ir_config_t *config);
void daikin_state_init_cool_fahrenheit(daikin_state_t *state, uint8_t target_fahrenheit);
esp_err_t daikin_ir_set_invert_out(bool invert_out);
void daikin_ir_set_repeat(uint8_t repeat_count, uint32_t repeat_gap_ms);
esp_err_t daikin_ir_set_timing_profile(daikin_timing_profile_t profile);
esp_err_t daikin_ir_send_state(const daikin_state_t *state);
