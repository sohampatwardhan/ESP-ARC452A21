#include "daikin_frame.h"

#include <string.h>

#include "esp_check.h"

static const char *TAG = "daikin_frame";

static const uint8_t SECTION_1[DAIKIN_FRAME_SECTION_1_LEN] = {
    0x11, 0xDA, 0x27, 0x00, 0xC5, 0x00, 0x00, 0xD7,
};

static const uint8_t SECTION_2[DAIKIN_FRAME_SECTION_2_LEN] = {
    0x11, 0xDA, 0x27, 0x00, 0x42, 0x00, 0x10, 0x64,
};

uint8_t daikin_frame_checksum(const uint8_t *bytes, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = (uint8_t)(sum + bytes[i]);
    }
    return sum;
}

static uint8_t mode_byte(const daikin_state_t *state)
{
    uint8_t mode = 0x38;
    switch (state->mode) {
    case DAIKIN_MODE_AUTO:
        mode = 0x08;
        break;
    case DAIKIN_MODE_DRY:
        mode = 0x28;
        break;
    case DAIKIN_MODE_COOL:
        mode = 0x38;
        break;
    case DAIKIN_MODE_HEAT:
        mode = 0x48;
        break;
    case DAIKIN_MODE_FAN:
        mode = 0x68;
        break;
    }

    if (state->power) {
        mode |= 0x01;
    }

    return mode;
}

static uint8_t temperature_byte(const daikin_state_t *state)
{
    if (state->mode == DAIKIN_MODE_DRY) {
        return 0xC0;
    }

    if (state->mode == DAIKIN_MODE_FAN) {
        return 0x32;
    }

    if (state->use_fahrenheit) {
        return (uint8_t)(state->target_fahrenheit - 28);
    }

    return (uint8_t)(((state->target_celsius * 9 / 5) + 32) - 28);
}

static uint8_t fan_byte(const daikin_state_t *state)
{
    uint8_t fan = 0x50;
    if (state->mode == DAIKIN_MODE_DRY) {
        fan = 0xA0;
    } else {
        switch (state->fan) {
        case DAIKIN_FAN_SPEED_1:
            fan = 0x30;
            break;
        case DAIKIN_FAN_SPEED_2:
            fan = 0x40;
            break;
        case DAIKIN_FAN_SPEED_3:
            fan = 0x50;
            break;
        case DAIKIN_FAN_SPEED_4:
            fan = 0x60;
            break;
        case DAIKIN_FAN_SPEED_5:
            fan = 0x70;
            break;
        case DAIKIN_FAN_AUTO:
            fan = 0xA0;
            break;
        case DAIKIN_FAN_NIGHT:
            fan = 0xB0;
            break;
        }
    }

    if (state->sensor == DAIKIN_SENSOR_COMFORT ||
        state->sensor == DAIKIN_SENSOR_COMFORT_AND_INTELLIGENT_EYE) {
        fan = 0xA0;
    }

    if (state->swing_vertical) {
        fan |= 0x0F;
    }

    return fan;
}

static uint8_t sensor_byte(const daikin_state_t *state)
{
    if (state->sensor == DAIKIN_SENSOR_INTELLIGENT_EYE ||
        state->sensor == DAIKIN_SENSOR_COMFORT_AND_INTELLIGENT_EYE) {
        return 0x82;
    }

    return 0x80;
}

void daikin_state_init_cool_fahrenheit(daikin_state_t *state, uint8_t target_fahrenheit)
{
    memset(state, 0, sizeof(*state));
    state->power = true;
    state->mode = DAIKIN_MODE_COOL;
    state->use_fahrenheit = true;
    state->target_fahrenheit = target_fahrenheit;
    state->target_celsius = (uint8_t)((target_fahrenheit - 32) * 5 / 9);
    state->fan = DAIKIN_FAN_SPEED_3;
    state->sensor = DAIKIN_SENSOR_OFF;
}

esp_err_t daikin_frame_build(const daikin_state_t *state, daikin_frame_t *frame)
{
    ESP_RETURN_ON_FALSE(state != NULL, ESP_ERR_INVALID_ARG, TAG, "state is required");
    ESP_RETURN_ON_FALSE(frame != NULL, ESP_ERR_INVALID_ARG, TAG, "frame is required");
    if (state->use_fahrenheit) {
        ESP_RETURN_ON_FALSE(state->target_fahrenheit >= DAIKIN_MIN_TEMP_F &&
                                state->target_fahrenheit <= DAIKIN_MAX_TEMP_F,
                            ESP_ERR_INVALID_ARG, TAG,
                            "target_fahrenheit must be in the 64..90 F range");
    } else {
        ESP_RETURN_ON_FALSE(state->target_celsius >= DAIKIN_MIN_TEMP_C &&
                                state->target_celsius <= DAIKIN_MAX_TEMP_C,
                            ESP_ERR_INVALID_ARG, TAG,
                            "target_celsius must be in the 18..32 C range");
    }

    memcpy(frame->section_1, SECTION_1, sizeof(frame->section_1));
    memcpy(frame->section_2, SECTION_2, sizeof(frame->section_2));
    memset(frame->section_3, 0, sizeof(frame->section_3));

    if (state->sensor == DAIKIN_SENSOR_COMFORT ||
        state->sensor == DAIKIN_SENSOR_COMFORT_AND_INTELLIGENT_EYE) {
        frame->section_1[6] = 0x10;
        frame->section_1[7] = daikin_frame_checksum(frame->section_1,
                                                    DAIKIN_FRAME_SECTION_1_LEN - 1);
    }

    frame->section_3[0] = 0x11;
    frame->section_3[1] = 0xDA;
    frame->section_3[2] = 0x27;
    frame->section_3[5] = mode_byte(state);
    frame->section_3[6] = temperature_byte(state);
    frame->section_3[8] = fan_byte(state);
    frame->section_3[9] = state->swing_horizontal ? 0x0F : 0x00;
    frame->section_3[11] = 0x06;
    frame->section_3[12] = 0x60;
    frame->section_3[13] = state->quiet ? 0x20 : 0x00;
    frame->section_3[15] = 0xC1;
    frame->section_3[16] = sensor_byte(state);
    frame->section_3[18] = daikin_frame_checksum(frame->section_3,
                                                 DAIKIN_FRAME_SECTION_3_LEN - 1);
    return ESP_OK;
}
