#pragma once

#include <stddef.h>

#include "esp_err.h"

#include "daikin_ir.h"

esp_err_t ac_control_get_state(daikin_state_t *state);
esp_err_t ac_control_apply_state(const daikin_state_t *state,
                                 char *response,
                                 size_t response_size);
void ac_control_state_set_temperature_celsius(daikin_state_t *state, uint8_t temp_c);
void ac_control_state_set_temperature_celsius_preserve_unit(daikin_state_t *state, uint8_t temp_c);
void ac_control_state_set_temperature_fahrenheit(daikin_state_t *state, uint8_t temp_f);
