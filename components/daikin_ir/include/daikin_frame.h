#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "daikin_ir.h"

enum {
    DAIKIN_FRAME_SECTION_1_LEN = 8,
    DAIKIN_FRAME_SECTION_2_LEN = 8,
    DAIKIN_FRAME_SECTION_3_LEN = 19,
};

typedef struct {
    uint8_t section_1[DAIKIN_FRAME_SECTION_1_LEN];
    uint8_t section_2[DAIKIN_FRAME_SECTION_2_LEN];
    uint8_t section_3[DAIKIN_FRAME_SECTION_3_LEN];
} daikin_frame_t;

uint8_t daikin_frame_checksum(const uint8_t *bytes, size_t len);
esp_err_t daikin_frame_build(const daikin_state_t *state, daikin_frame_t *frame);