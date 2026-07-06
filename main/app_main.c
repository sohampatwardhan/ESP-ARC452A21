#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "daikin_frame.h"
#include "daikin_ir.h"
#include "ac_control.h"
#include "homekit_accessory.h"
#include "iot_remote.h"
#include "ir_capture.h"
#include "led_strip.h"
#include "matter_accessory.h"

static const char *TAG = "esp_arc452a21";
static const char *APP_SETTINGS_NAMESPACE = "app_cfg";

#define IR_RX_GPIO GPIO_NUM_16
#define IR_TX_GPIO GPIO_NUM_17
#define RGB_LED_GPIO GPIO_NUM_5
#define IR_RX_LIVE_MEM_BLOCK_SYMBOLS 384
#define RESPONSE_MAX_LEN 640

static led_strip_handle_t s_rgb_led;
static SemaphoreHandle_t s_command_mutex;
static daikin_state_t s_state;
static int64_t s_state_updated_us;

typedef struct {
    bool use_fahrenheit;
    bool ir_invert_out;
    daikin_timing_profile_t ir_timing_profile;
    uint8_t ir_repeat_count;
    uint32_t ir_repeat_gap_ms;
    bool homekit_enabled;
} app_settings_t;

static app_settings_t s_settings = {
    .use_fahrenheit = true,
    .ir_invert_out = false,
    .ir_timing_profile = DAIKIN_TIMING_PROFILE_NOMINAL,
    .ir_repeat_count = 1,
    .ir_repeat_gap_ms = 80,
    .homekit_enabled = CONFIG_ESP_ARC452A21_HOMEKIT_ENABLE,
};

typedef enum {
    COMMAND_INVALID,
    COMMAND_NO_SEND,
    COMMAND_SEND,
    COMMAND_REBOOT,
} command_result_t;

static void ir_capture_task(void *arg)
{
    (void)arg;

    esp_err_t err = ir_capture_run_forever();
    ESP_LOGE(TAG, "IR capture task stopped: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
}

static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

static esp_err_t init_console_uart(void)
{
    esp_err_t err = uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return err;
}

static esp_err_t arm_loopback_expectation(const daikin_state_t *state)
{
    daikin_frame_t frame;
    ESP_RETURN_ON_ERROR(daikin_frame_build(state, &frame), TAG,
                        "failed to build Daikin loopback expectation");

    ir_capture_daikin_frame_t expected = {};
    memcpy(expected.section_1, frame.section_1, sizeof(expected.section_1));
    memcpy(expected.section_2, frame.section_2, sizeof(expected.section_2));
    memcpy(expected.section_3, frame.section_3, sizeof(expected.section_3));
    ir_capture_expect_daikin_frame(&expected);
    return ESP_OK;
}

static esp_err_t rgb_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };

    ESP_RETURN_ON_ERROR(led_strip_new_spi_device(&strip_config, &spi_config, &s_rgb_led),
                        TAG, "failed to create RGB LED strip");
    ESP_RETURN_ON_ERROR(led_strip_clear(s_rgb_led), TAG, "failed to clear RGB LED");
    ESP_LOGI(TAG, "IR activity RGB LED ready on GPIO %d", RGB_LED_GPIO);
    return ESP_OK;
}

static void rgb_led_set_ir_active(bool active)
{
    if (s_rgb_led == NULL) {
        return;
    }

    esp_err_t err = ESP_OK;
    if (active) {
        err = led_strip_set_pixel(s_rgb_led, 0, 0, 24, 24);
        if (err == ESP_OK) {
            err = led_strip_refresh(s_rgb_led);
        }
    } else {
        err = led_strip_clear(s_rgb_led);
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RGB LED update failed: %s", esp_err_to_name(err));
    }
}

static void read_serial_line(char *line, size_t line_size)
{
    size_t len = 0;

    while (true) {
        uint8_t ch = 0;
        int read = uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
        if (read <= 0) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            if (len == 0) {
                continue;
            }
            line[len] = '\0';
            printf("\n");
            return;
        }

        if (ch == '\b' || ch == 0x7F) {
            if (len > 0) {
                --len;
                printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if (len + 1 < line_size && isprint((unsigned char)ch)) {
            line[len++] = (char)ch;
            printf("%c", ch);
            fflush(stdout);
        }
    }
}

static const char *mode_name(daikin_mode_t mode)
{
    switch (mode) {
    case DAIKIN_MODE_AUTO:
        return "auto";
    case DAIKIN_MODE_DRY:
        return "dry";
    case DAIKIN_MODE_COOL:
        return "cool";
    case DAIKIN_MODE_HEAT:
        return "heat";
    case DAIKIN_MODE_FAN:
        return "fan";
    }
    return "unknown";
}

static const char *fan_name(daikin_fan_t fan)
{
    switch (fan) {
    case DAIKIN_FAN_SPEED_1:
        return "1";
    case DAIKIN_FAN_SPEED_2:
        return "2";
    case DAIKIN_FAN_SPEED_3:
        return "3";
    case DAIKIN_FAN_SPEED_4:
        return "4";
    case DAIKIN_FAN_SPEED_5:
        return "5";
    case DAIKIN_FAN_AUTO:
        return "auto";
    case DAIKIN_FAN_NIGHT:
        return "night";
    }
    return "unknown";
}

static const char *sensor_name(daikin_sensor_t sensor)
{
    switch (sensor) {
    case DAIKIN_SENSOR_OFF:
        return "off";
    case DAIKIN_SENSOR_COMFORT:
        return "comfort";
    case DAIKIN_SENSOR_INTELLIGENT_EYE:
        return "eye";
    case DAIKIN_SENSOR_COMFORT_AND_INTELLIGENT_EYE:
        return "both";
    }
    return "unknown";
}

static const char *timing_name(daikin_timing_profile_t profile)
{
    switch (profile) {
    case DAIKIN_TIMING_PROFILE_CAPTURED:
        return "captured";
    case DAIKIN_TIMING_PROFILE_NOMINAL:
        return "nominal";
    }
    return "unknown";
}

static const char *unit_name(const daikin_state_t *state)
{
    return state->use_fahrenheit ? "F" : "C";
}

static void set_temperature_fahrenheit(daikin_state_t *state, uint8_t temp_f);
static void set_temperature_celsius(daikin_state_t *state, uint8_t temp_c);

static esp_err_t init_nvs_storage(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "failed to erase NVS");
        err = nvs_flash_init();
    }
    return err;
}

static void load_app_settings(void)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(APP_SETTINGS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Using default app settings");
        return;
    }

    uint8_t unit = s_settings.use_fahrenheit ? 1 : 0;
    if (nvs_get_u8(nvs, "unit_f", &unit) == ESP_OK) {
        s_settings.use_fahrenheit = unit != 0;
    }

    uint8_t invert = s_settings.ir_invert_out ? 1 : 0;
    if (nvs_get_u8(nvs, "ir_inv", &invert) == ESP_OK) {
        s_settings.ir_invert_out = invert != 0;
    }

    uint8_t timing = (uint8_t)s_settings.ir_timing_profile;
    if (nvs_get_u8(nvs, "ir_timing", &timing) == ESP_OK &&
        timing <= (uint8_t)DAIKIN_TIMING_PROFILE_NOMINAL) {
        s_settings.ir_timing_profile = (daikin_timing_profile_t)timing;
    }

    uint8_t repeat = s_settings.ir_repeat_count;
    if (nvs_get_u8(nvs, "ir_repeat", &repeat) == ESP_OK && repeat >= 1 && repeat <= 10) {
        s_settings.ir_repeat_count = repeat;
    }

    uint32_t gap = s_settings.ir_repeat_gap_ms;
    if (nvs_get_u32(nvs, "ir_gap", &gap) == ESP_OK && gap <= 1000) {
        s_settings.ir_repeat_gap_ms = gap;
    }

    uint8_t homekit_enabled = s_settings.homekit_enabled ? 1 : 0;
    if (nvs_get_u8(nvs, "hk_en", &homekit_enabled) == ESP_OK) {
        s_settings.homekit_enabled = homekit_enabled != 0;
    }

    nvs_close(nvs);
    ESP_LOGI(TAG,
             "Loaded app settings: unit=%s polarity=%s timing=%s repeat=%u gap=%" PRIu32
             " homekit=%s",
             s_settings.use_fahrenheit ? "F" : "C",
             s_settings.ir_invert_out ? "invert" : "normal",
             timing_name(s_settings.ir_timing_profile),
             (unsigned)s_settings.ir_repeat_count,
             s_settings.ir_repeat_gap_ms,
             s_settings.homekit_enabled ? "enabled" : "disabled");
}

static void save_app_u8(const char *key, uint8_t value)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(APP_SETTINGS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, key, value);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save setting %s: %s", key, esp_err_to_name(err));
    }
}

static void save_app_u32(const char *key, uint32_t value)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(APP_SETTINGS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u32(nvs, key, value);
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save setting %s: %s", key, esp_err_to_name(err));
    }
}

static void load_ac_state(void)
{
    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(APP_SETTINGS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return;
    }

    uint8_t value = 0;
    if (nvs_get_u8(nvs, "ac_saved", &value) != ESP_OK || value == 0) {
        nvs_close(nvs);
        ESP_LOGI(TAG, "Using default AC state");
        return;
    }

    if (nvs_get_u8(nvs, "ac_pwr", &value) == ESP_OK) {
        s_state.power = value != 0;
    }
    if (nvs_get_u8(nvs, "ac_mode", &value) == ESP_OK && value <= (uint8_t)DAIKIN_MODE_FAN) {
        s_state.mode = (daikin_mode_t)value;
    }
    if (nvs_get_u8(nvs, "unit_f", &value) == ESP_OK) {
        s_state.use_fahrenheit = value != 0;
        s_settings.use_fahrenheit = s_state.use_fahrenheit;
    }
    if (nvs_get_u8(nvs, "ac_tf", &value) == ESP_OK &&
        value >= DAIKIN_MIN_TEMP_F && value <= DAIKIN_MAX_TEMP_F) {
        s_state.target_fahrenheit = value;
    }
    if (nvs_get_u8(nvs, "ac_tc", &value) == ESP_OK &&
        value >= DAIKIN_MIN_TEMP_C && value <= DAIKIN_MAX_TEMP_C) {
        s_state.target_celsius = value;
    }
    if (nvs_get_u8(nvs, "ac_fan", &value) == ESP_OK && value <= (uint8_t)DAIKIN_FAN_NIGHT) {
        s_state.fan = (daikin_fan_t)value;
    }
    if (nvs_get_u8(nvs, "ac_vs", &value) == ESP_OK) {
        s_state.swing_vertical = value != 0;
    }
    if (nvs_get_u8(nvs, "ac_hs", &value) == ESP_OK) {
        s_state.swing_horizontal = value != 0;
    }
    if (nvs_get_u8(nvs, "ac_quiet", &value) == ESP_OK) {
        s_state.quiet = value != 0;
    }
    if (nvs_get_u8(nvs, "ac_sensor", &value) == ESP_OK &&
        value <= (uint8_t)DAIKIN_SENSOR_COMFORT_AND_INTELLIGENT_EYE) {
        s_state.sensor = (daikin_sensor_t)value;
    }

    nvs_close(nvs);
    ESP_LOGI(TAG, "Loaded saved AC state");
}

static void save_ac_state_locked(void)
{
    s_settings.use_fahrenheit = s_state.use_fahrenheit;

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(APP_SETTINGS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_saved", 1);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_pwr", s_state.power ? 1 : 0);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_mode", (uint8_t)s_state.mode);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "unit_f", s_state.use_fahrenheit ? 1 : 0);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_tf", s_state.target_fahrenheit);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_tc", s_state.target_celsius);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_fan", (uint8_t)s_state.fan);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_vs", s_state.swing_vertical ? 1 : 0);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_hs", s_state.swing_horizontal ? 1 : 0);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_quiet", s_state.quiet ? 1 : 0);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ac_sensor", (uint8_t)s_state.sensor);
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    if (nvs != 0) {
        nvs_close(nvs);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to save AC state: %s", esp_err_to_name(err));
    }
}

static void apply_loaded_display_settings(void)
{
    if (s_settings.use_fahrenheit) {
        set_temperature_fahrenheit(&s_state, s_state.target_fahrenheit);
    } else {
        set_temperature_celsius(&s_state, s_state.target_celsius);
    }
}

static void print_state(const daikin_state_t *state)
{
    printf("State: power=%s mode=%s temp=%u %s fan=%s vswing=%s hswing=%s quiet=%s sensor=%s\n",
           state->power ? "on" : "off",
           mode_name(state->mode),
           state->use_fahrenheit ? state->target_fahrenheit : state->target_celsius,
           unit_name(state),
           fan_name(state->fan),
           state->swing_vertical ? "on" : "off",
           state->swing_horizontal ? "on" : "off",
           state->quiet ? "on" : "off",
           sensor_name(state->sensor));
}

static bool ac_state_equal(const daikin_state_t *a, const daikin_state_t *b)
{
    return a->power == b->power &&
           a->mode == b->mode &&
           a->use_fahrenheit == b->use_fahrenheit &&
           a->target_fahrenheit == b->target_fahrenheit &&
           a->target_celsius == b->target_celsius &&
           a->fan == b->fan &&
           a->swing_vertical == b->swing_vertical &&
           a->swing_horizontal == b->swing_horizontal &&
           a->quiet == b->quiet &&
           a->sensor == b->sensor;
}

static void mark_ac_state_updated_locked(void)
{
    s_state_updated_us = esp_timer_get_time();
}

static int64_t ac_state_updated_age_s(void)
{
    int64_t updated_us = s_state_updated_us;
    if (updated_us <= 0) {
        return 0;
    }
    int64_t age_us = esp_timer_get_time() - updated_us;
    return age_us > 0 ? age_us / 1000000 : 0;
}

static void format_state_json(const daikin_state_t *state,
                              bool ok,
                              const char *message,
                              char *response,
                              size_t response_size)
{
    snprintf(response, response_size,
             "{\"ok\":%s,\"message\":\"%s\",\"state\":{\"power\":\"%s\",\"mode\":\"%s\","
             "\"temperature\":%u,\"unit\":\"%s\",\"fan\":\"%s\",\"vswing\":%s,"
             "\"hswing\":%s,\"quiet\":%s,\"sensor\":\"%s\"},\"updated_age_s\":%" PRId64
             ",\"ir\":{\"polarity\":\"%s\","
             "\"timing\":\"%s\",\"repeat\":%u,\"gap_ms\":%" PRIu32 "}}",
             ok ? "true" : "false",
             message == NULL ? "" : message,
             state->power ? "on" : "off",
             mode_name(state->mode),
             state->use_fahrenheit ? state->target_fahrenheit : state->target_celsius,
             unit_name(state),
             fan_name(state->fan),
             state->swing_vertical ? "true" : "false",
             state->swing_horizontal ? "true" : "false",
             state->quiet ? "true" : "false",
             sensor_name(state->sensor),
             ac_state_updated_age_s(),
             s_settings.ir_invert_out ? "invert" : "normal",
             timing_name(s_settings.ir_timing_profile),
             (unsigned)s_settings.ir_repeat_count,
             s_settings.ir_repeat_gap_ms);
}

static void print_help(void)
{
    printf("Commands send the full current state after updating it:\n");
    printf("  72 | 22c | temp 72 | temp 22 c | on 72 | off 72\n");
    printf("  on | off\n");
    printf("  unit fahrenheit|celsius\n");
    printf("  mode auto|dry|cool|heat|fan\n");
    printf("  fan 1|2|3|4|5|auto|night\n");
    printf("  vswing on|off | hswing on|off | quiet on|off\n");
    printf("  sensor off|comfort|eye|both\n");
    printf("  polarity normal|invert\n");
    printf("  repeat 1..10 [gap_ms]\n");
    printf("  timing captured|nominal\n");
    printf("  loopback clear\n");
    printf("  reboot\n");
    printf("  set <command>     # update state without sending\n");
    printf("  send | status | help\n");
}

static bool parse_on_off(const char *value, bool *out)
{
    if (strcmp(value, "on") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(value, "off") == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool parse_unit(const char *value, bool *use_fahrenheit)
{
    if (strcmp(value, "f") == 0 || strcmp(value, "fahrenheit") == 0) {
        *use_fahrenheit = true;
        return true;
    }
    if (strcmp(value, "c") == 0 || strcmp(value, "celsius") == 0) {
        *use_fahrenheit = false;
        return true;
    }
    return false;
}

static void set_temperature_fahrenheit(daikin_state_t *state, uint8_t temp_f)
{
    state->use_fahrenheit = true;
    state->target_fahrenheit = temp_f;
    state->target_celsius = (uint8_t)((temp_f - 32) * 5 / 9);
}

static void set_temperature_celsius(daikin_state_t *state, uint8_t temp_c)
{
    state->use_fahrenheit = false;
    state->target_celsius = temp_c;
    state->target_fahrenheit = (uint8_t)((temp_c * 9 / 5) + 32);
}

static void set_temperature_celsius_preserve_unit(daikin_state_t *state, uint8_t temp_c)
{
    bool use_fahrenheit = state->use_fahrenheit;
    set_temperature_celsius(state, temp_c);
    state->use_fahrenheit = use_fahrenheit;
}

void ac_control_state_set_temperature_fahrenheit(daikin_state_t *state, uint8_t temp_f)
{
    set_temperature_fahrenheit(state, temp_f);
}

void ac_control_state_set_temperature_celsius(daikin_state_t *state, uint8_t temp_c)
{
    set_temperature_celsius(state, temp_c);
}

void ac_control_state_set_temperature_celsius_preserve_unit(daikin_state_t *state, uint8_t temp_c)
{
    set_temperature_celsius_preserve_unit(state, temp_c);
}

static bool set_temperature(daikin_state_t *state, const char *value, const char *unit)
{
    char *end = NULL;
    long temp = strtol(value, &end, 10);
    if (value == end) {
        return false;
    }

    bool use_fahrenheit = state->use_fahrenheit;
    if (*end != '\0') {
        if (end[1] != '\0' || unit != NULL) {
            return false;
        }
        char suffix[2] = {(char)tolower((unsigned char)*end), '\0'};
        if (!parse_unit(suffix, &use_fahrenheit)) {
            return false;
        }
    } else if (unit != NULL && !parse_unit(unit, &use_fahrenheit)) {
        return false;
    }

    if (use_fahrenheit) {
        if (temp < DAIKIN_MIN_TEMP_F || temp > DAIKIN_MAX_TEMP_F) {
            printf("Temperature must be %d..%d F\n", DAIKIN_MIN_TEMP_F, DAIKIN_MAX_TEMP_F);
            return false;
        }
        set_temperature_fahrenheit(state, (uint8_t)temp);
    } else {
        if (temp < DAIKIN_MIN_TEMP_C || temp > DAIKIN_MAX_TEMP_C) {
            printf("Temperature must be %d..%d C\n", DAIKIN_MIN_TEMP_C, DAIKIN_MAX_TEMP_C);
            return false;
        }
        set_temperature_celsius(state, (uint8_t)temp);
    }

    return true;
}

static bool parse_long_range(const char *value, long min, long max, long *out)
{
    char *end = NULL;
    long parsed = strtol(value, &end, 10);
    if (value == end || *end != '\0' || parsed < min || parsed > max) {
        return false;
    }
    *out = parsed;
    return true;
}

static command_result_t parse_command(char *line, daikin_state_t *state)
{
    for (char *ch = line; *ch != '\0'; ++ch) {
        *ch = (char)tolower((unsigned char)*ch);
    }

    char *saveptr = NULL;
    char *command = strtok_r(line, " \t", &saveptr);
    if (command == NULL) {
        return COMMAND_NO_SEND;
    }

    if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
        print_help();
        return COMMAND_NO_SEND;
    }
    if (strcmp(command, "status") == 0) {
        print_state(state);
        return COMMAND_NO_SEND;
    }
    if (strcmp(command, "send") == 0) {
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }
    if (strcmp(command, "reboot") == 0 || strcmp(command, "restart") == 0) {
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_REBOOT : COMMAND_INVALID;
    }

    if (strcmp(command, "unit") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        bool use_fahrenheit = true;
        if (value == NULL || !parse_unit(value, &use_fahrenheit) ||
            strtok_r(NULL, " \t", &saveptr) != NULL) {
            return COMMAND_INVALID;
        }

        if (use_fahrenheit) {
            set_temperature_fahrenheit(state, state->target_fahrenheit);
        } else {
            set_temperature_celsius(state, state->target_celsius);
        }
        s_settings.use_fahrenheit = use_fahrenheit;
        save_app_u8("unit_f", use_fahrenheit ? 1 : 0);
        printf("Temperature unit set: %s\n", unit_name(state));
        return COMMAND_NO_SEND;
    }

    if (strcmp(command, "polarity") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL || strtok_r(NULL, " \t", &saveptr) != NULL) {
            return COMMAND_INVALID;
        }

        bool invert_out = false;
        if (strcmp(value, "normal") == 0 || strcmp(value, "active-high") == 0) {
            invert_out = false;
        } else if (strcmp(value, "invert") == 0 || strcmp(value, "active-low") == 0) {
            invert_out = true;
        } else {
            return COMMAND_INVALID;
        }

        esp_err_t err = daikin_ir_set_invert_out(invert_out);
        if (err != ESP_OK) {
            printf("Failed to set TX polarity: %s\n", esp_err_to_name(err));
        } else {
            s_settings.ir_invert_out = invert_out;
            save_app_u8("ir_inv", invert_out ? 1 : 0);
            printf("TX polarity set: invert_out=%d\n", invert_out);
        }
        return COMMAND_NO_SEND;
    }

    if (strcmp(command, "repeat") == 0) {
        char *count_value = strtok_r(NULL, " \t", &saveptr);
        if (count_value == NULL) {
            return COMMAND_INVALID;
        }

        long repeat_count = 0;
        if (!parse_long_range(count_value, 1, 10, &repeat_count)) {
            printf("Repeat count must be 1..10\n");
            return COMMAND_INVALID;
        }

        long repeat_gap_ms = 80;
        char *gap_value = strtok_r(NULL, " \t", &saveptr);
        if (gap_value != NULL && !parse_long_range(gap_value, 0, 1000, &repeat_gap_ms)) {
            printf("Repeat gap must be 0..1000 ms\n");
            return COMMAND_INVALID;
        }
        if (strtok_r(NULL, " \t", &saveptr) != NULL) {
            return COMMAND_INVALID;
        }

        daikin_ir_set_repeat((uint8_t)repeat_count, (uint32_t)repeat_gap_ms);
        s_settings.ir_repeat_count = (uint8_t)repeat_count;
        s_settings.ir_repeat_gap_ms = (uint32_t)repeat_gap_ms;
        save_app_u8("ir_repeat", s_settings.ir_repeat_count);
        save_app_u32("ir_gap", s_settings.ir_repeat_gap_ms);
        printf("TX repeat set: repeat=%ld gap=%ld ms\n", repeat_count, repeat_gap_ms);
        return COMMAND_NO_SEND;
    }

    if (strcmp(command, "timing") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL || strtok_r(NULL, " \t", &saveptr) != NULL) {
            return COMMAND_INVALID;
        }

        daikin_timing_profile_t profile = DAIKIN_TIMING_PROFILE_CAPTURED;
        if (strcmp(value, "captured") == 0) {
            profile = DAIKIN_TIMING_PROFILE_CAPTURED;
        } else if (strcmp(value, "nominal") == 0) {
            profile = DAIKIN_TIMING_PROFILE_NOMINAL;
        } else {
            return COMMAND_INVALID;
        }

        esp_err_t err = daikin_ir_set_timing_profile(profile);
        if (err != ESP_OK) {
            printf("Failed to set timing profile: %s\n", esp_err_to_name(err));
        } else {
            s_settings.ir_timing_profile = profile;
            save_app_u8("ir_timing", (uint8_t)profile);
            printf("TX timing set: %s\n", value);
        }
        return COMMAND_NO_SEND;
    }

    if (strcmp(command, "loopback") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL || strtok_r(NULL, " \t", &saveptr) != NULL) {
            return COMMAND_INVALID;
        }
        if (strcmp(value, "clear") != 0) {
            return COMMAND_INVALID;
        }
        ir_capture_clear_expected_daikin_frame();
        return COMMAND_NO_SEND;
    }

    if (strcmp(command, "on") == 0 || strcmp(command, "off") == 0) {
        state->power = strcmp(command, "on") == 0;
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value != NULL) {
            char *unit = strtok_r(NULL, " \t", &saveptr);
            if (!set_temperature(state, value, unit)) {
                return COMMAND_INVALID;
            }
            return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
        }
        return COMMAND_SEND;
    }

    if (strcmp(command, "temp") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        char *unit = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL || !set_temperature(state, value, unit)) {
            return COMMAND_INVALID;
        }
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    if (set_temperature(state, command, NULL)) {
        state->power = true;
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    if (strcmp(command, "mode") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL) {
            return COMMAND_INVALID;
        }
        if (strcmp(value, "auto") == 0) {
            state->mode = DAIKIN_MODE_AUTO;
        } else if (strcmp(value, "dry") == 0) {
            state->mode = DAIKIN_MODE_DRY;
        } else if (strcmp(value, "cool") == 0) {
            state->mode = DAIKIN_MODE_COOL;
        } else if (strcmp(value, "heat") == 0) {
            state->mode = DAIKIN_MODE_HEAT;
        } else if (strcmp(value, "fan") == 0) {
            state->mode = DAIKIN_MODE_FAN;
        } else {
            return COMMAND_INVALID;
        }
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    if (strcmp(command, "fan") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL) {
            return COMMAND_INVALID;
        }
        if (strcmp(value, "1") == 0) {
            state->fan = DAIKIN_FAN_SPEED_1;
        } else if (strcmp(value, "2") == 0) {
            state->fan = DAIKIN_FAN_SPEED_2;
        } else if (strcmp(value, "3") == 0) {
            state->fan = DAIKIN_FAN_SPEED_3;
        } else if (strcmp(value, "4") == 0) {
            state->fan = DAIKIN_FAN_SPEED_4;
        } else if (strcmp(value, "5") == 0) {
            state->fan = DAIKIN_FAN_SPEED_5;
        } else if (strcmp(value, "auto") == 0) {
            state->fan = DAIKIN_FAN_AUTO;
        } else if (strcmp(value, "night") == 0) {
            state->fan = DAIKIN_FAN_NIGHT;
        } else {
            return COMMAND_INVALID;
        }
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    if (strcmp(command, "vswing") == 0 || strcmp(command, "hswing") == 0 ||
        strcmp(command, "quiet") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        bool enabled = false;
        if (value == NULL || !parse_on_off(value, &enabled)) {
            return COMMAND_INVALID;
        }
        if (strcmp(command, "vswing") == 0) {
            state->swing_vertical = enabled;
        } else if (strcmp(command, "hswing") == 0) {
            state->swing_horizontal = enabled;
        } else {
            state->quiet = enabled;
        }
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    if (strcmp(command, "sensor") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL) {
            return COMMAND_INVALID;
        }
        if (strcmp(value, "off") == 0) {
            state->sensor = DAIKIN_SENSOR_OFF;
        } else if (strcmp(value, "comfort") == 0) {
            state->sensor = DAIKIN_SENSOR_COMFORT;
        } else if (strcmp(value, "eye") == 0) {
            state->sensor = DAIKIN_SENSOR_INTELLIGENT_EYE;
        } else if (strcmp(value, "both") == 0) {
            state->sensor = DAIKIN_SENSOR_COMFORT_AND_INTELLIGENT_EYE;
        } else {
            return COMMAND_INVALID;
        }
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    return COMMAND_INVALID;
}

static esp_err_t execute_command_line(const char *input,
                                      bool console_output,
                                      char *response,
                                      size_t response_size)
{
    if (response != NULL && response_size > 0) {
        response[0] = '\0';
    }

    if (input == NULL || input[0] == '\0') {
        if (response != NULL) {
            snprintf(response, response_size, "{\"ok\":false,\"error\":\"empty command\"}");
        }
        return ESP_ERR_INVALID_ARG;
    }

    char line[96];
    strlcpy(line, input, sizeof(line));
    char *parse_line = line;
    while (isspace((unsigned char)*parse_line)) {
        ++parse_line;
    }

    bool update_only = false;
    if ((parse_line[0] == 's' || parse_line[0] == 'S') &&
        (parse_line[1] == 'e' || parse_line[1] == 'E') &&
        (parse_line[2] == 't' || parse_line[2] == 'T') &&
        isspace((unsigned char)parse_line[3])) {
        parse_line += 3;
        while (isspace((unsigned char)*parse_line)) {
            ++parse_line;
        }
        update_only = true;
    }

    xSemaphoreTake(s_command_mutex, portMAX_DELAY);

    daikin_state_t previous_state = s_state;
    command_result_t command = parse_command(parse_line, &s_state);
    if (update_only && command == COMMAND_SEND) {
        command = COMMAND_NO_SEND;
    }
    if (command == COMMAND_INVALID) {
        if (console_output) {
            printf("Unknown command or invalid value. Type 'help'.\n");
        }
        if (response != NULL) {
            snprintf(response, response_size,
                     "{\"ok\":false,\"error\":\"unknown command or invalid value\"}");
        }
        xSemaphoreGive(s_command_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    if (command == COMMAND_REBOOT) {
        if (console_output) {
            printf("Rebooting...\n");
        }
        if (response != NULL) {
            snprintf(response, response_size, "{\"ok\":true,\"message\":\"Rebooting\"}");
        }
        xSemaphoreGive(s_command_mutex);
        xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
        return ESP_OK;
    }

    if (command == COMMAND_NO_SEND) {
        if (!ac_state_equal(&previous_state, &s_state)) {
            mark_ac_state_updated_locked();
            save_ac_state_locked();
        }
        if (response != NULL) {
            format_state_json(&s_state, true, "updated", response, response_size);
        }
        xSemaphoreGive(s_command_mutex);
        return ESP_OK;
    }

    esp_err_t err = arm_loopback_expectation(&s_state);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Loopback expectation failed: %s", esp_err_to_name(err));
        if (response != NULL) {
            snprintf(response, response_size,
                     "{\"ok\":false,\"error\":\"loopback expectation failed: %s\"}",
                     esp_err_to_name(err));
        }
        xSemaphoreGive(s_command_mutex);
        return err;
    }

    rgb_led_set_ir_active(true);
    err = daikin_ir_send_state(&s_state);
    rgb_led_set_ir_active(false);
    if (err != ESP_OK) {
        ir_capture_clear_expected_daikin_frame();
        ESP_LOGE(TAG, "IR transmit failed: %s", esp_err_to_name(err));
        if (response != NULL) {
            snprintf(response, response_size,
                     "{\"ok\":false,\"error\":\"IR transmit failed: %s\"}",
                     esp_err_to_name(err));
        }
        xSemaphoreGive(s_command_mutex);
        return err;
    }

    if (console_output) {
        printf("Sent ");
        print_state(&s_state);
    }
    if (!ac_state_equal(&previous_state, &s_state)) {
        mark_ac_state_updated_locked();
        save_ac_state_locked();
    }
    if (response != NULL) {
        format_state_json(&s_state, true, "sent", response, response_size);
    }

    xSemaphoreGive(s_command_mutex);
    return ESP_OK;
}

static esp_err_t remote_command_handler(const char *command, char *response, size_t response_size)
{
    return execute_command_line(command, false, response, response_size);
}

esp_err_t ac_control_get_state(daikin_state_t *state)
{
    ESP_RETURN_ON_FALSE(state != NULL, ESP_ERR_INVALID_ARG, TAG, "state is required");
    ESP_RETURN_ON_FALSE(s_command_mutex != NULL, ESP_ERR_INVALID_STATE, TAG, "command mutex unavailable");

    xSemaphoreTake(s_command_mutex, portMAX_DELAY);
    *state = s_state;
    xSemaphoreGive(s_command_mutex);
    return ESP_OK;
}

esp_err_t ac_control_apply_state(const daikin_state_t *state, char *response, size_t response_size)
{
    ESP_RETURN_ON_FALSE(state != NULL, ESP_ERR_INVALID_ARG, TAG, "state is required");
    ESP_RETURN_ON_FALSE(s_command_mutex != NULL, ESP_ERR_INVALID_STATE, TAG, "command mutex unavailable");

    if (response != NULL && response_size > 0) {
        response[0] = '\0';
    }

    xSemaphoreTake(s_command_mutex, portMAX_DELAY);
    daikin_state_t previous_state = s_state;
    s_state = *state;
    if (s_settings.use_fahrenheit != s_state.use_fahrenheit) {
        s_settings.use_fahrenheit = s_state.use_fahrenheit;
        save_app_u8("unit_f", s_settings.use_fahrenheit ? 1 : 0);
    }

    esp_err_t err = arm_loopback_expectation(&s_state);
    if (err == ESP_OK) {
        rgb_led_set_ir_active(true);
        err = daikin_ir_send_state(&s_state);
        rgb_led_set_ir_active(false);
    }

    if (err != ESP_OK) {
        ir_capture_clear_expected_daikin_frame();
        ESP_LOGE(TAG, "IR transmit failed: %s", esp_err_to_name(err));
        if (response != NULL && response_size > 0) {
            snprintf(response, response_size,
                     "{\"ok\":false,\"error\":\"IR transmit failed: %s\"}",
                     esp_err_to_name(err));
        }
        xSemaphoreGive(s_command_mutex);
        return err;
    }

    if (!ac_state_equal(&previous_state, &s_state)) {
        mark_ac_state_updated_locked();
        save_ac_state_locked();
    }
    if (response != NULL && response_size > 0) {
        format_state_json(&s_state, true, "sent", response, response_size);
    }
    xSemaphoreGive(s_command_mutex);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP-ARC452A21");

    s_command_mutex = xSemaphoreCreateMutex();
    if (s_command_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create command mutex");
        return;
    }
    daikin_state_init_cool_fahrenheit(&s_state, 72);

    esp_err_t err = init_nvs_storage();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS unavailable for app settings: %s", esp_err_to_name(err));
    } else {
        load_app_settings();
        load_ac_state();
        apply_loaded_display_settings();
    }
    s_state_updated_us = esp_timer_get_time();

    err = init_console_uart();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART console init failed: %s", esp_err_to_name(err));
        return;
    }

    ir_capture_config_t rx_config = IR_CAPTURE_DEFAULT_CONFIG(IR_RX_GPIO);
    rx_config.mem_block_symbols = IR_RX_LIVE_MEM_BLOCK_SYMBOLS;
    err = ir_capture_init(&rx_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IR receiver init failed: %s", esp_err_to_name(err));
        return;
    }

    BaseType_t task_ok = xTaskCreate(ir_capture_task, "ir_capture", 4096, NULL, 5, NULL);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to start IR capture task");
        return;
    }

    daikin_ir_config_t tx_config = DAIKIN_IR_DEFAULT_CONFIG(IR_TX_GPIO);
    // GPIO17 drives the low-side IR LED transistor, so the output is active-high.
    // Active-low IR blaster modules can still be tested with `polarity invert`.
    tx_config.invert_out = s_settings.ir_invert_out;
    tx_config.repeat_count = s_settings.ir_repeat_count;
    tx_config.repeat_gap_ms = s_settings.ir_repeat_gap_ms;
    err = daikin_ir_init(&tx_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IR transmitter init failed: %s", esp_err_to_name(err));
        return;
    }
    err = daikin_ir_set_timing_profile(s_settings.ir_timing_profile);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IR timing restore failed: %s", esp_err_to_name(err));
        return;
    }

    err = rgb_led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "RGB LED indicator disabled: %s", esp_err_to_name(err));
    }

    iot_remote_config_t remote_config = {
        .wifi_ssid = CONFIG_ESP_ARC452A21_WIFI_SSID,
        .wifi_password = CONFIG_ESP_ARC452A21_WIFI_PASSWORD,
        .mqtt_uri = CONFIG_ESP_ARC452A21_MQTT_URI,
        .mqtt_topic_prefix = CONFIG_ESP_ARC452A21_MQTT_TOPIC_PREFIX,
        .command_handler = remote_command_handler,
    };
    err = iot_remote_start(&remote_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "IoT remote interfaces disabled: %s", esp_err_to_name(err));
    }

    if (s_settings.homekit_enabled) {
        err = homekit_accessory_start();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HomeKit disabled: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGI(TAG, "HomeKit disabled by settings");
    }

    err = matter_accessory_start();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Matter disabled: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "IR receiver input: GPIO %d", rx_config.rx_gpio);
    ESP_LOGI(TAG, "IR blaster output: GPIO %d", tx_config.tx_gpio);
    printf("\nDaikin ARC452A21 transmit/capture test\n");
    printf("IR receiver input: GPIO %d\n", rx_config.rx_gpio);
    printf("IR blaster output: GPIO %d\n", tx_config.tx_gpio);
    printf("IR activity RGB LED: GPIO %d%s\n",
           RGB_LED_GPIO,
           s_rgb_led == NULL ? " (disabled)" : "");
    printf("TX polarity: invert_out=%d carrier_active_low=%d repeat=%u gap=%" PRIu32
           " ms timing=%s\n",
           s_settings.ir_invert_out,
           tx_config.carrier_active_low,
           (unsigned)s_settings.ir_repeat_count,
           s_settings.ir_repeat_gap_ms,
           timing_name(s_settings.ir_timing_profile));
    printf("Type 'help' for commands. Temperature range: %d..%d F or %d..%d C\n\n",
           DAIKIN_MIN_TEMP_F, DAIKIN_MAX_TEMP_F,
           DAIKIN_MIN_TEMP_C, DAIKIN_MAX_TEMP_C);

    char line[96];
    char response[RESPONSE_MAX_LEN];
    while (true) {
        printf("daikin> ");
        fflush(stdout);

        read_serial_line(line, sizeof(line));
        if (line[0] == '\0') {
            continue;
        }

        err = execute_command_line(line, true, response, sizeof(response));
        if (err != ESP_OK && response[0] != '\0') {
            printf("%s\n", response);
        }
    }
}
