#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "daikin_ir.h"
#include "ir_capture.h"

static const char *TAG = "esp_arc452a21";

#define IR_RX_GPIO GPIO_NUM_16
#define IR_TX_GPIO GPIO_NUM_17
#define IR_RX_LIVE_MEM_BLOCK_SYMBOLS 384

typedef enum {
    COMMAND_INVALID,
    COMMAND_NO_SEND,
    COMMAND_SEND,
} command_result_t;

static void ir_capture_task(void *arg)
{
    (void)arg;

    esp_err_t err = ir_capture_run_forever();
    ESP_LOGE(TAG, "IR capture task stopped: %s", esp_err_to_name(err));
    vTaskDelete(NULL);
}

static esp_err_t init_console_uart(void)
{
    esp_err_t err = uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    return err;
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

static void print_state(const daikin_state_t *state)
{
    printf("State: power=%s mode=%s temp=%u F fan=%s vswing=%s hswing=%s quiet=%s sensor=%s\n",
           state->power ? "on" : "off",
           mode_name(state->mode),
           state->target_fahrenheit,
           fan_name(state->fan),
           state->swing_vertical ? "on" : "off",
           state->swing_horizontal ? "on" : "off",
           state->quiet ? "on" : "off",
           sensor_name(state->sensor));
}

static void print_help(void)
{
    printf("Commands send the full current state after updating it:\n");
    printf("  72 | temp 72 | on 72 | off 72\n");
    printf("  on | off\n");
    printf("  mode auto|dry|cool|heat|fan\n");
    printf("  fan 1|2|3|4|5|auto|night\n");
    printf("  vswing on|off | hswing on|off | quiet on|off\n");
    printf("  sensor off|comfort|eye|both\n");
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

static bool set_temperature(daikin_state_t *state, const char *value)
{
    char *end = NULL;
    long temp_f = strtol(value, &end, 10);
    if (value == end || *end != '\0') {
        return false;
    }
    if (temp_f < DAIKIN_MIN_TEMP_F || temp_f > DAIKIN_MAX_TEMP_F) {
        printf("Temperature must be %d..%d F\n", DAIKIN_MIN_TEMP_F, DAIKIN_MAX_TEMP_F);
        return false;
    }
    state->target_fahrenheit = (uint8_t)temp_f;
    state->target_celsius = (uint8_t)((temp_f - 32) * 5 / 9);
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

    if (strcmp(command, "on") == 0 || strcmp(command, "off") == 0) {
        state->power = strcmp(command, "on") == 0;
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value != NULL && !set_temperature(state, value)) {
            return COMMAND_INVALID;
        }
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    if (strcmp(command, "temp") == 0) {
        char *value = strtok_r(NULL, " \t", &saveptr);
        if (value == NULL || !set_temperature(state, value)) {
            return COMMAND_INVALID;
        }
        return strtok_r(NULL, " \t", &saveptr) == NULL ? COMMAND_SEND : COMMAND_INVALID;
    }

    if (set_temperature(state, command)) {
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

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP-ARC452A21");

    esp_err_t err = init_console_uart();
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
    // Some transistor/IR blaster modules are active-low. If the serial log says
    // frames are sent but the IR LED does not visibly blink, try enabling this.
    // tx_config.invert_out = true;
    err = daikin_ir_init(&tx_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IR transmitter init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "IR receiver input: GPIO %d", rx_config.rx_gpio);
    ESP_LOGI(TAG, "IR blaster output: GPIO %d", tx_config.tx_gpio);
    printf("\nDaikin ARC452A21 transmit/capture test\n");
    printf("IR receiver input: GPIO %d\n", rx_config.rx_gpio);
    printf("IR blaster output: GPIO %d\n", tx_config.tx_gpio);
    printf("TX polarity: invert_out=%d carrier_active_low=%d repeat=%u gap=%" PRIu32 " ms\n",
           tx_config.invert_out,
           tx_config.carrier_active_low,
           (unsigned)tx_config.repeat_count,
           tx_config.repeat_gap_ms);
    printf("Type 'help' for commands. Validated temperature range: %d..%d F\n\n",
           DAIKIN_MIN_TEMP_F, DAIKIN_MAX_TEMP_F);

    daikin_state_t state;
    daikin_state_init_cool_fahrenheit(&state, 72);
    char line[96];
    while (true) {
        printf("daikin> ");
        fflush(stdout);

        read_serial_line(line, sizeof(line));
        if (line[0] == '\0') {
            continue;
        }

        command_result_t command = parse_command(line, &state);
        if (command == COMMAND_INVALID) {
            printf("Unknown command or invalid value. Type 'help'.\n");
            continue;
        }
        if (command == COMMAND_NO_SEND) {
            continue;
        }

        err = daikin_ir_send_state(&state);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "IR transmit failed: %s", esp_err_to_name(err));
        } else {
            printf("Sent ");
            print_state(&state);
        }
    }
}
