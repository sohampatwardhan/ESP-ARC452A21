#include "homekit_accessory.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ac_control.h"

#if CONFIG_ESP_ARC452A21_HOMEKIT_ENABLE
#include "hap.h"
#include "hap_apple_chars.h"
#include "hap_apple_servs.h"
#endif

static const char *TAG = "homekit_accessory";

#if CONFIG_ESP_ARC452A21_HOMEKIT_ENABLE

#define HOMEKIT_ACTIVE_INACTIVE 0
#define HOMEKIT_ACTIVE_ACTIVE 1
#define HOMEKIT_CURRENT_INACTIVE 0
#define HOMEKIT_CURRENT_IDLE 1
#define HOMEKIT_CURRENT_HEATING 2
#define HOMEKIT_CURRENT_COOLING 3
#define HOMEKIT_TARGET_AUTO 0
#define HOMEKIT_TARGET_HEAT 1
#define HOMEKIT_TARGET_COOL 2
#define HOMEKIT_TEMP_CELSIUS 0
#define HOMEKIT_TEMP_FAHRENHEIT 1
#define HOMEKIT_SWING_DISABLED 0
#define HOMEKIT_SWING_ENABLED 1

typedef struct {
    hap_serv_t *service;
    hap_char_t *active;
    hap_char_t *current_temperature;
    hap_char_t *current_state;
    hap_char_t *target_state;
    hap_char_t *cooling_threshold;
    hap_char_t *heating_threshold;
    hap_char_t *rotation_speed;
    hap_char_t *temperature_units;
    hap_char_t *swing_mode;
} homekit_chars_t;

static homekit_chars_t s_hk;
static bool s_homekit_started;
static bool s_homekit_pairing;
static bool s_homekit_pairing_timed_out;
static int s_homekit_connected_controllers;

static float state_target_celsius(const daikin_state_t *state)
{
    return (float)state->target_celsius;
}

static uint8_t current_heater_cooler_state(const daikin_state_t *state)
{
    if (!state->power) {
        return HOMEKIT_CURRENT_INACTIVE;
    }
    if (state->mode == DAIKIN_MODE_HEAT) {
        return HOMEKIT_CURRENT_HEATING;
    }
    if (state->mode == DAIKIN_MODE_COOL || state->mode == DAIKIN_MODE_DRY) {
        return HOMEKIT_CURRENT_COOLING;
    }
    return HOMEKIT_CURRENT_IDLE;
}

static uint8_t target_heater_cooler_state(const daikin_state_t *state)
{
    if (state->mode == DAIKIN_MODE_HEAT) {
        return HOMEKIT_TARGET_HEAT;
    }
    if (state->mode == DAIKIN_MODE_COOL) {
        return HOMEKIT_TARGET_COOL;
    }
    return HOMEKIT_TARGET_AUTO;
}

static float fan_to_rotation_speed(daikin_fan_t fan)
{
    switch (fan) {
    case DAIKIN_FAN_SPEED_1:
        return 20.0f;
    case DAIKIN_FAN_SPEED_2:
        return 40.0f;
    case DAIKIN_FAN_SPEED_3:
        return 60.0f;
    case DAIKIN_FAN_SPEED_4:
        return 80.0f;
    case DAIKIN_FAN_SPEED_5:
        return 100.0f;
    case DAIKIN_FAN_NIGHT:
        return 10.0f;
    case DAIKIN_FAN_AUTO:
        return 50.0f;
    }
    return 50.0f;
}

static daikin_fan_t rotation_speed_to_fan(float speed)
{
    if (speed <= 10.0f) {
        return DAIKIN_FAN_NIGHT;
    }
    if (speed <= 30.0f) {
        return DAIKIN_FAN_SPEED_1;
    }
    if (speed <= 50.0f) {
        return DAIKIN_FAN_SPEED_2;
    }
    if (speed <= 70.0f) {
        return DAIKIN_FAN_SPEED_3;
    }
    if (speed <= 90.0f) {
        return DAIKIN_FAN_SPEED_4;
    }
    return DAIKIN_FAN_SPEED_5;
}

static uint8_t clamp_target_celsius(float value)
{
    long rounded = lroundf(value);
    if (rounded < DAIKIN_MIN_TEMP_C) {
        rounded = DAIKIN_MIN_TEMP_C;
    } else if (rounded > DAIKIN_MAX_TEMP_C) {
        rounded = DAIKIN_MAX_TEMP_C;
    }
    return (uint8_t)rounded;
}

static float homekit_heating_threshold(const daikin_state_t *state)
{
    float temp = state_target_celsius(state);
    return temp > 25.0f ? 25.0f : temp;
}

static void update_char(hap_char_t *hc, hap_val_t val)
{
    if (hc != NULL) {
        hap_char_update_val(hc, &val);
    }
}

static void sync_homekit_values(const daikin_state_t *state)
{
    float target_c = state_target_celsius(state);

    update_char(s_hk.active, (hap_val_t){.u = state->power ? HOMEKIT_ACTIVE_ACTIVE : HOMEKIT_ACTIVE_INACTIVE});
    update_char(s_hk.current_temperature, (hap_val_t){.f = target_c});
    update_char(s_hk.current_state, (hap_val_t){.u = current_heater_cooler_state(state)});
    update_char(s_hk.target_state, (hap_val_t){.u = target_heater_cooler_state(state)});
    update_char(s_hk.cooling_threshold, (hap_val_t){.f = target_c});
    update_char(s_hk.heating_threshold, (hap_val_t){.f = homekit_heating_threshold(state)});
    update_char(s_hk.rotation_speed, (hap_val_t){.f = fan_to_rotation_speed(state->fan)});
    update_char(s_hk.temperature_units,
                (hap_val_t){.u = state->use_fahrenheit ? HOMEKIT_TEMP_FAHRENHEIT : HOMEKIT_TEMP_CELSIUS});
    update_char(s_hk.swing_mode,
                (hap_val_t){.u = state->swing_vertical ? HOMEKIT_SWING_ENABLED : HOMEKIT_SWING_DISABLED});
}

static bool apply_homekit_write(daikin_state_t *state, const hap_write_data_t *write)
{
    const char *uuid = hap_char_get_type_uuid(write->hc);

    if (!strcmp(uuid, HAP_CHAR_UUID_ACTIVE)) {
        state->power = write->val.u == HOMEKIT_ACTIVE_ACTIVE;
        return true;
    }
    if (!strcmp(uuid, HAP_CHAR_UUID_TARGET_HEATER_COOLER_STATE)) {
        if (write->val.u == HOMEKIT_TARGET_AUTO) {
            state->mode = DAIKIN_MODE_AUTO;
        } else if (write->val.u == HOMEKIT_TARGET_HEAT) {
            state->mode = DAIKIN_MODE_HEAT;
        } else if (write->val.u == HOMEKIT_TARGET_COOL) {
            state->mode = DAIKIN_MODE_COOL;
        } else {
            return false;
        }
        return true;
    }
    if (!strcmp(uuid, HAP_CHAR_UUID_COOLING_THRESHOLD_TEMPERATURE) ||
        !strcmp(uuid, HAP_CHAR_UUID_HEATING_THRESHOLD_TEMPERATURE)) {
        ac_control_state_set_temperature_celsius(state, clamp_target_celsius(write->val.f));
        return true;
    }
    if (!strcmp(uuid, HAP_CHAR_UUID_ROTATION_SPEED)) {
        state->fan = rotation_speed_to_fan(write->val.f);
        return true;
    }
    if (!strcmp(uuid, HAP_CHAR_UUID_TEMPERATURE_DISPLAY_UNITS)) {
        state->use_fahrenheit = write->val.u == HOMEKIT_TEMP_FAHRENHEIT;
        return true;
    }
    if (!strcmp(uuid, HAP_CHAR_UUID_SWING_MODE)) {
        state->swing_vertical = write->val.u == HOMEKIT_SWING_ENABLED;
        return true;
    }

    return false;
}

static int heat_pump_write(hap_write_data_t write_data[], int count,
                           void *serv_priv, void *write_priv)
{
    (void)serv_priv;
    (void)write_priv;

    daikin_state_t next = {};
    if (ac_control_get_state(&next) != ESP_OK) {
        for (int i = 0; i < count; ++i) {
            *(write_data[i].status) = HAP_STATUS_COMM_ERR;
        }
        return HAP_FAIL;
    }

    bool valid = true;
    for (int i = 0; i < count; ++i) {
        if (!apply_homekit_write(&next, &write_data[i])) {
            *(write_data[i].status) = HAP_STATUS_RES_ABSENT;
            valid = false;
        } else {
            *(write_data[i].status) = HAP_STATUS_SUCCESS;
        }
    }
    if (!valid) {
        return HAP_FAIL;
    }

    esp_err_t err = ac_control_apply_state(&next, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HomeKit state apply failed: %s", esp_err_to_name(err));
        for (int i = 0; i < count; ++i) {
            *(write_data[i].status) = HAP_STATUS_COMM_ERR;
        }
        return HAP_FAIL;
    }

    sync_homekit_values(&next);
    return HAP_SUCCESS;
}

static int heat_pump_identify(hap_acc_t *ha)
{
    (void)ha;
    ESP_LOGI(TAG, "HomeKit identify requested");
    return HAP_SUCCESS;
}

static void homekit_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event, void *data)
{
    (void)arg;
    (void)event_base;

    switch (event) {
    case HAP_EVENT_PAIRING_STARTED:
        s_homekit_pairing = true;
        s_homekit_pairing_timed_out = false;
        ESP_LOGI(TAG, "HomeKit pairing started");
        break;
    case HAP_EVENT_PAIRING_ABORTED:
        s_homekit_pairing = false;
        ESP_LOGI(TAG, "HomeKit pairing aborted");
        break;
    case HAP_EVENT_CTRL_PAIRED:
        s_homekit_pairing = false;
        s_homekit_pairing_timed_out = false;
        ESP_LOGI(TAG, "HomeKit controller paired: %s", (char *)data);
        break;
    case HAP_EVENT_CTRL_UNPAIRED:
        ESP_LOGI(TAG, "HomeKit controller removed: %s", (char *)data);
        break;
    case HAP_EVENT_CTRL_CONNECTED:
        ++s_homekit_connected_controllers;
        ESP_LOGI(TAG, "HomeKit controller connected");
        break;
    case HAP_EVENT_CTRL_DISCONNECTED:
        if (s_homekit_connected_controllers > 0) {
            --s_homekit_connected_controllers;
        }
        ESP_LOGI(TAG, "HomeKit controller disconnected");
        break;
    case HAP_EVENT_PAIRING_MODE_TIMED_OUT:
        s_homekit_pairing = false;
        s_homekit_pairing_timed_out = true;
        ESP_LOGI(TAG, "HomeKit pairing mode timed out");
        break;
    default:
        break;
    }
}

esp_err_t homekit_accessory_start(void)
{
    daikin_state_t state = {};
    ESP_RETURN_ON_ERROR(ac_control_get_state(&state), TAG, "failed to read AC state");

    hap_cfg_t hap_cfg = {};
    hap_get_config(&hap_cfg);
    hap_cfg.unique_param = UNIQUE_NAME;
    hap_set_config(&hap_cfg);

    ESP_RETURN_ON_FALSE(hap_init(HAP_TRANSPORT_WIFI) == HAP_SUCCESS,
                        ESP_FAIL, TAG, "hap_init failed");

    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char serial[18] = {};
    snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    hap_acc_cfg_t cfg = {
        .name = "ESP-ARC452A21",
        .manufacturer = "Soham",
        .model = "Daikin ARC452A21 Heat Pump",
        .serial_num = serial,
        .fw_rev = "0.1.0",
        .hw_rev = NULL,
        .pv = "1.1.0",
        .identify_routine = heat_pump_identify,
        .cid = HAP_CID_AIR_CONDITIONER,
    };

    hap_acc_t *accessory = hap_acc_create(&cfg);
    ESP_RETURN_ON_FALSE(accessory != NULL, ESP_ERR_NO_MEM, TAG, "failed to create accessory");

    uint8_t product_data[] = {'E', 'A', '2', '1', 'D', 'A', 'I', 'K'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    s_hk.service = hap_serv_heater_cooler_create(state.power ? HOMEKIT_ACTIVE_ACTIVE : HOMEKIT_ACTIVE_INACTIVE,
                                                 state_target_celsius(&state),
                                                 current_heater_cooler_state(&state),
                                                 target_heater_cooler_state(&state));
    ESP_RETURN_ON_FALSE(s_hk.service != NULL, ESP_ERR_NO_MEM, TAG, "failed to create heat pump service");

    hap_serv_add_char(s_hk.service, hap_char_name_create("Heat Pump"));
    hap_serv_add_char(s_hk.service, hap_char_cooling_threshold_temperature_create(state_target_celsius(&state)));
    hap_serv_add_char(s_hk.service, hap_char_heating_threshold_temperature_create(homekit_heating_threshold(&state)));
    hap_serv_add_char(s_hk.service, hap_char_rotation_speed_create(fan_to_rotation_speed(state.fan)));
    hap_serv_add_char(s_hk.service,
                      hap_char_temperature_display_units_create(state.use_fahrenheit ?
                                                               HOMEKIT_TEMP_FAHRENHEIT :
                                                               HOMEKIT_TEMP_CELSIUS));
    hap_serv_add_char(s_hk.service, hap_char_swing_mode_create(state.swing_vertical ?
                                                               HOMEKIT_SWING_ENABLED :
                                                               HOMEKIT_SWING_DISABLED));

    s_hk.active = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_ACTIVE);
    s_hk.current_temperature = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_CURRENT_TEMPERATURE);
    s_hk.current_state = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_CURRENT_HEATER_COOLER_STATE);
    s_hk.target_state = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_TARGET_HEATER_COOLER_STATE);
    s_hk.cooling_threshold = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_COOLING_THRESHOLD_TEMPERATURE);
    s_hk.heating_threshold = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_HEATING_THRESHOLD_TEMPERATURE);
    s_hk.rotation_speed = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_ROTATION_SPEED);
    s_hk.temperature_units = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_TEMPERATURE_DISPLAY_UNITS);
    s_hk.swing_mode = hap_serv_get_char_by_uuid(s_hk.service, HAP_CHAR_UUID_SWING_MODE);

    hap_serv_set_write_cb(s_hk.service, heat_pump_write);
    hap_acc_add_serv(accessory, s_hk.service);
    hap_add_accessory(accessory);

    hap_set_setup_code(CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_CODE);
    hap_set_setup_id(CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_ID);
    hap_enable_mfi_auth(HAP_MFI_AUTH_NONE);

    esp_event_handler_register(HAP_EVENT, ESP_EVENT_ANY_ID, homekit_event_handler, NULL);

    ESP_RETURN_ON_FALSE(hap_start() == HAP_SUCCESS, ESP_FAIL, TAG, "hap_start failed");
    s_homekit_started = true;
    s_homekit_pairing = false;
    s_homekit_pairing_timed_out = false;

    char *payload = esp_hap_get_setup_payload(CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_CODE,
                                              CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_ID,
                                              false,
                                              cfg.cid);
    ESP_LOGI(TAG, "HomeKit accessory ready on port %d", CONFIG_HAP_HTTP_SERVER_PORT);
    ESP_LOGI(TAG, "Pairing code: %s", CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_CODE);
    if (payload != NULL) {
        ESP_LOGI(TAG, "Setup payload: %s", payload);
        free(payload);
    }
    return ESP_OK;
}

void homekit_accessory_get_status(homekit_accessory_status_t *status)
{
    if (status == NULL) {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->compiled = true;
    status->started = s_homekit_started;
    status->pairing = s_homekit_pairing;
    status->pairing_timed_out = s_homekit_pairing_timed_out;
    status->connected_controller_count = s_homekit_connected_controllers;
    strlcpy(status->setup_code, CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_CODE,
            sizeof(status->setup_code));
    strlcpy(status->setup_id, CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_ID,
            sizeof(status->setup_id));
    if (s_homekit_started) {
        status->paired_controller_count = hap_get_paired_controller_count();
    }

    char *payload = esp_hap_get_setup_payload(CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_CODE,
                                              CONFIG_ESP_ARC452A21_HOMEKIT_SETUP_ID,
                                              false,
                                              HAP_CID_AIR_CONDITIONER);
    if (payload != NULL) {
        strlcpy(status->setup_payload, payload, sizeof(status->setup_payload));
        free(payload);
    }
}

#else

esp_err_t homekit_accessory_start(void)
{
    ESP_LOGI(TAG, "HomeKit disabled");
    return ESP_OK;
}

void homekit_accessory_get_status(homekit_accessory_status_t *status)
{
    if (status != NULL) {
        memset(status, 0, sizeof(*status));
    }
}

#endif
