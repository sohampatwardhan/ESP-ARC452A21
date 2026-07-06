#include "matter_accessory.h"

#include "sdkconfig.h"

#include "esp_check.h"
#include "esp_log.h"

#include "ac_control.h"

static const char *TAG = "matter_accessory";

#if CONFIG_ESP_ARC452A21_MATTER_ENABLE

#include <esp_matter.h>

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <lib/core/CHIPError.h>
#include <platform/CHIPDeviceEvent.h>

using namespace chip::app::Clusters;
using namespace esp_matter;

static uint16_t s_endpoint_id;

static constexpr uint8_t MATTER_ROCK_LEFT_RIGHT = 0x01;
static constexpr uint8_t MATTER_ROCK_UP_DOWN = 0x02;
static constexpr uint8_t MATTER_ROCK_SWING_SUPPORT = MATTER_ROCK_LEFT_RIGHT | MATTER_ROCK_UP_DOWN;

static int16_t target_c_to_matter_centi(const daikin_state_t *state)
{
    return (int16_t)state->target_celsius * 100;
}

static uint8_t matter_system_mode(const daikin_state_t *state)
{
    if (!state->power) {
        return chip::to_underlying(Thermostat::SystemModeEnum::kOff);
    }
    switch (state->mode) {
    case DAIKIN_MODE_HEAT:
        return chip::to_underlying(Thermostat::SystemModeEnum::kHeat);
    case DAIKIN_MODE_COOL:
    case DAIKIN_MODE_DRY:
        return chip::to_underlying(Thermostat::SystemModeEnum::kCool);
    case DAIKIN_MODE_AUTO:
        return chip::to_underlying(Thermostat::SystemModeEnum::kAuto);
    case DAIKIN_MODE_FAN:
        return chip::to_underlying(Thermostat::SystemModeEnum::kFanOnly);
    }
    return chip::to_underlying(Thermostat::SystemModeEnum::kAuto);
}

static daikin_fan_t percent_to_fan(uint8_t percent)
{
    if (percent == 0) {
        return DAIKIN_FAN_AUTO;
    }
    if (percent <= 10) {
        return DAIKIN_FAN_NIGHT;
    }
    if (percent <= 30) {
        return DAIKIN_FAN_SPEED_1;
    }
    if (percent <= 50) {
        return DAIKIN_FAN_SPEED_2;
    }
    if (percent <= 70) {
        return DAIKIN_FAN_SPEED_3;
    }
    if (percent <= 90) {
        return DAIKIN_FAN_SPEED_4;
    }
    return DAIKIN_FAN_SPEED_5;
}

static uint8_t fan_to_percent(daikin_fan_t fan)
{
    switch (fan) {
    case DAIKIN_FAN_NIGHT:
        return 10;
    case DAIKIN_FAN_SPEED_1:
        return 20;
    case DAIKIN_FAN_SPEED_2:
        return 40;
    case DAIKIN_FAN_SPEED_3:
        return 60;
    case DAIKIN_FAN_SPEED_4:
        return 80;
    case DAIKIN_FAN_SPEED_5:
        return 100;
    case DAIKIN_FAN_AUTO:
        return 0;
    }
    return 0;
}

static uint8_t fan_to_matter_mode(daikin_fan_t fan)
{
    switch (fan) {
    case DAIKIN_FAN_AUTO:
        return chip::to_underlying(FanControl::FanModeEnum::kAuto);
    case DAIKIN_FAN_NIGHT:
    case DAIKIN_FAN_SPEED_1:
        return chip::to_underlying(FanControl::FanModeEnum::kLow);
    case DAIKIN_FAN_SPEED_2:
    case DAIKIN_FAN_SPEED_3:
        return chip::to_underlying(FanControl::FanModeEnum::kMedium);
    case DAIKIN_FAN_SPEED_4:
    case DAIKIN_FAN_SPEED_5:
        return chip::to_underlying(FanControl::FanModeEnum::kHigh);
    }
    return chip::to_underlying(FanControl::FanModeEnum::kAuto);
}

static daikin_fan_t matter_mode_to_fan(uint8_t mode)
{
    if (mode == chip::to_underlying(FanControl::FanModeEnum::kAuto)) {
        return DAIKIN_FAN_AUTO;
    }
    if (mode == chip::to_underlying(FanControl::FanModeEnum::kHigh)) {
        return DAIKIN_FAN_SPEED_5;
    }
    if (mode == chip::to_underlying(FanControl::FanModeEnum::kMedium)) {
        return DAIKIN_FAN_SPEED_3;
    }
    if (mode == chip::to_underlying(FanControl::FanModeEnum::kLow)) {
        return DAIKIN_FAN_SPEED_1;
    }
    return DAIKIN_FAN_AUTO;
}

static uint8_t swing_to_rock_setting(const daikin_state_t *state)
{
    uint8_t rock_setting = 0;
    if (state->swing_horizontal) {
        rock_setting |= MATTER_ROCK_LEFT_RIGHT;
    }
    if (state->swing_vertical) {
        rock_setting |= MATTER_ROCK_UP_DOWN;
    }
    return rock_setting;
}

static esp_err_t set_attr(uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t val)
{
    esp_matter::attribute_t *attr = attribute::get(s_endpoint_id, cluster_id, attribute_id);
    if (attr == nullptr) {
        return ESP_ERR_NOT_FOUND;
    }
    return attribute::set_val(attr, &val);
}

static void sync_matter_values(const daikin_state_t *state)
{
    int16_t target = target_c_to_matter_centi(state);
    uint8_t fan_percent = fan_to_percent(state->fan);
    uint8_t fan_mode = fan_to_matter_mode(state->fan);

    set_attr(OnOff::Id, OnOff::Attributes::OnOff::Id, esp_matter_bool(state->power));
    set_attr(Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id, esp_matter_nullable_int16(target));
    set_attr(Thermostat::Id, Thermostat::Attributes::OccupiedCoolingSetpoint::Id, esp_matter_int16(target));
    set_attr(Thermostat::Id, Thermostat::Attributes::OccupiedHeatingSetpoint::Id, esp_matter_int16(target));
    set_attr(Thermostat::Id, Thermostat::Attributes::SystemMode::Id, esp_matter_enum8(matter_system_mode(state)));
    set_attr(FanControl::Id, FanControl::Attributes::FanMode::Id, esp_matter_enum8(fan_mode));
    set_attr(FanControl::Id, FanControl::Attributes::PercentSetting::Id, esp_matter_nullable_uint8(fan_percent));
    set_attr(FanControl::Id, FanControl::Attributes::PercentCurrent::Id, esp_matter_uint8(fan_percent));
    set_attr(FanControl::Id, FanControl::Attributes::RockSetting::Id,
             esp_matter_bitmap8(swing_to_rock_setting(state)));
    set_attr(ThermostatUserInterfaceConfiguration::Id,
             ThermostatUserInterfaceConfiguration::Attributes::TemperatureDisplayMode::Id,
             esp_matter_enum8(state->use_fahrenheit ? 1 : 0));
}

static esp_err_t apply_matter_write(uint32_t cluster_id, uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    daikin_state_t next = {};
    ESP_RETURN_ON_ERROR(ac_control_get_state(&next), TAG, "failed to read AC state");

    bool handled = true;
    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id) {
        next.power = val->val.b;
    } else if (cluster_id == Thermostat::Id &&
               attribute_id == Thermostat::Attributes::SystemMode::Id) {
        if (val->val.u8 == chip::to_underlying(Thermostat::SystemModeEnum::kOff)) {
            next.power = false;
        } else if (val->val.u8 == chip::to_underlying(Thermostat::SystemModeEnum::kAuto)) {
            next.power = true;
            next.mode = DAIKIN_MODE_AUTO;
        } else if (val->val.u8 == chip::to_underlying(Thermostat::SystemModeEnum::kHeat)) {
            next.power = true;
            next.mode = DAIKIN_MODE_HEAT;
        } else if (val->val.u8 == chip::to_underlying(Thermostat::SystemModeEnum::kCool)) {
            next.power = true;
            next.mode = DAIKIN_MODE_COOL;
        } else if (val->val.u8 == chip::to_underlying(Thermostat::SystemModeEnum::kFanOnly)) {
            next.power = true;
            next.mode = DAIKIN_MODE_FAN;
        } else {
            handled = false;
        }
    } else if (cluster_id == Thermostat::Id &&
               (attribute_id == Thermostat::Attributes::OccupiedCoolingSetpoint::Id ||
                attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id)) {
        int16_t temp_c = (int16_t)((val->val.i16 + 50) / 100);
        ac_control_state_set_temperature_celsius(&next, (uint8_t)temp_c);
    } else if (cluster_id == FanControl::Id &&
               attribute_id == FanControl::Attributes::FanMode::Id) {
        next.fan = matter_mode_to_fan(val->val.u8);
    } else if (cluster_id == FanControl::Id &&
               attribute_id == FanControl::Attributes::PercentSetting::Id) {
        next.fan = percent_to_fan(val->val.u8);
    } else if (cluster_id == FanControl::Id &&
               attribute_id == FanControl::Attributes::RockSetting::Id) {
        next.swing_horizontal = (val->val.u8 & MATTER_ROCK_LEFT_RIGHT) != 0;
        next.swing_vertical = (val->val.u8 & MATTER_ROCK_UP_DOWN) != 0;
    } else if (cluster_id == ThermostatUserInterfaceConfiguration::Id &&
               attribute_id == ThermostatUserInterfaceConfiguration::Attributes::TemperatureDisplayMode::Id) {
        next.use_fahrenheit = val->val.u8 != 0;
    } else {
        handled = false;
    }

    if (!handled) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(ac_control_apply_state(&next, NULL, 0), TAG, "failed to apply Matter state");
    sync_matter_values(&next);
    return ESP_OK;
}

static void matter_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    (void)arg;
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Matter commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Matter commissioning session started");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Matter commissioning window opened");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Matter commissioning window closed");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Matter fabric removed");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            chip::CommissioningWindowManager &manager =
                chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto timeout = chip::System::Clock::Seconds16(300);
            if (!manager.IsCommissioningWindowOpen()) {
                CHIP_ERROR err = manager.OpenBasicCommissioningWindow(
                    timeout, chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR) {
                    ESP_LOGW(TAG, "failed to reopen Matter commissioning window: %" CHIP_ERROR_FORMAT,
                             err.Format());
                }
            }
        }
        break;
    default:
        break;
    }
}

static esp_err_t matter_identification_cb(identification::callback_type_t type,
                                          uint16_t endpoint_id,
                                          uint8_t effect_id,
                                          uint8_t effect_variant,
                                          void *priv_data)
{
    (void)priv_data;
    ESP_LOGI(TAG, "Matter identify endpoint=%u type=%u effect=%u variant=%u",
             endpoint_id, type, effect_id, effect_variant);
    return ESP_OK;
}

static esp_err_t matter_attribute_update_cb(attribute::callback_type_t type,
                                            uint16_t endpoint_id,
                                            uint32_t cluster_id,
                                            uint32_t attribute_id,
                                            esp_matter_attr_val_t *val,
                                            void *priv_data)
{
    (void)priv_data;
    if (type != PRE_UPDATE || endpoint_id != s_endpoint_id) {
        return ESP_OK;
    }
    return apply_matter_write(cluster_id, attribute_id, val);
}

esp_err_t matter_accessory_start(void)
{
    daikin_state_t state = {};
    ESP_RETURN_ON_ERROR(ac_control_get_state(&state), TAG, "failed to read AC state");

    node::config_t node_config;
    node_t *node = node::create(&node_config, matter_attribute_update_cb, matter_identification_cb);
    ESP_RETURN_ON_FALSE(node != nullptr, ESP_ERR_NO_MEM, TAG, "failed to create Matter node");

    endpoint::room_air_conditioner::config_t ac_config;
    ac_config.on_off.on_off = state.power;
    ac_config.thermostat.local_temperature = nullable<int16_t>(target_c_to_matter_centi(&state));
    ac_config.thermostat.system_mode = matter_system_mode(&state);
    ac_config.thermostat.control_sequence_of_operation =
        chip::to_underlying(Thermostat::ControlSequenceOfOperationEnum::kCoolingAndHeating);
    ac_config.thermostat.feature_flags =
        cluster::thermostat::feature::heating::get_id() |
        cluster::thermostat::feature::cooling::get_id() |
        cluster::thermostat::feature::auto_mode::get_id();
    ac_config.thermostat.features.heating.occupied_heating_setpoint = target_c_to_matter_centi(&state);
    ac_config.thermostat.features.cooling.occupied_cooling_setpoint = target_c_to_matter_centi(&state);

    endpoint_t *endpoint = endpoint::room_air_conditioner::create(node, &ac_config, ENDPOINT_FLAG_NONE, NULL);
    ESP_RETURN_ON_FALSE(endpoint != nullptr, ESP_ERR_NO_MEM, TAG, "failed to create Matter Room AC endpoint");
    s_endpoint_id = endpoint::get_id(endpoint);

    cluster::fan_control::config_t fan_config;
    fan_config.fan_mode = fan_to_matter_mode(state.fan);
    fan_config.percent_setting = nullable<uint8_t>(fan_to_percent(state.fan));
    fan_config.percent_current = fan_to_percent(state.fan);
    cluster_t *fan_cluster = cluster::fan_control::create(endpoint, &fan_config, CLUSTER_FLAG_SERVER);
    ESP_RETURN_ON_FALSE(fan_cluster != nullptr, ESP_ERR_NO_MEM, TAG, "failed to create Matter Fan cluster");

    cluster::fan_control::feature::multi_speed::config_t multi_speed_config;
    multi_speed_config.speed_max = 5;
    multi_speed_config.speed_setting = nullable<uint8_t>(fan_to_percent(state.fan) / 20);
    multi_speed_config.speed_current = fan_to_percent(state.fan) / 20;
    cluster::fan_control::feature::multi_speed::add(fan_cluster, &multi_speed_config);
    cluster::fan_control::feature::fan_auto::add(fan_cluster);
    cluster::fan_control::feature::rocking::config_t rocking_config;
    rocking_config.rock_support = MATTER_ROCK_SWING_SUPPORT;
    rocking_config.rock_setting = swing_to_rock_setting(&state);
    cluster::fan_control::feature::rocking::add(fan_cluster, &rocking_config);

    cluster::thermostat_user_interface_configuration::config_t ui_config;
    ui_config.temperature_display_mode = state.use_fahrenheit ? 1 : 0;
    cluster::thermostat_user_interface_configuration::create(endpoint, &ui_config, CLUSTER_FLAG_SERVER);

    esp_err_t err = esp_matter::start(matter_event_cb);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to start Matter");

    sync_matter_values(&state);
    ESP_LOGI(TAG, "Matter Room Air Conditioner accessory started on endpoint %u", s_endpoint_id);
    return ESP_OK;
}

#else

esp_err_t matter_accessory_start(void)
{
    ESP_LOGI(TAG, "Matter disabled");
    return ESP_OK;
}

#endif
