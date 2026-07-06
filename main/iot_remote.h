#pragma once

#include <stddef.h>

#include "esp_err.h"

typedef esp_err_t (*iot_remote_command_handler_t)(const char *command,
                                                  char *response,
                                                  size_t response_size);

typedef struct {
    const char *wifi_ssid;
    const char *wifi_password;
    const char *mqtt_uri;
    const char *mqtt_topic_prefix;
    iot_remote_command_handler_t command_handler;
} iot_remote_config_t;

esp_err_t iot_remote_start(const iot_remote_config_t *config);
