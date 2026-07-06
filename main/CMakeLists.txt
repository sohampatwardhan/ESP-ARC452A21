set(srcs
    "app_main.c"
    "homekit_accessory.c"
    "iot_remote.c"
    "matter_accessory.cpp")

set(requires
    daikin_ir
    ir_capture
    esp_event
    esp_http_server
    esp_netif
    esp_wifi
    esp_driver_uart
    esp_hap_apple_profiles
    esp_hap_core
    led_strip
    lwip
    mqtt
    nvs_flash)

if(CONFIG_ESP_ARC452A21_MATTER_ENABLE)
    if(NOT DEFINED ENV{ESP_ARC452A21_ENABLE_MATTER}
        OR NOT "$ENV{ESP_ARC452A21_ENABLE_MATTER}" STREQUAL "1")
        message(FATAL_ERROR
            "Matter is enabled in Kconfig, but ESP_ARC452A21_ENABLE_MATTER=1 "
            "is not set in the build environment.")
    endif()

    set(MATTER_COMPONENT_PATH
        "${CMAKE_CURRENT_LIST_DIR}/../external/esp-matter/connectedhomeip/connectedhomeip/config/esp32/components")
    if(NOT EXISTS "${MATTER_COMPONENT_PATH}")
        message(FATAL_ERROR
            "Matter is enabled, but ESP-Matter submodules are not initialized. "
            "Run the platform-specific ESP-Matter checkout documented in README.md.")
    endif()

    list(APPEND requires esp_matter)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires})
