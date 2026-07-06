cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/version.cmake)
if(NOT IDF_VERSION_MAJOR EQUAL 5)
    message(FATAL_ERROR
        "ESP-ARC452A21 is pinned to ESP-IDF 5.x. "
        "Source /Users/soham/.espressif/v5.5.4/esp-idf/export.sh before building.")
endif()

set(HOMEKIT_PATH "${CMAKE_CURRENT_LIST_DIR}/external/esp-homekit-sdk")
set(EXTRA_COMPONENT_DIRS
    "${HOMEKIT_PATH}/components"
    "${HOMEKIT_PATH}/components/homekit")

# ESP-IDF projects must include project.cmake before calling project().
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(MATTER_PATH "${CMAKE_CURRENT_LIST_DIR}/external/esp-matter")
set(MATTER_SDK_PATH "${MATTER_PATH}/connectedhomeip/connectedhomeip")
set(MATTER_COMPONENT_PATH "${MATTER_SDK_PATH}/config/esp32/components")

if(DEFINED ENV{ESP_ARC452A21_ENABLE_MATTER}
    AND "$ENV{ESP_ARC452A21_ENABLE_MATTER}" STREQUAL "1"
    AND EXISTS "${MATTER_COMPONENT_PATH}")
    set(MATTER_COMPONENTS_AVAILABLE TRUE)
    set(ENV{ESP_MATTER_PATH} "${MATTER_PATH}")
    if(NOT DEFINED ENV{ESP_MATTER_DEVICE_PATH})
        set(ENV{ESP_MATTER_DEVICE_PATH} "${MATTER_PATH}/device_hal/device/esp32_devkit_c")
    endif()

    include($ENV{ESP_MATTER_DEVICE_PATH}/esp_matter_device.cmake)
    include(${MATTER_PATH}/examples/common/cmake_common/components_include.cmake)

    list(APPEND EXTRA_COMPONENT_DIRS
        "${MATTER_PATH}/examples/common"
        "${MATTER_COMPONENT_PATH}"
        "${MATTER_PATH}/components"
        "${MATTER_PATH}/device_hal/device"
        ${extra_components_dirs_append})
endif()

project(esp_arc452a21)

if(MATTER_COMPONENTS_AVAILABLE)
    idf_build_set_property(CXX_COMPILE_OPTIONS
        "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual"
        APPEND)
    idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
    idf_build_set_property(COMPILE_OPTIONS
        "-Wno-format-nonliteral;-Wno-format-security"
        APPEND)
endif()
