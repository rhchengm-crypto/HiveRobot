set(OLD_DEPS "C:/Users/rhche/Desktop/rlcd-qwen-asr/deps")
set(USER_LIBS "C:/Users/rhche/Documents/Arduino/libraries")
# Includes the news speech worker; reconfigure when adding source files.
file(GLOB APP_SOURCES "*.cpp")
file(GLOB_RECURSE CODEC_SOURCES "src/esp_codec_dev/*.c")
file(GLOB WEBSOCKET_SOURCES "${OLD_DEPS}/WebSockets/src/*.cpp")
file(GLOB U8G2_SOURCES "${USER_LIBS}/U8g2/src/clib/*.c" "${USER_LIBS}/U8g2/src/*.cpp")
idf_component_register(SRCS ${APP_SOURCES} ${CODEC_SOURCES} ${WEBSOCKET_SOURCES} ${U8G2_SOURCES}
 INCLUDE_DIRS "." "src/esp_codec_dev/include" "src/esp_codec_dev/interface" "src/esp_codec_dev/platform" "src/esp_codec_dev/device/include" "src/esp_codec_dev/device/priv_include"
 "${OLD_DEPS}/WebSockets/src" "${OLD_DEPS}/RlcdOpus/src" "${USER_LIBS}/U8g2/src" "${USER_LIBS}/U8g2/src/clib" "${USER_LIBS}/ArduinoJson/src"
 REQUIRES arduino microlink esp_timer esp_adc esp_http_client esp_driver_i2c esp_driver_i2s esp_driver_sdmmc sdmmc fatfs esp_hw_support esp_partition esp_psram)
target_link_libraries(${COMPONENT_LIB} PUBLIC "${OLD_DEPS}/RlcdOpus/src/esp32s3/libRlcdOpus.a")
target_compile_definitions(${COMPONENT_LIB} PUBLIC ARDUINO_USB_CDC_ON_BOOT=1 ARDUINO_USB_MODE=1 BOARD_HAS_PSRAM)
idf_component_get_property(ARDUINO_LIB arduino COMPONENT_LIB)
target_compile_definitions(${ARDUINO_LIB} PUBLIC ARDUINO_USB_CDC_ON_BOOT=1 ARDUINO_USB_MODE=1 BOARD_HAS_PSRAM)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=deprecated-declarations -Wno-error=missing-field-initializers -Wno-error=format -Wno-error=misleading-indentation)
