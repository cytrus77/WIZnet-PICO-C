set(TARGET_NAME w5x00_mqtt_publish_subscribe_ztrans_fan)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.cpp
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib
        hardware_adc
        hardware_spi
        hardware_dma
        hardware_pwm
        ETHERNET_FILES
        IOLIBRARY_FILES
        DHCP_FILES
        DNS_FILES
        MQTT_FILES
        TIMER_FILES
        )

pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 0)

pico_add_extra_outputs(w5x00_mqtt_publish_subscribe_ztrans_fan)

add_compile_definitions(MAX_MESSAGE_HANDLERS=50)
