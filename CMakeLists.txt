cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(libmicroros)
add_executable(pico_micro_ros_example
    pico_micro_ros_example.c
    pico_uart_transport.c
)
target_link_libraries(pico_micro_ros_example
    pico_stdlib
    microros
)

target_include_directories(pico_micro_ros_example PUBLIC
    libmicroros/include
)

add_executable(pico_micro_ros_udp_example
    pico_micro_ros_udp_example.c
    picow_udp_transports.c
)
target_link_libraries(pico_micro_ros_udp_example
    pico_stdlib
    hardware_adc
    # pico_cyw43_arch_none
    pico_cyw43_arch_lwip_poll
    # pico_cyw43_arch_lwip_threadsafe_background
    microros
)

target_include_directories(pico_micro_ros_udp_example PUBLIC
    libmicroros/include
    include
)
message(STATUS "PICO_BOARD: ${PICO_BOARD}")
message(STATUS "WIFI_SSID: ${WIFI_SSID}")
message(STATUS "WIFI_PASSWORD: ${WIFI_PASSWORD}")

add_definitions(-DWIFI_SSID="${WIFI_SSID}")
add_definitions(-DWIFI_PASSWORD="${WIFI_PASSWORD}")

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 0)

pico_enable_stdio_usb(pico_micro_ros_udp_example 0)
pico_enable_stdio_uart(pico_micro_ros_udp_example 1)


add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)
pico_add_extra_outputs(pico_micro_ros_udp_example)
