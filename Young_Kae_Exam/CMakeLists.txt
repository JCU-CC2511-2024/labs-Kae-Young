cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

set(projname "Young_Kae_Exam")

project(${projname} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

#include(example_auto_set_url.cmake)

add_executable(${projname}
        main.c
        )

target_link_libraries(${projname} pico_stdlib pico_stdio_usb hardware_pwm hardware_adc)
pico_add_extra_outputs(${projname})

