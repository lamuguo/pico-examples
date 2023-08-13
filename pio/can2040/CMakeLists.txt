cmake_minimum_required(VERSION 3.13)

add_executable(can_main can_main.c)

target_sources(can_main PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/can2040.c
        ${CMAKE_CURRENT_LIST_DIR}/can_main.c
        )

#target_include_directories(host_cdc_msc_hid PUBLIC
#        ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(can_main PUBLIC
        cmsis_core
        hardware_pio
        pico_stdlib
        )

pico_add_extra_outputs(can_main)

# add url via pico_set_program_url
example_auto_set_url(can_main)
