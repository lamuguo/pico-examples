add_executable(simple_can)

pico_generate_pio_header(simple_can ${CMAKE_CURRENT_LIST_DIR}/simple_can.pio)

target_sources(simple_can PRIVATE simple_can.c)

target_link_libraries(simple_can PRIVATE
        pico_stdlib
        hardware_pio
)

pico_add_extra_outputs(simple_can)

# add url via pico_set_program_url
example_auto_set_url(simple_can)
