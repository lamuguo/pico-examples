add_executable(tmc2209
        tmc2209.c
        )

# pull in common dependencies
target_link_libraries(tmc2209 pico_stdlib)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(tmc2209)

# add url via pico_set_program_url
example_auto_set_url(tmc2209)
