# BR/EDR tests.
#
# Each test is a small executable that returns 0 on success and non-zero
# on failure. They are registered with ctest via add_test() so the whole
# suite can be run with `ctest` (or `ctest --output-on-failure`).

add_executable(test_fec_2_3
    test_fec_2_3.c
)

target_link_libraries(test_fec_2_3
    supertooth_core
)

add_test(NAME test_fec_2_3 COMMAND test_fec_2_3)

add_executable(test_crc
    test_crc.c
)

target_link_libraries(test_crc
    supertooth_core
)

add_test(NAME test_crc COMMAND test_crc)
