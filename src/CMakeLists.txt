# Find required libraries
find_package(PkgConfig)

# On macOS, add common Homebrew paths
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew" "/usr/local")
    set(CMAKE_FIND_FRAMEWORK NEVER)
endif()

# Try to find libraries using pkg-config first
if(PkgConfig_FOUND)
    pkg_check_modules(HACKRF libhackrf)
    pkg_check_modules(LIQUID liquid-dsp)
    pkg_check_modules(BTBB libbtbb)
endif()

# Set up proper library paths for pkg-config found libraries
if(HACKRF_FOUND)
    # Convert library names to full paths
    find_library(HACKRF_LIBRARY_PATH NAMES hackrf libhackrf
        PATHS ${HACKRF_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(HACKRF_LIBRARY_PATH)
        set(HACKRF_LIBRARIES ${HACKRF_LIBRARY_PATH})
    endif()
endif()

if(BTBB_FOUND)
    # Convert library names to full paths
    find_library(BTBB_LIBRARY_PATH NAMES btbb libbtbb
        PATHS ${BTBB_LIBRARY_DIRS}
        NO_DEFAULT_PATH)
    if(BTBB_LIBRARY_PATH)
        set(BTBB_LIBRARIES ${BTBB_LIBRARY_PATH})
    endif()
endif()

# Fallback to find_package if pkg-config didn't find them
if(NOT LIQUID_FOUND)
    find_library(LIQUID_LIBRARIES NAMES liquid 
        PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(LIQUID_INCLUDE_DIRS liquid/liquid.h 
        PATHS /opt/homebrew/include /usr/local/include)
    
    if(LIQUID_LIBRARIES AND LIQUID_INCLUDE_DIRS)
        set(LIQUID_FOUND TRUE)
        message(STATUS "Found Liquid DSP manually: ${LIQUID_LIBRARIES}")
    endif()
endif()

# Fallback for HackRF if pkg-config didn't find it
if(NOT HACKRF_FOUND)
    find_library(HACKRF_LIBRARIES NAMES hackrf libhackrf
        PATHS /opt/homebrew/lib /usr/local/lib)
    find_path(HACKRF_INCLUDE_DIRS libhackrf/hackrf.h 
        PATHS /opt/homebrew/include /usr/local/include)
    
    if(HACKRF_LIBRARIES AND HACKRF_INCLUDE_DIRS)
        set(HACKRF_FOUND TRUE)
        message(STATUS "Found HackRF manually: ${HACKRF_LIBRARIES}")
    endif()
endif()

# Fallback for BTBB if pkg-config didn't find it - search more locations
if(NOT BTBB_FOUND)
    find_library(BTBB_LIBRARIES NAMES btbb libbtbb
        PATHS 
            /opt/homebrew/lib 
            /usr/local/lib
            /usr/lib
            /opt/local/lib)
    find_path(BTBB_INCLUDE_DIRS btbb.h 
        PATHS 
            /opt/homebrew/include
            /usr/local/include
            /usr/include
            /opt/local/include)
    
    if(BTBB_LIBRARIES AND BTBB_INCLUDE_DIRS)
        set(BTBB_FOUND TRUE)
        message(STATUS "Found BTBB manually: ${BTBB_LIBRARIES}")
    else()
        # Try to find it in a libbtbb subdirectory
        find_path(BTBB_INCLUDE_DIRS btbb.h 
            PATHS 
                /opt/homebrew/include/libbtbb
                /usr/local/include/libbtbb
                /usr/include/libbtbb
                /opt/local/include/libbtbb)
        
        if(BTBB_INCLUDE_DIRS)
            set(BTBB_FOUND TRUE)
            message(STATUS "Found BTBB headers in subdirectory: ${BTBB_INCLUDE_DIRS}")
        endif()
    endif()
endif()

# Check if all libraries were found
if(NOT HACKRF_FOUND)
    message(FATAL_ERROR "HackRF library not found. Please install with: brew install hackrf")
endif()

if(NOT LIQUID_FOUND)
    message(FATAL_ERROR "Liquid DSP library not found. Please install with: brew install liquid-dsp")
endif()

if(NOT BTBB_FOUND)
    message(FATAL_ERROR "BTBB library is required for this project")
endif()

find_package(Threads REQUIRED)

set(SUPERTOOTH_SOURCE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/apps
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/radio
    ${CMAKE_CURRENT_SOURCE_DIR}/dsp
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/ble
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/bredr
    ${CMAKE_CURRENT_SOURCE_DIR}/service
)

add_library(supertooth_radio_support STATIC
    radio/radio_common.c
    radio/radio_common.h
    radio/sample_dispatcher.c
    radio/sample_dispatcher.h
    radio/hackrf.c
    radio/hackrf.h
)

target_link_libraries(supertooth_radio_support
    ${HACKRF_LIBRARIES}
)

target_include_directories(supertooth_radio_support PUBLIC
    ${HACKRF_INCLUDE_DIRS}
    ${SUPERTOOTH_SOURCE_INCLUDE_DIRS}
)

if(HACKRF_CFLAGS_OTHER)
    target_compile_options(supertooth_radio_support PRIVATE ${HACKRF_CFLAGS_OTHER})
endif()

add_library(supertooth_protocol_core STATIC
    models/receive_event_models.h
    protocol/ble/ble_display.c protocol/ble/ble_display.h
    protocol/ble/ble_codec.c protocol/ble/ble_codec.h
    protocol/ble/ble_bitstream_decoder.c protocol/ble/ble_bitstream_decoder.h
    protocol/ble/bt_assigned_numbers.c protocol/ble/bt_assigned_numbers.h
    protocol/bredr/bredr_display.c protocol/bredr/bredr_display.h
    protocol/bredr/bredr_codec.c protocol/bredr/bredr_codec.h
    protocol/bredr/bredr_bitstream_decoder.c protocol/bredr/bredr_bitstream_decoder.h
    protocol/bredr/bredr_piconet.c protocol/bredr/bredr_piconet.h
    protocol/bredr/bredr_piconet_store.c protocol/bredr/bredr_piconet_store.h
    protocol/bredr/bredr_recovery.c protocol/bredr/bredr_recovery.h
    protocol/bredr/bredr_recovery_backend.h
    protocol/bredr/bredr_recovery_btbb.c protocol/bredr/bredr_recovery_btbb.h
)

target_link_libraries(supertooth_protocol_core
    ${LIQUID_LIBRARIES}
    ${BTBB_LIBRARIES}
)

target_include_directories(supertooth_protocol_core PUBLIC
    ${LIQUID_INCLUDE_DIRS}
    ${BTBB_INCLUDE_DIRS}
    ${SUPERTOOTH_SOURCE_INCLUDE_DIRS}
)

if(LIQUID_CFLAGS_OTHER)
    target_compile_options(supertooth_protocol_core PRIVATE ${LIQUID_CFLAGS_OTHER})
endif()
if(BTBB_CFLAGS_OTHER)
    target_compile_options(supertooth_protocol_core PRIVATE ${BTBB_CFLAGS_OTHER})
endif()

if(UNIX AND NOT APPLE)
    target_link_libraries(supertooth_protocol_core m)
endif()

add_library(supertooth_service STATIC
    service/ble_channel_processor.c
    service/ble_channel_processor.h
    service/bredr_channel_processor.c
    service/bredr_channel_processor.h
    service/receiver_session.c
    service/receiver_session.h
    service/receiver_session_internal.h
    dsp/rssi_measurements.c
    dsp/rssi_measurements.h
    service/receiver_session_ble.c
    service/receiver_session_bredr.c
    service/receiver_session_hybrid.c
)

target_link_libraries(supertooth_service
    supertooth_radio_support
    supertooth_protocol_core
)

target_include_directories(supertooth_service PUBLIC
    ${SUPERTOOTH_SOURCE_INCLUDE_DIRS}
)

add_library(supertooth_app_support STATIC
    apps/app_common.c
    apps/app_common.h
)

target_link_libraries(supertooth_app_support
    supertooth_service
)

target_include_directories(supertooth_app_support PUBLIC
    ${SUPERTOOTH_SOURCE_INCLUDE_DIRS}
)

add_executable(supertooth-rx
    apps/supertooth-rx.c
)

target_link_libraries(supertooth-rx 
    supertooth_app_support
    supertooth_service
)

target_include_directories(supertooth-rx PRIVATE
    ${SUPERTOOTH_SOURCE_INCLUDE_DIRS}
)

# Add supertooth-ble executable (BLE advertising packet detector)
add_executable(supertooth-ble
    apps/supertooth-ble.c
)

target_link_libraries(supertooth-ble 
    supertooth_app_support
    supertooth_service
)

target_include_directories(supertooth-ble PRIVATE
    ${SUPERTOOTH_SOURCE_INCLUDE_DIRS}
)

# supertooth-hybrid: BR/EDR ch0-19 + BLE ch37 simultaneous capture
add_executable(supertooth-hybrid
    apps/supertooth-hybrid.c
)

target_link_libraries(supertooth-hybrid
    supertooth_app_support
    supertooth_service
)

target_include_directories(supertooth-hybrid PRIVATE
    ${SUPERTOOTH_SOURCE_INCLUDE_DIRS}
)
