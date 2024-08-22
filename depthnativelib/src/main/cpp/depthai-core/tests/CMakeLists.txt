# Add CMake testing infrastructure
include(CTest)
enable_testing()

# Add catch2 for writing tests
hunter_add_package(Catch2)
find_package(Catch2 CONFIG REQUIRED)

# Create environments for usb & poe labels
set(test_usb_env
    # Misc
    "UBSAN_OPTIONS=halt_on_error=1"
    # DepthAI
    "DEPTHAI_PROTOCOL=usb"
)
set(test_poe_env
    # Misc
    "UBSAN_OPTIONS=halt_on_error=1"
    # DepthAI
    "DEPTHAI_PROTOCOL=tcpip;DEPTHAI_SEARCH_TIMEOUT=15000"
)

# Function for adding new tests
# Creates separate tests for USB and PoE devices
include(CMakeParseArguments)
function(dai_add_test test_name test_src)
    # parse arguments
    cmake_parse_arguments(DAT "CONFORMING" "CXX_STANDARD" "" ${ARGN})

    # check compiler for C++ standard
    if(NOT DAT_CXX_STANDARD)
        set(DAT_CXX_STANDARD 14)
    endif()
    if(NOT COMPILER_SUPPORTS_CXX${DAT_CXX_STANDARD})
        message(STATUS "Skipping test ${test_name} with c++${DAT_CXX_STANDARD}; no compiler support")
        return()
    endif()

    # Create test executable
    add_executable(${test_name} ${test_src})
    add_default_flags(${test_name} LEAN)

    # Add to clangformat target
    if(COMMAND target_clangformat_setup)
        target_clangformat_setup(${test_name} "")
    endif()

    # Link to core and Catch2 testing framework
    target_link_libraries(${test_name} PRIVATE depthai-core Catch2::Catch2WithMain Threads::Threads)

    # Add sanitizers for tests as well
    if(COMMAND add_sanitizers)
        add_sanitizers(${test_name})
    endif()

    # Set compiler c++ standard
    target_compile_options(${test_name} PRIVATE "${CMAKE_CXX${DAT_CXX_STANDARD}_STANDARD_COMPILE_OPTION}")

    # Add to list of tests
    add_test(${test_name} ${test_name})
    # Add ubsan halt on error
    set_tests_properties(${test_name} PROPERTIES ENVIRONMENT "${test_usb_env}" LABELS usb)

    # add a conforming preprocessor test (serialization macros) if MSVC and CONFORMING option set on function call
    if(MSVC AND DAT_CONFORMING)
        # Create test executable with MSVC conforming preprocessor
        add_executable(${test_name}_conforming ${test_src})
        add_default_flags(${test_name}_conforming LEAN)

        # Link to core and Catch2 testing framework
        target_link_libraries(${test_name}_conforming PRIVATE depthai-core Catch2::Catch2WithMain Threads::Threads)

        # Set compiler c++ standard
        target_compile_options(${test_name}_conforming PRIVATE "${CMAKE_CXX${DAT_CXX_STANDARD}_STANDARD_COMPILE_OPTION}")

        if(MSVC_VERSION LESS 1925)
            target_compile_options(${test_name}_conforming PRIVATE "/experimental:preprocessor")
        else()
            target_compile_options(${test_name}_conforming PRIVATE "/Zc:preprocessor")
        endif()

        # Add to list of tests
        add_test(${test_name}_conforming ${test_name}_conforming)

        # Add ubsan halt on error
        set_tests_properties(${test_name}_conforming PROPERTIES ENVIRONMENT "${test_usb_env}" LABELS usb)
    endif()

    # Add PoE test variants
    add_test(${test_name}_poe ${test_name})
    # Add ubsan halt on error
    set_tests_properties(${test_name}_poe PROPERTIES ENVIRONMENT "${test_poe_env}" LABELS poe)
    if(MSVC AND DAT_CONFORMING)
        # Add to list of tests
        add_test(${test_name}_conforming_poe ${test_name}_conforming)
        # Add ubsan halt on error
        set_tests_properties(${test_name}_conforming_poe PROPERTIES ENVIRONMENT "${test_poe_env}" LABELS poe)
    endif()

    # Copy over required DLLs (Windows)
    if(WIN32)
        # Copy dlls to target directory - Windows only
        # TARGET_RUNTIME_DLLS generator expression available since CMake 3.21
        if(CMAKE_VERSION VERSION_LESS "3.21")
            file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
        else()
            set(depthai_dll_libraries "$<TARGET_RUNTIME_DLLS:${test_name}>")
        endif()
        add_custom_command(TARGET ${test_name} POST_BUILD COMMAND
            "$<$<BOOL:${depthai_dll_libraries}>:${CMAKE_COMMAND};-E;copy_if_different;${depthai_dll_libraries};$<TARGET_FILE_DIR:${test_name}>>"
            COMMAND_EXPAND_LISTS
            VERBATIM
        )
    endif()
endfunction()

# Function for adding compile definitions to tests
function(dai_test_compile_definitions)
    if(TARGET ${ARGV0})
        target_compile_definitions(${ARGV})
        if(TARGET ${ARGV0}_conforming)
            set(ARGV0 ${ARGV0}_conforming)
            list(REMOVE_AT ARGV 0)
            list(INSERT ARGV 0 ${ARGV0})
            target_compile_definitions(${ARGV})
        endif()
    endif()
endfunction()

# Mobilenet network
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.2_8shave.blob"
    SHA1 "3329bb8f3a9c881ef9756d232055f9d6f38aa07b"
    FILE "mobilenet-ssd_openvino_2021.2_8shave.blob"
    LOCATION mobilenet_blob
)

# OpenVINO 2020.3 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2020.3_4shave.blob"
    SHA1 "f0134c9b843fe414f6d98b17a70f069d1ab0f3d8"
    FILE "text-image-super-resolution-0001_2020.3_4shave.blob"
    LOCATION openvino_2020_3_blob
)
# OpenVINO 2020.4 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2020.4_4shave.blob"
    SHA1 "25dcf0b146da8c85c9c4cba00ad5fdd4ed02a1b6"
    FILE "text-image-super-resolution-0001_2020.4_4shave.blob"
    LOCATION openvino_2020_4_blob
)

# OpenVINO 2021.1 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.1_4shave.blob"
    SHA1 "39c4f47f2a75627b7561e97dd7cdfcd0b1925a1e"
    FILE "text-image-super-resolution-0001_2021.1_4shave.blob"
    LOCATION openvino_2021_1_blob
)
# OpenVINO 2021.2 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.2_4shave.blob"
    SHA1 "a204467f86aa4ad63d31782ada271bea6f57f789"
    FILE "text-image-super-resolution-0001_2021.2_4shave.blob"
    LOCATION openvino_2021_2_blob
)
# OpenVINO 2021.3 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.3_4shave.blob"
    SHA1 "af19470feb59317e74d045bc31d93ca129c46674"
    FILE "text-image-super-resolution-0001_2021.3_4shave.blob"
    LOCATION openvino_2021_3_blob
)
# OpenVINO 2021.4.2 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2021.4.2_4shave.blob"
    SHA1 "164b6b2ae48d38bc4f07cc8296b8bcb7644a1578"
    FILE "text-image-super-resolution-0001_2021.4.2_4shave.blob"
    LOCATION openvino_2021_4_2_blob
)
# OpenVINO 2022.1.0 blob
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/text-image-super-resolution-0001_2022.1.0_4shave.blob"
    SHA1 "98e94b865b9c48a92eaebd1ddc883712dfe7cfcb"
    FILE "text-image-super-resolution-0001_2022.1.0_4shave.blob"
    LOCATION openvino_2022_1_blob
)
# YoloV4 resource
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_4shave.blob"
    SHA1 "7da2f96f7300e3828940557e6a86ac6f243eef7e"
    FILE "yolo-v4-tiny-tf_openvino_2021.4_4shave.blob"
    LOCATION tiny_yolo_v4_2021-4_4shave_blob
)


# Add tests
dai_add_test(color_camera_node_test src/color_camera_node_test.cpp)
dai_add_test(image_manip_node_test src/image_manip_node_test.cpp)
dai_add_test(pipeline_test src/pipeline_test.cpp)
dai_add_test(logging_test src/logging_test.cpp)

dai_add_test(neural_network_test src/neural_network_test.cpp)
dai_test_compile_definitions(neural_network_test PRIVATE BLOB_PATH="${mobilenet_blob}")

dai_add_test(openvino_blob_test src/openvino_blob_test.cpp)
dai_test_compile_definitions(openvino_blob_test PRIVATE
    OPENVINO_2020_3_BLOB_PATH="${openvino_2020_3_blob}"
    OPENVINO_2020_4_BLOB_PATH="${openvino_2020_4_blob}"
    OPENVINO_2021_1_BLOB_PATH="${openvino_2021_1_blob}"
    OPENVINO_2021_2_BLOB_PATH="${openvino_2021_2_blob}"
    OPENVINO_2021_3_BLOB_PATH="${openvino_2021_3_blob}"
    OPENVINO_2021_4_BLOB_PATH="${openvino_2021_4_2_blob}"
    OPENVINO_2022_1_BLOB_PATH="${openvino_2022_1_blob}"
)

# Bootloader configuration tests
dai_add_test(bootloader_config_test src/bootloader_config_test.cpp)

# Eeprom naming parsing tests
dai_add_test(naming_test src/naming_test.cpp)

# Device USB Speed and serialization macros test
dai_add_test(device_usbspeed_test    src/device_usbspeed_test.cpp CONFORMING)
dai_add_test(device_usbspeed_test_17 src/device_usbspeed_test.cpp CONFORMING CXX_STANDARD 17)
dai_add_test(device_usbspeed_test_20 src/device_usbspeed_test.cpp CONFORMING CXX_STANDARD 20)

# Unlimited io connections test
dai_add_test(unlimited_io_connection_test src/unlimited_io_connection_test.cpp)

# Serialization test
dai_add_test(serialization_test src/serialization_test.cpp)

# Multiple devices test
dai_add_test(multiple_devices_test src/multiple_devices_test.cpp)

# Filesystem test
dai_add_test(filesystem_test src/filesystem_test.cpp)
dai_test_compile_definitions(filesystem_test PRIVATE BLOB_PATH="${mobilenet_blob}")
dai_add_test(filesystem_test_17 src/filesystem_test.cpp CXX_STANDARD 17)
dai_test_compile_definitions(filesystem_test_17 PRIVATE BLOB_PATH="${mobilenet_blob}")
dai_add_test(filesystem_test_20 src/filesystem_test.cpp CXX_STANDARD 20)
dai_test_compile_definitions(filesystem_test_20 PRIVATE BLOB_PATH="${mobilenet_blob}")

# Bootloader version tests
dai_add_test(bootloader_version_test src/bootloader_version_test.cpp)

# XLinkIn -> XLinkOut passthrough with large frames
dai_add_test(xlink_roundtrip_test src/xlink_roundtrip_test.cpp)

# Stability stress test (OAK-D oriented USB/PoE)
# TODO
# # dai_add_test(stability_stress_test src/stability_stress_test.cpp)
option(DEPTHAI_STABILITY_STRESS_TEST_DEBUG "Enable visualization for stability stress test" OFF)
add_executable(stability_stress_test src/stability_stress_test.cpp)
# Set compiler features (c++14), and disables extensions (g++14)
set_property(TARGET stability_stress_test PROPERTY CXX_STANDARD 14)
set_property(TARGET stability_stress_test PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET stability_stress_test PROPERTY CXX_EXTENSIONS OFF)

add_default_flags(stability_stress_test LEAN)
if(COMMAND target_clangformat_setup)
    target_clangformat_setup(stability_stress_test "")
endif()
if(DEPTHAI_STABILITY_TEST_DEBUG)
    target_link_libraries(stability_stress_test PRIVATE depthai::opencv opencv_highgui Threads::Threads)
    target_compile_definitions(stability_stress_test PRIVATE DEPTHAI_STABILITY_TEST_DEBUG)
else()
    target_link_libraries(stability_stress_test PRIVATE depthai::core Threads::Threads)
endif()
target_compile_definitions(stability_stress_test PRIVATE BLOB_PATH="${tiny_yolo_v4_2021-4_4shave_blob}")
# add_test(stability_stress_test stability_stress_test)
# add_test(stability_stress_test_poe stability_stress_test)
# set_tests_properties(stability_stress_test PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1;DEPTHAI_PROTOCOL=usb" LABELS stability_usb)
# set_tests_properties(stability_stress_test_poe PROPERTIES ENVIRONMENT "UBSAN_OPTIONS=halt_on_error=1;DEPTHAI_PROTOCOL=poe" LABELS stability_poe)

# StreamMessageParser tests
dai_add_test(stream_message_parser_test src/stream_message_parser_test.cpp)
