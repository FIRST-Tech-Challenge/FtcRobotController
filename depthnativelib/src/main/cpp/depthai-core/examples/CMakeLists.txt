# Add an option to test examples
option(DEPTHAI_TEST_EXAMPLES "Test examples - examples will be ran as a part of test suite" OFF)

# Dependencies
find_package(OpenCV REQUIRED)
find_package(Sanitizers)

# Create utility library
add_library(utility utility/utility.cpp)
target_include_directories(utility PUBLIC "utility")
add_default_flags(utility LEAN)
target_link_libraries(utility FP16::fp16 ${OpenCV_LIBS})

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

# Helper for adding new examples
function(dai_add_example example_name example_src enable_test)

    # Add example
    add_executable(${example_name} ${example_src})
    add_default_flags(${example_name} LEAN)
    target_link_libraries(${example_name} PRIVATE utility depthai::opencv ${OpenCV_LIBS} Threads::Threads)
    # Set compiler features (c++14), and disables extensions (g++14)
    set_property(TARGET ${example_name} PROPERTY CXX_STANDARD 14)
    set_property(TARGET ${example_name} PROPERTY CXX_STANDARD_REQUIRED ON)
    set_property(TARGET ${example_name} PROPERTY CXX_EXTENSIONS OFF)

    # Add sanitizers for example
    if(COMMAND add_sanitizers)
        add_sanitizers(${example_name})
    endif()
    # Add to clangformat target
    if(COMMAND target_clangformat_setup)
        target_clangformat_setup(${example_name} "")
    endif()

    # parse the rest of the arguments
    set(arguments ${ARGV})
    list(REMOVE_AT arguments 0 1 2)

    # If 'DEPTHAI_TEST_EXAMPLES' is ON, then examples will be part of the test suite
    if(${enable_test} AND DEPTHAI_TEST_EXAMPLES)
        add_test(NAME ${example_name} COMMAND ${CMAKE_COMMAND}
            -DTIMEOUT_SECONDS=10
            -P ${CMAKE_CURRENT_LIST_DIR}/cmake/ExecuteTestTimeout.cmake $<TARGET_FILE:${example_name}> ${arguments}
        )

        # Sets a regex catching any logged warnings, errors or critical (coming either from device or host)
        set_tests_properties(${example_name} PROPERTIES FAIL_REGULAR_EXPRESSION "\\[warning\\];\\[error\\];\\[critical\\]")

        # Add ubsan halt on error
        set_tests_properties(${example_name} PROPERTIES ENVIRONMENT "${test_usb_env}" LABELS usb)

        # Add PoE test variants
        add_test(NAME ${example_name}_poe COMMAND ${CMAKE_COMMAND}
            -DTIMEOUT_SECONDS=50
            -P ${CMAKE_CURRENT_LIST_DIR}/cmake/ExecuteTestTimeout.cmake $<TARGET_FILE:${example_name}> ${arguments}
        )
        # Add ubsan halt on error
        set_tests_properties(${example_name}_poe PROPERTIES ENVIRONMENT "${test_poe_env}" LABELS poe)

    endif()

    # Copy over required DLLs (Windows)
    if(WIN32)
        # Copy dlls to target directory - Windows only
        # TARGET_RUNTIME_DLLS generator expression available since CMake 3.21
        if(CMAKE_VERSION VERSION_LESS "3.21")
            file(GLOB depthai_dll_libraries "${HUNTER_INSTALL_PREFIX}/bin/*.dll")
        else()
            set(depthai_dll_libraries "$<TARGET_RUNTIME_DLLS:${example_name}>")
        endif()
        add_custom_command(TARGET ${example_name} POST_BUILD COMMAND
            "$<$<BOOL:${depthai_dll_libraries}>:${CMAKE_COMMAND};-E;copy_if_different;${depthai_dll_libraries};$<TARGET_FILE_DIR:${example_name}>>"
            COMMAND_EXPAND_LISTS
            VERBATIM
        )
    endif()
endfunction()

# Create a custom target which runs all examples for 10 seconds max, and check if they executed without errors



# Add any custom binary data using the following snippet
## # Hunter test data download
## hunter_private_data(
##     URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet.blob"
##     SHA1 "e89d3ee9f26d80397e44f89c6b375990064a4a42"
##     FILE "mobilenet.blob"
##     LOCATION mobilenet_blob
## )


# Hunter test data download

# NeuralNetwork node, mobilenet example
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_6shave.blob"
    SHA1 "4f4506726e3083981064938a0faaf9af6180d2c6"
    FILE "mobilenet-ssd_openvino_2021.4_6shave.blob"
    LOCATION mobilenet_blob
)

# YoloV3 resource
hunter_private_data(
    URL "http://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
    SHA1 "dedb2d4d96b23e42d15c15e454b8f02eca2713de"
    FILE "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
    LOCATION tiny_yolo_v3_blob
)

# YoloV4 resource
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"
    SHA1 "d8d09b697dac298fe83cf8856740a21b1a61ab89"
    FILE "yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"
    LOCATION tiny_yolo_v4_blob
)

# NeuralNetwork node, mobilenet example, 5 shaves
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_5shave.blob"
    SHA1 "14ab3ef446b8a649d6694d94c2fc8b798efd74a3"
    FILE "mobilenet-ssd_openvino_2021.4_5shave.blob"
    LOCATION mobilenet_5shaves_blob
)

# NeuralNetwork node, mobilenet example, 8 shaves
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_8shave.blob"
    SHA1 "7e7ea9a680668f3ca9e8f1d22cc8f3f1a02ac018"
    FILE "mobilenet-ssd_openvino_2021.4_8shave.blob"
    LOCATION mobilenet_8shaves_blob
)

# Video file with objects to detect
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/construction_vest.mp4"
    SHA1 "271d8d0b702e683ce02957db7c100843de5ceaec"
    FILE "construction_vest.mp4"
    LOCATION construction_vest
)

# Calibration Sample files
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/depthai_calib.json"
    SHA1 "ec50a47496473116c51cec1c8249bff2679383c3"
    FILE "depthai_calib.json"
    LOCATION calib_v6
)

hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/depthai_v5.calib"
    SHA1 "adfe9976b9aacfdaa76ac7138810999988177e71"
    FILE "depthai_v5.calib"
    LOCATION calib_v5
)

hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/misc/BW1098OBC.json"
    SHA1 "1e84b6c339ac51e3b254775090b91235a40fac71"
    FILE "BW1098OBC.json"
    LOCATION device_config
)

# person-detection
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/person-detection-retail-0013_openvino_2021.4_7shave.blob"
    SHA1 "946d102874b287292cdb5952fa66356c8d36025f"
    FILE "person-detection-retail-0013_openvino_2021.4_7shave.blob"
    LOCATION person_detection
)

# concat model
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/concat_openvino_2021.4_6shave.blob"
    SHA1 "e4af6fb2bed39648a2a4424d8bdc5bac56d616df"
    FILE "concat_openvino_2021.4_6shave.blob"
    LOCATION concat_model
)

# normalization model
hunter_private_data(
    URL "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/normalize_openvino_2021.4_4shave.blob"
    SHA1 "1435f6d11e0dfdbc60f2dfa45aecd3d530b6153b"
    FILE "normalize_openvino_2021.4_4shave.blob"
    LOCATION normalization_model
)

## message(STATUS "Location of test1.data: ${test1_data}")

# bootloader
dai_add_example(bootloader_version bootloader/bootloader_version.cpp ON)
dai_add_example(flash_bootloader bootloader/flash_bootloader.cpp OFF)
dai_add_example(flash_user_bootloader bootloader/flash_user_bootloader.cpp OFF)
dai_add_example(bootloader_config bootloader/bootloader_config.cpp ON read)
dai_add_example(poe_set_ip bootloader/poe_set_ip.cpp OFF)
dai_add_example(read_flash_memory bootloader/read_flash_memory.cpp OFF)
dai_add_example(write_flash_memory bootloader/write_flash_memory.cpp OFF)
dai_add_example(flash_boot_header bootloader/flash_boot_header.cpp OFF)

# calibration
dai_add_example(calibration_flash calibration/calibration_flash.cpp OFF)
dai_add_example(calibration_flash_version5 calibration/calibration_flash_v5.cpp OFF)
dai_add_example(calibration_load calibration/calibration_load.cpp ON)
dai_add_example(calibration_reader calibration/calibration_reader.cpp ON)
dai_add_example(calibration_dump calibration/calibration_dump.cpp ON)
dai_add_example(calibration_factory_reset calibration/calibration_factory_reset.cpp OFF)

target_compile_definitions(calibration_flash PRIVATE CALIB_PATH="${calib_v6}")
target_compile_definitions(calibration_flash_version5 PRIVATE CALIB_PATH="${calib_v5}" BOARD_PATH="${device_config}")
target_compile_definitions(calibration_load PRIVATE CALIB_PATH="${calib_v6}")

# ColorCamera
dai_add_example(rgb_camera_control ColorCamera/rgb_camera_control.cpp ON)
dai_add_example(rgb_preview ColorCamera/rgb_preview.cpp ON)
dai_add_example(rgb_video ColorCamera/rgb_video.cpp ON)
dai_add_example(rgb_isp_scale ColorCamera/rgb_isp_scale.cpp ON)

# EdgeDetector
dai_add_example(edge_detector EdgeDetector/edge_detector.cpp ON)

# FeatureTracker
dai_add_example(feature_detector FeatureTracker/feature_detector.cpp ON)
dai_add_example(feature_tracker FeatureTracker/feature_tracker.cpp ON)

# host_side
dai_add_example(device_queue_event host_side/device_queue_event.cpp ON)
dai_add_example(opencv_support host_side/opencv_support.cpp ON)
dai_add_example(queue_add_callback host_side/queue_add_callback.cpp ON)
dai_add_example(device_information host_side/device_information.cpp OFF)
dai_add_example(device_logging host_side/device_logging.cpp OFF)

# ImageManip
dai_add_example(image_manip ImageManip/image_manip_example.cpp ON)
dai_add_example(image_manip_rotate ImageManip/image_manip_rotate.cpp ON)
dai_add_example(image_manip_tiling ImageManip/image_manip_tiling.cpp ON)
dai_add_example(rgb_rotate_warp ImageManip/rgb_rotate_warp.cpp ON)
dai_add_example(image_manip_warp_mesh ImageManip/image_manip_warp_mesh.cpp ON)

# Warp
dai_add_example(warp_mesh Warp/warp_mesh.cpp ON)

# IMU
dai_add_example(imu_gyroscope_accelerometer IMU/imu_gyroscope_accelerometer.cpp ON)
dai_add_example(imu_rotation_vector IMU/imu_rotation_vector.cpp ON)

# mixed
dai_add_example(mono_depth_mobilenetssd mixed/mono_depth_mobilenetssd.cpp ON)
dai_add_example(rgb_encoding_mono_mobilenet mixed/rgb_encoding_mono_mobilenet.cpp ON)
dai_add_example(rgb_encoding_mono_mobilenet_depth mixed/rgb_encoding_mono_mobilenet_depth.cpp ON)
dai_add_example(rgb_encoding_mobilenet mixed/rgb_encoding_mobilenet.cpp ON)
dai_add_example(multiple_devices mixed/multiple_devices.cpp OFF)
dai_add_example(frame_sync mixed/frame_sync.cpp ON)

target_compile_definitions(mono_depth_mobilenetssd PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_encoding_mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_encoding_mono_mobilenet_depth PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_encoding_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")

# MobileNet
dai_add_example(camera_mobilenet_sync MobileNet/camera_mobilenet_sync_example.cpp ON)
dai_add_example(rgb_mobilenet MobileNet/rgb_mobilenet.cpp  ON)
dai_add_example(rgb_mobilenet_4k MobileNet/rgb_mobilenet_4k.cpp ON)
dai_add_example(mono_mobilenet MobileNet/mono_mobilenet.cpp  ON)
dai_add_example(video_mobilenet MobileNet/video_mobilenet.cpp ON)
# Usually not available - removed from tests
# TODO - add to a separate group if possible
dai_add_example(webcam_mobilenet MobileNet/webcam_mobilenet_example.cpp OFF)

target_compile_definitions(camera_mobilenet_sync PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(rgb_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}"  ON)
target_compile_definitions(rgb_mobilenet_4k PRIVATE BLOB_PATH="${mobilenet_5shaves_blob}")
target_compile_definitions(mono_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(video_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}" VIDEO_PATH="${construction_vest}")
target_compile_definitions(webcam_mobilenet PRIVATE BLOB_PATH="${mobilenet_8shaves_blob}")

# MonoCamera
dai_add_example(mono_camera_control MonoCamera/mono_camera_control.cpp ON)
dai_add_example(mono_preview MonoCamera/mono_preview.cpp  ON)
dai_add_example(mono_full_resolution_saver MonoCamera/mono_full_resolution_saver.cpp  ON)

# NeuralNetwork
dai_add_example(concatenate NeuralNetwork/concat_multi_input.cpp ON)
dai_add_example(normalize NeuralNetwork/normalization_multi_input.cpp ON)

target_compile_definitions(concatenate PRIVATE BLOB_PATH="${concat_model}")
target_compile_definitions(normalize PRIVATE BLOB_PATH="${normalization_model}")

# ObjectTracker
dai_add_example(object_tracker ObjectTracker/object_tracker.cpp ON)
dai_add_example(spatial_object_tracker ObjectTracker/spatial_object_tracker.cpp ON)
dai_add_example(object_tracker_video ObjectTracker/object_tracker_video.cpp ON)

target_compile_definitions(object_tracker PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(spatial_object_tracker PRIVATE BLOB_PATH="${mobilenet_5shaves_blob}")
target_compile_definitions(object_tracker_video PRIVATE BLOB_PATH="${person_detection}" VIDEO_PATH="${construction_vest}")

# Script
dai_add_example(script_camera_control Script/script_camera_control.cpp ON)
dai_add_example(script_forward_frames Script/script_forward_frames.cpp ON)
dai_add_example(script_get_ip Script/script_get_ip.cpp OFF)
dai_add_example(script_http_client Script/script_http_client.cpp OFF)
dai_add_example(script_http_server Script/script_http_server.cpp OFF)
dai_add_example(script_mjpeg_server Script/script_mjpeg_server.cpp OFF)
dai_add_example(script_nndata_example Script/script_nndata_example.cpp ON)
dai_add_example(script_json_communication Script/script_json_communication.cpp ON)
dai_add_example(script_change_pipeline_flow Script/script_change_pipeline_flow.cpp OFF)
target_compile_definitions(script_change_pipeline_flow PRIVATE BLOB_PATH="${mobilenet_5shaves_blob}")
dai_add_example(script_get_device_info Script/script_get_device_info.cpp ON)
dai_add_example(script_read_calibration Script/script_read_calibration.cpp ON)

# SpatialDetection
dai_add_example(spatial_location_calculator SpatialDetection/spatial_location_calculator.cpp ON)
dai_add_example(spatial_mobilenet_mono SpatialDetection/spatial_mobilenet_mono.cpp ON)
dai_add_example(spatial_mobilenet SpatialDetection/spatial_mobilenet.cpp ON)
dai_add_example(spatial_tiny_yolo_v3 SpatialDetection/spatial_tiny_yolo.cpp ON)
dai_add_example(spatial_tiny_yolo_v4 SpatialDetection/spatial_tiny_yolo.cpp ON)
dai_add_example(spatial_calculator_multi_roi SpatialDetection/spatial_calculator_multi_roi.cpp OFF)

target_compile_definitions(spatial_mobilenet_mono PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(spatial_mobilenet PRIVATE BLOB_PATH="${mobilenet_blob}")
target_compile_definitions(spatial_tiny_yolo_v3 PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")
target_compile_definitions(spatial_tiny_yolo_v4 PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")

# StereoDepth
dai_add_example(depth_crop_control StereoDepth/depth_crop_control.cpp ON)
dai_add_example(depth_preview StereoDepth/depth_preview.cpp  ON)
dai_add_example(depth_post_processing StereoDepth/depth_post_processing.cpp  ON)
dai_add_example(rgb_depth_aligned StereoDepth/rgb_depth_aligned.cpp ON)
dai_add_example(rgb_depth_confidence_aligned StereoDepth/rgb_depth_confidence_aligned.cpp ON)
dai_add_example(stereo_depth_video StereoDepth/stereo_depth_video.cpp ON)

# SystemLogger
dai_add_example(system_information SystemLogger/system_information.cpp ON)

# VideoEncoder
dai_add_example(disparity_encoding VideoEncoder/disparity_encoding.cpp  ON)
dai_add_example(rgb_encoding VideoEncoder/rgb_encoding.cpp  ON)
dai_add_example(rgb_full_resolution_saver VideoEncoder/rgb_full_resolution_saver.cpp  ON)
dai_add_example(rgb_mono_encoding VideoEncoder/rgb_mono_encoding.cpp  ON)
dai_add_example(encoding_max_limit VideoEncoder/encoding_max_limit.cpp ON)
dai_add_example(mjpeg_encoding VideoEncoder/mjpeg_encoding_example.cpp ON)

# Yolo
dai_add_example(tiny_yolo_v4_device_side_decoding Yolo/tiny_yolo.cpp ON)
dai_add_example(tiny_yolo_v3_device_side_decoding Yolo/tiny_yolo.cpp  ON)
target_compile_definitions(tiny_yolo_v4_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v4_blob}")
target_compile_definitions(tiny_yolo_v3_device_side_decoding PRIVATE BLOB_PATH="${tiny_yolo_v3_blob}")
dai_add_example(apriltag AprilTag/apriltag.cpp ON)
dai_add_example(apriltag_rgb AprilTag/apriltag_rgb.cpp ON)

# DetectionParser
dai_add_example(detection_parser NeuralNetwork/detection_parser.cpp ON)
target_compile_definitions(detection_parser PRIVATE BLOB_PATH="${mobilenet_blob}")

# DetectionParser
dai_add_example(crash_report CrashReport/crash_report.cpp OFF)
