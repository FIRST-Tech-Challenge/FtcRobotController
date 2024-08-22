cmake_minimum_required(VERSION 3.18.1)

set(CMAKE_DEBUG_POSTFIX "")

set(BUILD_SHARED_LIBS ON)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)
project("depthaiandroid")

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/libusb)

add_library(depthaiandroid SHARED native-lib.cpp)

find_library(log-lib log)

target_link_libraries(depthaiandroid
                        depthai::core
                        usb-1.0
                        ${log-lib})
