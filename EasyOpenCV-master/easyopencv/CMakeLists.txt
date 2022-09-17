# Sets the minimum version of CMake required to build your native library.
# This ensures that a certain set of CMake features is available to
# your build.

cmake_minimum_required(VERSION 3.4.1)

set (CMAKE_CXX_STANDARD 11)

# Specifies a library name, specifies whether the library is STATIC or
# SHARED, and provides relative paths to the source code. You can
# define multiple libraries by adding multiple add_library() commands,
# and CMake builds them for you. When you build your app, Gradle
# automatically packages shared libraries with your APK.

add_library( # Specifies the name of the library.
             EasyOpenCV

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             src/main/cpp/webcam_jni.cpp
             src/main/cpp/camera2_jni.cpp
             src/main/cpp/camera_base_jni.cpp
             src/main/cpp/vuforia_passthrough_jni.cpp)

#########################################################
# You must clone OpenCv-Repackaged into the same parent #
# directory as this copy of EasyOpenCV in order to be   #
# able to compile the native code. (Or, you can edit    #
# these paths)                                          #
#########################################################

# Specifies a path to native header files.
include_directories(src/main/cpp/include/
                    ${CMAKE_CURRENT_SOURCE_DIR}/../../OpenCV-Repackaged/doc/native_headers/)

# Specify path to prebuilt object

set(OPENCV_NATIVE_LIB_PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../OpenCV-Repackaged/doc/native_libs)

add_library(openCvNative SHARED IMPORTED)
set_target_properties(openCvNative PROPERTIES IMPORTED_LOCATION
        ${OPENCV_NATIVE_LIB_PARENT_DIR}/${ANDROID_ABI}/libOpenCvAndroid453.so)

# linking libs
target_link_libraries(
        EasyOpenCV
        openCvNative
        log
        android)