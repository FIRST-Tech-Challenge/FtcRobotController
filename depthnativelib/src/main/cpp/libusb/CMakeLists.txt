cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

file(READ libusb/version.h LU_VERSION_H)

function(LU_DEFINE_VERSION_ATOM var_name)
    if(LU_VERSION_H MATCHES "#define[ \\t]+${var_name}[ \\t]([0-9]+)")
        set("${var_name}" "${CMAKE_MATCH_1}" PARENT_SCOPE)
    endif()
endfunction()

function(LU_DEFINE_VERSION_RC_ATOM var_name)
    if(LU_VERSION_H MATCHES "#define[ \\t]+${var_name}[ \\t]\"(-rc[0-9]*)\"")
        set("${var_name}" "${CMAKE_MATCH_1}" PARENT_SCOPE)
    endif()
endfunction()

LU_DEFINE_VERSION_ATOM(LIBUSB_MAJOR)
LU_DEFINE_VERSION_ATOM(LIBUSB_MINOR)
LU_DEFINE_VERSION_ATOM(LIBUSB_MICRO)
LU_DEFINE_VERSION_RC_ATOM(LIBUSB_RC)

project(
    libusb
    VERSION "${LIBUSB_MAJOR}.${LIBUSB_MINOR}.${LIBUSB_MICRO}${LIBUSB_RC}"
    LANGUAGES C CXX
)

# Creates library target "usb-1.0"
set(TARGET_NAME usb-1.0)
add_library(${TARGET_NAME})
get_cmake_property(is_multi_config GENERATOR_IS_MULTI_CONFIG)
if(is_multi_config)
    set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "d")
endif()

# Add PUBLIC include directory
target_include_directories(${TARGET_NAME} PUBLIC
    # Relative path to include directories after installed
    "$<INSTALL_INTERFACE:include>"
    # Build time
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/libusb>"
)

# Create a list of sources
# Common sources
set(SOURCE_LIST "libusb/core.c" "libusb/descriptor.c" "libusb/hotplug.c" "libusb/io.c" "libusb/strerror.c" "libusb/sync.c")

# Add target properties
target_include_directories(${TARGET_NAME} PRIVATE libusb libusb/os ${CMAKE_CURRENT_BINARY_DIR})
# Add definitions - HAVE_CONFIG_H
target_compile_definitions(${TARGET_NAME} PRIVATE HAVE_CONFIG_H)

# Host system checks
include(CheckCSourceCompiles)
include(CheckFunctionExists)
include(CheckLibraryExists)
include(CheckSymbolExists)
include(CheckIncludeFile)

# Platform and backend detection
if(APPLE)
    set(backend darwin)
    set(platform posix)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Haiku")
    set(backend haiku)
    set(platform posix)
elseif(ANDROID)
    set(backend linux)
    set(platform posix)
    set(is_android_linux ON)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(backend linux)
    set(platform posix)
elseif(CYGWIN)
    set(backend windows)
    set(platform windows)
    set(EXTRA_CFLAGS ${EXTRA_CFLAGS} "-mwin32")  # is this necessary?
elseif(MINGW)
    set(backend windows)
    set(platform windows)
    set(EXTRA_CFLAGS ${EXTRA_CFLAGS} "-mwin32 -fno-omit-frame-pointer")  # is this necessary?
elseif(WIN32)
    set(backend windows)
    set(platform windows)
elseif(CMAKE_SYSTEM_NAME STREQUAL "NetBSD")
    set(backend netbsd)
    set(platform posix)
elseif(CMAKE_SYSTEM_NAME STREQUAL "OpenBSD")
    set(backend openbsd)
    set(platform posix)
elseif(CMAKE_SYSTEM_NAME STREQUAL "SunOS")
    set(backend sunos)
    set(platform posix)
else()
    message(WARNING "The host being compiled for is not supported.")
    message(WARNING "The library may compile but will not function in any useful manner.")
    set(backend null)
    set(platform posix)
endif()

# Settings based on platform and backend
if(platform STREQUAL "posix")
    set(PLATFORM_POSIX 1)

    # Set C standard to C11 and _GNU_SOURCE definition
    set_property(TARGET ${TARGET_NAME} PROPERTY C_STANDARD 11)
    target_compile_definitions(${TARGET_NAME} PRIVATE _GNU_SOURCE)

    check_c_source_compiles("#include <poll.h>\nint main(void) { nfds_t foo; return 0; }" HAVE_NFDS_T)
    check_function_exists(pipe2 HAVE_PIPE2)

    check_function_exists(clock_gettime HAVE_CLOCK_GETTIME)
    if(NOT HAVE_CLOCK_GETTIME)
        check_library_exists(rt clock_gettime "" HAVE_CLOCK_GETTIME)
        if(HAVE_CLOCK_GETTIME)
            target_link_libraries(${TARGET_NAME} PRIVATE rt)
        endif()
    endif()

    check_symbol_exists("CLOCK_MONOTONIC" "time.h" HAVE_CLOCK_MONOTONIC)
    check_symbol_exists("pthread_condattr_setclock" "pthread.h" HAVE_PTHREAD_CONDATTR_SETCLOCK)

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/events_posix.c libusb/os/threads_posix.c)

elseif(platform STREQUAL "windows")
    set(PLATFORM_WINDOWS 1)

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/events_windows.c libusb/os/threads_windows.c $<$<CONFIG:DEBUG>:libusb/libusb-1.0d.def> $<$<CONFIG:Release>:libusb/libusb-1.0.def>)
    set_target_properties(${TARGET_NAME} PROPERTIES PREFIX "lib")

else()
    message(FATAL_ERROR "Unknown platform ${platform}")
endif()

if(backend STREQUAL "darwin")
    check_function_exists(pthread_threadid_np HAVE_PTHREAD_THREADID_NP)

    find_library(IOKIT_FRAMEWORK IOKit REQUIRED)
    target_link_libraries(${TARGET_NAME} PRIVATE "${IOKIT_FRAMEWORK}")

    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation REQUIRED)
    target_link_libraries(${TARGET_NAME} PRIVATE "${COREFOUNDATION_FRAMEWORK}")

    find_library(SECURITY_FRAMEWORK Security REQUIRED)
    target_link_libraries(${TARGET_NAME} PRIVATE "${SECURITY_FRAMEWORK}")

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/darwin_usb.c)

elseif(backend STREQUAL "haiku")
    target_link_libraries(${TARGET_NAME} PRIVATE be)

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/haiku_usb_backend.cpp libusb/os/haiku_pollfs.cpp libusb/os/haiku_usb_raw.cpp)

elseif(backend STREQUAL "linux")

    check_function_exists(pthread_setname_np HAVE_PTHREAD_SETNAME_NP)

    # Compile with udev support
    if(NOT is_android_linux)
        option(WITH_UDEV "use udev for device enumeration and hotplug support (recommended)" ON)
        if(WITH_UDEV)
            CHECK_INCLUDE_FILE("libudev.h" HAVE_LIB_UDEV_H)
            if(NOT HAVE_LIB_UDEV_H)
                message(FATAL_ERROR "udev support requested but libudev header not installed")
            endif()

            find_library(LIBUDEV_LIBRARY NAMES udev)
            check_library_exists(udev udev_new ${LIBUDEV_LIBRARY} HAVE_LIBUDEV)
            if(NOT HAVE_LIBUDEV)
                message(FATAL_ERROR "udev support requested but libudev not installed")
            endif()

            # Set source files
            set(SOURCE_LIST ${SOURCE_LIST} libusb/os/linux_udev.c)
            # Link to udev
            target_link_libraries(${TARGET_NAME} PRIVATE udev)

        endif()
    endif()

    if(NOT WITH_UDEV)
        CHECK_INCLUDE_FILE("asm/types.h" HAVE_ASM_TYPES_H)
        CHECK_INCLUDE_FILE("linux/netlink.h" HAVE_LINUX_NETLINK_H)
        CHECK_INCLUDE_FILE("sys/socket.h" HAVE_SYS_SOCKET_H)
        # Set source files
        set(SOURCE_LIST ${SOURCE_LIST} libusb/os/linux_netlink.c)
        set(HAVE_LIBUDEV)
        set(HAVE_LIB_UDEV_H)
    endif()

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/linux_usbfs.c)

    # Android support via JNI
    if(is_android_linux)
        # Add additional sources
        set(SOURCE_LIST ${SOURCE_LIST} libusb/os/linux_android_jni.c)
        # Links to Android logging library
        target_link_libraries(${TARGET_NAME} PRIVATE log)
    endif()

elseif(backend STREQUAL "netbsd")

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/netbsd_usb.c)

elseif(backend STREQUAL "openbsd")

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/openbsd_usb.c)

elseif(backend STREQUAL "sunos")
    target_link_libraries(${TARGET_NAME} PRIVATE devinfo)

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/sunos_usb.c)

elseif(backend STREQUAL "windows")

    CHECK_INCLUDE_FILE("string.h" HAVE_STRING_H)

    check_c_source_compiles("#include <time.h>\nint main(void) { struct timespec foo; return 0; }" HAVE_STRUCT_TIMESPEC)
    # Define oldest supported version
    target_compile_definitions(${TARGET_NAME} PRIVATE _WIN32_WINNT=_WIN32_WINNT_VISTA)
    # Set UNICODE API usage for Win
    target_compile_definitions(${TARGET_NAME} PRIVATE UNICODE _UNICODE)

    # Set source files
    set(SOURCE_LIST ${SOURCE_LIST} libusb/os/windows_common.c libusb/os/windows_usbdk.c libusb/os/windows_winusb.c)

endif()

# Headers not available on all platforms but required on others
check_c_source_compiles("#include <sys/time.h>\nint main() { return 0; }" HAVE_SYS_TIME_H)

# Eventfd support
if(${backend} STREQUAL "linux" OR ${backend} STREQUAL "sunos")

    # Add option to enable or disable eventfd
    option(WITH_EVENTFD "Use eventfd for signaling" ON)

    if(WITH_EVENTFD)
        CHECK_INCLUDE_FILE("sys/eventfd.h" eventfd_h)
        if(eventfd_h)
            check_symbol_exists("EFD_NONBLOCK" "sys/eventfd.h" HAVE_DECL_EFD_NONBLOCK)
            check_symbol_exists("EFD_CLOEXEC" "sys/eventfd.h" HAVE_DECL_EFD_CLOEXEC)
            if(HAVE_DECL_EFD_NONBLOCK AND HAVE_DECL_EFD_CLOEXEC)
                check_function_exists(eventfd eventfd_ok)
                if(eventfd_ok)
                    set(HAVE_EVENTFD 1)
                endif()
            endif()
        endif()
        # Final check
        if(NOT HAVE_EVENTFD)
            message(FATAL_ERROR "eventfd() function not found. glibc 2.9+ required")
        endif()
    endif()
endif()

# timerfd support
if(${backend} STREQUAL "linux" OR ${backend} STREQUAL "sunos")

    # Add option to enable or disable eventfd
    option(WITH_TIMERFD "Use timerfd for timing" ON)

    if(WITH_TIMERFD)
        check_include_file("sys/timerfd.h" timerfd_h)
        if(timerfd_h)
            check_symbol_exists("TFD_NONBLOCK" "sys/timerfd.h" HAVE_DECL_TFD_NONBLOCK)
            check_symbol_exists("TFD_CLOEXEC" "sys/timerfd.h" HAVE_DECL_TFD_CLOEXEC)
            if(HAVE_DECL_TFD_NONBLOCK AND HAVE_DECL_TFD_CLOEXEC)
                check_function_exists(timerfd_create timerfd_ok)
                if(timerfd_ok)
                    set(HAVE_TIMERFD 1)
                endif()
            endif()
        endif()
        # Final check
        if(NOT HAVE_TIMERFD)
            message(FATAL_ERROR "timerfd_create() function not found. glibc 2.9+ required")
        endif()
    endif()
endif()


# Message logging
option(WITH_LOG "Enable logging" ON)
if(WITH_LOG)
    set(ENABLE_LOGGING 1)
endif()

option(WITH_DEBUG_LOG "Starts with debug message logging enabled" OFF)
if(WITH_DEBUG_LOG)
    set(ENABLE_DEBUG_LOGGING 1)
endif()

option(WITH_SYSTEM_LOG "Output logging messages to the system-wide log, if supported by the OS" OFF)
if(WITH_SYSTEM_LOG)
    # TODO
    message(FATAL_ERROR "WITH_SYSTEM_LOG not implemented yet")
endif()

# Examples build
option(BUILD_EXAMPLES "Build example applications" OFF)
if(BUILD_EXAMPLES)
    # TODO
    message(FATAL_ERROR "BUILD_EXAMPLES not implemented yet")
endif()

# Tests build
option(BUILD_TESTS "Build test applications" OFF)
if(BUILD_TESTS)
    # TODO
    message(FATAL_ERROR "BUILD_TESTS not implemented yet")
endif()

# Finally copy a config.h, or configure from settings above
if(backend STREQUAL "darwin")
    configure_file(Xcode/config.h config.h COPYONLY)
elseif(backend STREQUAL "windows")
    if(MINGW)
        configure_file(cmake/template_config.h config.h @ONLY)
    else()
        configure_file(msvc/config.h config.h COPYONLY)
    endif()
elseif(is_android_linux)
    configure_file(android/config.h config.h COPYONLY)
else()
    # Otherwise use template_config and configure it
    configure_file(cmake/template_config.h config.h @ONLY)
endif()

########################################
# Add sources to target
########################################
target_sources(${TARGET_NAME} PRIVATE ${SOURCE_LIST})

########################################
# Installation steps
########################################
include(GNUInstallDirs)
install(TARGETS ${TARGET_NAME}
    EXPORT "${TARGET_NAME}Config"
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES libusb/libusb.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/libusb-1.0/)

set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET_NAME}")
export(TARGETS
        ${TARGET_NAME}
    FILE "${config_install_dir}/${TARGET_NAME}Config.cmake"
)
install(EXPORT
    "${TARGET_NAME}Config"
    DESTINATION "${config_install_dir}"
)
