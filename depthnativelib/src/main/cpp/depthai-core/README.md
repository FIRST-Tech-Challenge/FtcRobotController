# DepthAI C++ Library

[![Forum](https://img.shields.io/badge/Forum-discuss-orange)](https://discuss.luxonis.com/)
[![Docs](https://img.shields.io/badge/Docs-DepthAI_API-yellow)](https://docs.luxonis.com/projects/api)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

Core C++ library

## Documentation

Documentation is available over at [Luxonis DepthAI API](https://docs.luxonis.com/projects/api/en/latest/)

## Disclaimer
DepthAI library doesn't yet provide API stability guarantees. While we take care to properly deprecate old functions, some changes might still be breaking. We expect to provide API stability from version 3.0.0 onwards.

## Dependencies
- CMake >= 3.10
- C/C++14 compiler
- [optional] OpenCV 4 (required if building examples)

MacOS: Optional `brew install opencv`

Linux: Optional `sudo apt install libopencv-dev`

## Building

Make sure submodules are updated
```
git submodule update --init --recursive
```

Then configure and build

```
cmake -S. -Bbuild
cmake --build build
```

> ℹ️ To speed up build times, use `cmake --build build --parallel [num CPU cores]` (CMake >= 3.12).
For older versions use: Linux/macOS: `cmake --build build -- -j[num CPU cores]`, MSVC: `cmake --build build -- /MP[num CPU cores]`

> ⚠️ If any CMake commands error with `CMake Error: The source directory "" does not exist.` replace argument `-S` with `-H`

### Dynamic library

To build dynamic version of library configure with following option added
```
cmake -S. -Bbuild -D'BUILD_SHARED_LIBS=ON'
cmake --build build
```


### Android

Android is supported to some extent but not actively pursued nor tested. PRs with any improvements are welcome.

Steps:

 - Install Android NDK (for example via Android Studio).
 - Set the NDK path:
```
export ANDROID_HOME=$HOME/.local/lib/Android
export PATH=$PATH:$ANDROID_HOME/emulator:$ANDROID_HOME/platform-tools
export NDK=$ANDROID_HOME/ndk/23.1.7779620/ # Check version
```
 - Ensure a recent version of cmake (apt version is outdated, install snap install cmake --classic)
 - Run cmake, set your ABI and Platform as needed:

```
cmake -S. -Bbuild -DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake -DANDROID_ABI=armeabi-v7a -DANDROID_PLATFORM=android-25
cmake --build build
```


## Running examples

To build the examples configure with following option added
```
cmake -S. -Bbuild -D'DEPTHAI_BUILD_EXAMPLES=ON'
cmake --build build
```

Then navigate to `build/examples` folder and run a preferred example
```
cd build/examples
./MobileNet/rgb_mobilenet
```

> ℹ️ Multi-Config generators (like Visual Studio on Windows) will have the examples built in `build/examples/MobileNet/[Debug/Release/...]/rgb_mobilenet`

## Integration

Under releases you may find prebuilt library for Windows, for use in either integration method. See [Releases](https://github.com/luxonis/depthai-core/releases)

### CMake

Targets available to link to are:
 - depthai::core - Core library, without using opencv internally
 - depthai::opencv - Core + support for opencv related helper functions (requires OpenCV4)

#### Using find_package

Build static or dynamic version of library (See: [Building](#building) and optionally [Installing](#installing))

Add `find_package` and `target_link_libraries` to your project
```
find_package(depthai CONFIG REQUIRED)
...
target_link_libraries([my-app] PRIVATE depthai::opencv)
```

And point CMake to either build directory or install directory:
```
-D'depthai_DIR=depthai-core/build'
```
or
```
-D'depthai_DIR=depthai-core/build/install/lib/cmake/depthai'
```

If library was installed to default search path like `/usr/local` on Linux, specifying `depthai_DIR` isn't necessary as CMake will find it automatically.

#### Using add_subdirectory

This method is more intrusive but simpler as it doesn't require building the library separately.

Add `add_subdirectory` which points to `depthai-core` folder **before** project command. Then link to any required targets.
```
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)
...
project(my-app)
...
target_link_libraries([my-app] PRIVATE depthai::opencv)
```

### Non-CMake integration (Visual Studio, Xcode, CodeBlocks, ...)

To integrate into a different build system than CMake, prefered way is compiling as dynamic library and setting correct build options.
1. First build as dynamic library: [Building Dynamic library](#dynamic-library)
2. Then install: [Installing](#installing)

In your non-CMake project (new Visual Studio project, ...)
1. Set needed library directories:
    - `build/install/lib` (for linking to either depthai-core or depthai-opencv)
    - `build/install/bin` (for .dll's)
2. And include directories
    - `build/install/include` (library headers)
    - `build/install/include/depthai-shared/3rdparty` (shared 3rdparty headers)
    - `build/install/lib/cmake/depthai/dependencies/include` (dependency headers)

> ℹ️ Threading library might need to be linked to explicitly.

> ℹ️ Check `build/depthai-core-integration.txt` or `build/depthai-opencv-integration.txt` for up to date define options.
The generated integration file also specifies include paths without requiring installation.

## Installing

To install specify optional prefix and build target install
```
cmake -S. -Bbuild -D'CMAKE_INSTALL_PREFIX=[path/to/install/dir]'
cmake --build build --target install
```

If `CMAKE_INSTALL_PREFIX` isn't specified, the library is installed under build folder `install`.


## Environment variables

The following environment variables can be set to alter default behavior of the library without having to recompile

| Environment variable  | Description   |
|--------------|-----------|
| DEPTHAI_LEVEL | Sets logging verbosity, 'trace', 'debug', 'warn', 'error' and 'off' |
| XLINK_LEVEL | Sets logging verbosity of XLink library, 'debug'. 'info', 'warn', 'error', 'fatal' and 'off' |
| DEPTHAI_INSTALL_SIGNAL_HANDLER | Set to 0 to disable installing Backward signal handler for stack trace printing |
| DEPTHAI_WATCHDOG | Sets device watchdog timeout. Useful for debugging (`DEPTHAI_WATCHDOG=0`), to prevent device reset while the process is paused. |
| DEPTHAI_WATCHDOG_INITIAL_DELAY | Specifies delay after which the device watchdog starts. |
| DEPTHAI_SEARCH_TIMEOUT | Specifies timeout in milliseconds for device searching in blocking functions. |
| DEPTHAI_CONNECT_TIMEOUT | Specifies timeout in milliseconds for establishing a connection to a given device. |
| DEPTHAI_BOOTUP_TIMEOUT | Specifies timeout in milliseconds for waiting the device to boot after sending the binary. |
| DEPTHAI_PROTOCOL | Restricts default search to the specified protocol. Options: any, usb, tcpip. |
| DEPTHAI_DEVICE_MXID_LIST | Restricts default search to the specified MXIDs. Accepts comma separated list of MXIDs. Lists filter results in an "AND" manner and not "OR" |
| DEPTHAI_DEVICE_ID_LIST | Alias to MXID list. Lists filter results in an "AND" manner and not "OR" |
| DEPTHAI_DEVICE_NAME_LIST | Restricts default search to the specified NAMEs. Accepts comma separated list of NAMEs. Lists filter results in an "AND" manner and not "OR" |
| DEPTHAI_DEVICE_BINARY | Overrides device Firmware binary. Mostly for internal debugging purposes. |
| DEPTHAI_BOOTLOADER_BINARY_USB | Overrides device USB Bootloader binary. Mostly for internal debugging purposes. |
| DEPTHAI_BOOTLOADER_BINARY_ETH | Overrides device Network Bootloader binary. Mostly for internal debugging purposes. |
| DEPTHAI_ALLOW_FACTORY_FLASHING | Internal use only |

## Running tests

To run the tests build the library with the following options
```
cmake -S. -Bbuild -D'DEPTHAI_TEST_EXAMPLES=ON' -D'DEPTHAI_BUILD_TESTS=ON' -D'DEPTHAI_BUILD_EXAMPLES=ON'
cmake --build build
```

Then navigate to `build` folder and run `ctest` with specified labels that denote device type to test on.
Currently available labels:
 - usb
 - poe

```
cd build
# Run tests on USB devices
ctest -L usb
# Run tests on PoE devices
ctest -L poe
```

## Style check

The library uses clang format to enforce a certain coding style.
If a style check is failing, run the `clangformat` target, check the output and push changes.

To use this target clang format must be installed, preferably clang-format-10
```
sudo apt install clang-format-10
```

And to apply formatting
```
cmake --build build --target clangformat
```

## Documentation generation

Doxygen is used to generate documentation. Follow [doxygen download](https://www.doxygen.nl/download.html#srcbin) and install the required binaries for your platform.

After that specify CMake define `-D'DEPTHAI_BUILD_DOCS=ON`' and build the target `doxygen`

## Debugging tips

Debugging can be done using **Visual Studio Code** and either **GDB** or **LLDB** (extension 'CodeLLDB').
LLDB in some cases was much faster to step with and resolved more `incomplete_type` variables than GDB. Your mileage may vary though.


If there is a need to step into **Hunter** libraries, that can be achieved by removing previous built artifacts
```
rm -r ~/.hunter
```

And configuring the project with the following CMake option set to `ON`
```
cmake . -D'HUNTER_KEEP_PACKAGE_SOURCES=ON'
```

This retains the libraries source code, so that debugger can step through it (the paths are already set up correctly)


## Troubleshooting

### Build fails with missing OpenCV dependency

If your build process happen to fail due to OpenCV library not being found, but you have the OpenCV installed, please
run build with additional `-D'OpenCV_DIR=...`' flag (replacing default Ubuntu path `/usr/lib/x86_64-linux-gnu/cmake/opencv4` with yours)

```
cmake -S. -Bbuild -D'OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4'
```

Now the build process should correctly discover your OpenCV installation

### Hunter
Hunter is a CMake-only dependency manager for C/C++ projects.

If you are stuck with error message which mentions external libraries (subdirectory of `.hunter`) like the following:
```
/usr/bin/ld: /home/[user]/.hunter/_Base/062a19a/ccfed35/a84a713/Install/lib/liblzma.a(stream_flags_decoder.c.o): warning: relocation against `lzma_footer_magic' in read-only section `.text'
```

Try erasing the **Hunter** cache folder.

Linux/MacOS:
```
rm -r ~/.hunter
```
Windows:
```
del C:/.hunter
```
or
```
del C:/[user]/.hunter
```
