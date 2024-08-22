## setup compilation flags
include(CMakeParseArguments)
include(CheckCXXCompilerFlag)

# conditionally applies flag. If flag is supported by current compiler, it will be added to compile options.
function(add_flag target flag)
    check_cxx_compiler_flag(${flag} FLAG_${flag})
    if (FLAG_${flag} EQUAL 1)
        target_compile_options(${target} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:${flag}>)
    endif ()
endfunction()

function(add_default_flags target)
    cmake_parse_arguments(ADF "LEAN" "" "" ${ARGN})
    if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "^(AppleClang|Clang|GNU)$")
        if(NOT ADF_LEAN)
            # enable those flags
            add_flag(${target} -Wall)
            add_flag(${target} -Wextra)
            add_flag(${target} -Woverloaded-virtual)     # warn if you overload (not override) a virtual function
            add_flag(${target} -Wformat=2)               # warn on security issues around functions that format output (ie printf)
            add_flag(${target} -Wmisleading-indentation) # (only in GCC >= 6.0) warn if indentation implies blocks where blocks do not exist
            add_flag(${target} -Wduplicated-cond)        # (only in GCC >= 6.0) warn if if / else chain has duplicated conditions
            add_flag(${target} -Wduplicated-branches)    # (only in GCC >= 7.0) warn if if / else branches have duplicated code
            add_flag(${target} -Wnull-dereference)       # (only in GCC >= 6.0) warn if a null dereference is detected
            add_flag(${target} -Wdouble-promotion)       # (GCC >= 4.6, Clang >= 3.8) warn if float is implicit promoted to double
            add_flag(${target} -Wsign-compare)
            add_flag(${target} -Wtype-limits)            # size_t - size_t >= 0 -> always true

            # disable those flags
            # add_flag(${target} -Wno-unused-command-line-argument)    # clang: warning: argument unused during compilation: '--coverage' [-Wunused-command-line-argument]
            # add_flag(${target} -Wno-unused-parameter)    # prints too many useless warnings
            # add_flag(${target} -Wno-format-nonliteral)   # prints way too many warnings from spdlog
            # add_flag(${target} -Wno-gnu-zero-variadic-macro-arguments)   # https://stackoverflow.com/questions/21266380/is-the-gnu-zero-variadic-macro-arguments-safe-to-ignore

            # promote to errors
            add_flag(${target} -Werror=self-assign-field)  # error if self assign - bugprone
            add_flag(${target} -Werror=unused-lambda-capture)  # error if lambda capture is unused
            add_flag(${target} -Werror=return-type)      # warning: control reaches end of non-void function [-Wreturn-type]
            add_flag(${target} -Werror=non-virtual-dtor) # warn the user if a class with virtual functions has a non-virtual destructor. This helps catch hard to track down memory errors
            add_flag(${target} -Werror=sign-compare)     # warn the user if they compare a signed and unsigned numbers
            add_flag(${target} -Werror=reorder)          # field '$1' will be initialized after field '$2'
            add_flag(${target} -Werror=switch-enum)      # if switch case is missing - error
        endif()

    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
        # using Visual Studio C++
        add_flag(${target} -DNOMINMAX)
        add_flag(${target} -DWIN32_LEAN_AND_MEAN)

        if(NOT ADF_LEAN)
            # TODO(warchant): add flags https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md#msvc
        endif()

        # disable INTERPROCEDURAL_OPTIMIZATION and enable /LTCG for issue https://github.com/luxonis/depthai-core/issues/334
        if(WIN32)
            get_target_property(_EXPORT ${target} WINDOWS_EXPORT_ALL_SYMBOLS)
            if(_EXPORT)
                if(CMAKE_BUILD_TYPE)
                    string(TOUPPER "_${CMAKE_BUILD_TYPE}" _BUILD_TYPE)
                else()
                    set(_BUILD_TYPE "")
                endif()
                set(_PROP_NAME INTERPROCEDURAL_OPTIMIZATION${_BUILD_TYPE})
                get_property(_PROP_SET TARGET ${target} PROPERTY ${_PROP_NAME} SET)
                if(NOT _PROP_SET)
                    set(_PROP_NAME INTERPROCEDURAL_OPTIMIZATION)
                endif()
                get_target_property(_INTER_OPT ${target} ${_PROP_NAME})
                if(_INTER_OPT)
                    set_target_properties(${target} PROPERTIES ${_PROP_NAME} OFF)
                    # check_linker_flag() only available in cmake 3.18+
                    target_link_options(${target} PRIVATE /LTCG)
                endif()
                unset(_BUILD_TYPE)
                unset(_INTER_OPT)
                unset(_PROP_NAME)
                unset(_PROP_SET)
            endif()
            unset(_EXPORT)
        endif()
    endif()

endfunction()

# discover compiler C++ standard support
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    check_cxx_compiler_flag("-std:c++14" COMPILER_SUPPORTS_CXX14)
    check_cxx_compiler_flag("-std:c++17" COMPILER_SUPPORTS_CXX17)
    check_cxx_compiler_flag("-std:c++20" COMPILER_SUPPORTS_CXX20)
    check_cxx_compiler_flag("-std:c++23" COMPILER_SUPPORTS_CXX23)
    if(COMPILER_SUPPORTS_CXX14)
        set(CMAKE_CXX14_STANDARD_COMPILE_OPTION "-std:c++14")
        set(CMAKE_CXX14_EXTENSION_COMPILE_OPTION "-std:c++14")
    endif()
    if(COMPILER_SUPPORTS_CXX17)
        set(CMAKE_CXX17_STANDARD_COMPILE_OPTION "-std:c++17")
        set(CMAKE_CXX17_EXTENSION_COMPILE_OPTION "-std:c++17")
    endif()
    if(COMPILER_SUPPORTS_CXX20)
        set(CMAKE_CXX20_STANDARD_COMPILE_OPTION "-std:c++20")
        set(CMAKE_CXX20_EXTENSION_COMPILE_OPTION "-std:c++20")
    endif()
    if(COMPILER_SUPPORTS_CXX23)
        set(CMAKE_CXX23_STANDARD_COMPILE_OPTION "-std:c++23")
        set(CMAKE_CXX23_EXTENSION_COMPILE_OPTION "-std:c++23")
    endif()
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "^(AppleClang|Clang|GNU)$")
    check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
    check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
    check_cxx_compiler_flag("-std=c++20" COMPILER_SUPPORTS_CXX20)
    check_cxx_compiler_flag("-std=c++23" COMPILER_SUPPORTS_CXX23)
    if(COMPILER_SUPPORTS_CXX14)
        set(CMAKE_CXX14_STANDARD_COMPILE_OPTION "-std=c++14")
        set(CMAKE_CXX14_EXTENSION_COMPILE_OPTION "-std=gnu++14")
    endif()
    if(COMPILER_SUPPORTS_CXX17)
        set(CMAKE_CXX17_STANDARD_COMPILE_OPTION "-std=c++17")
        set(CMAKE_CXX17_EXTENSION_COMPILE_OPTION "-std=gnu++17")
    endif()
    if(COMPILER_SUPPORTS_CXX20)
        set(CMAKE_CXX20_STANDARD_COMPILE_OPTION "-std=c++20")
        set(CMAKE_CXX20_EXTENSION_COMPILE_OPTION "-std=gnu++20")
    endif()
    if(COMPILER_SUPPORTS_CXX23)
        set(CMAKE_CXX23_STANDARD_COMPILE_OPTION "-std=c++23")
        set(CMAKE_CXX23_EXTENSION_COMPILE_OPTION "-std=gnu++23")
    endif()
endif()
