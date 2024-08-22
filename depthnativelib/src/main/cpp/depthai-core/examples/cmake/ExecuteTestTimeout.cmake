cmake_minimum_required(VERSION 3.2)

# Parse out arguments
foreach(_arg RANGE ${CMAKE_ARGC})
    if(append)
        list(APPEND arguments "${CMAKE_ARGV${_arg}}")
    endif()

    if("${CMAKE_ARGV${_arg}}" STREQUAL "-P")
        set(append true)
        message(STATUS "found -P")
    endif()
endforeach()
list(REMOVE_AT arguments 0)

message(STATUS "arguments: ${arguments}")

# Check if ENV variable TEST_TIMEOUT is set and use that rather than TIMEOUT_SECONDS
if(DEFINED ENV{TEST_TIMEOUT})
    message(STATUS "Overriding timeout: ${TIMEOUT_SECONDS} with $ENV{TEST_TIMEOUT}")
    set(TIMEOUT_SECONDS $ENV{TEST_TIMEOUT})
endif()

# Execute the example (SIGTERM for now, could be improved with SIGINT -> SIGKILL)
if(TIMEOUT_SECONDS GREATER 0)
    execute_process(COMMAND ${arguments} TIMEOUT ${TIMEOUT_SECONDS} RESULT_VARIABLE error_variable)
else()
    execute_process(COMMAND ${arguments} RESULT_VARIABLE error_variable)
endif()

message(STATUS "After process executed, ${PATH_TO_TEST_EXECUTABLE} produced the following exit code: ${error_variable}")

if(error_variable MATCHES "timeout" OR error_variable EQUAL 128 OR error_variable EQUAL 124)
    # Okay
elseif(NOT error_variable)
    # return code == 0, also okay
elseif(error_variable EQUAL 133 OR error_variable MATCHES "Child killed")
    # sigkill, return fatal error but mark the issue
    message(FATAL_ERROR "${PATH_TO_TEST_EXECUTABLE} had to be forcefully killed after 5 additional seconds after SIGINT")
else()
    # not timeout and error code != 0, not okay
    message(FATAL_ERROR "${PATH_TO_TEST_EXECUTABLE} produced an error (${error_variable}) while running")
endif()

