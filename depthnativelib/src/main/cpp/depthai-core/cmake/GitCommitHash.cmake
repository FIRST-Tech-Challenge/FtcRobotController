# for commit hash
find_package(Git)

set(commit_version "unknown")

if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    WORKING_DIRECTORY "${local_dir}"
    OUTPUT_VARIABLE commit_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
else()
  message(STATUS "GIT module not found")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../shared/version.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/../../shared/version.hpp @ONLY)

