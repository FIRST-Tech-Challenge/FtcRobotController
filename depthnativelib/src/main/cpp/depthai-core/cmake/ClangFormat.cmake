# Copyright Tomas Zeman 2019.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

function(clangformat_setup files target)

  if(NOT CLANG_FORMAT_BIN)
    set(CLANG_FORMAT_BIN clang-format)
  endif()

  if(NOT EXISTS ${CLANG_FORMAT_BIN})
    find_program(CLANG_FORMAT_BIN_tmp NAMES clang-format clang-format-10 clang-format-11 clang-format-12 clang-format-13 clang-format-14)
    if(CLANG_FORMAT_BIN_tmp)
      set(CLANG_FORMAT_BIN ${CLANG_FORMAT_BIN_tmp})
      unset(CLANG_FORMAT_BIN_tmp)
    else()
      message(STATUS "ClangFormat: ${CLANG_FORMAT_BIN} not found! Target 'clangformat' not available...")
      return()
    endif()
  endif()

  foreach(clangformat_source ${files})
    get_filename_component(clangformat_source ${clangformat_source} ABSOLUTE)
    list(APPEND clangformat_sources ${clangformat_source})
  endforeach()

  add_custom_target(${target}_clangformat
    COMMAND
      ${CLANG_FORMAT_BIN}
      -style=file
      -i
      ${clangformat_sources}
    WORKING_DIRECTORY
      ${CMAKE_SOURCE_DIR}
    COMMENT
      "Formating with ${CLANG_FORMAT_BIN} ..."
  )

  if(TARGET clangformat)
    add_dependencies(clangformat ${target}_clangformat)
  else()
    add_custom_target(clangformat DEPENDS ${target}_clangformat)
  endif()
endfunction()


macro(header_directories header_dirs return_list)

  foreach(header_dir ${header_dirs})
    file(GLOB_RECURSE new_list "${header_dir}/*.hpp")
    set(file_list "")
    foreach(file_path ${new_list})
      set(file_list "${file_list}" ${file_path})
    endforeach()
    list(REMOVE_DUPLICATES file_list)
    set(${return_list} "${${return_list}}" "${file_list}")
  endforeach()

endmacro()

function(target_clangformat_setup target header_dirs)
  get_target_property(target_sources ${target} SOURCES)
  header_directories("${header_dirs}" header_files)
  set(target_files_to_format "${target_sources};${header_files}")
  clangformat_setup("${target_files_to_format}" ${target})
endfunction()
