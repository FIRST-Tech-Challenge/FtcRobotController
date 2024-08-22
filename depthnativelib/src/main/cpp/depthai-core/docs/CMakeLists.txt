# Get doxygen executable - required
find_package(Doxygen REQUIRED)

# options
set(DEPTHAI_DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/" CACHE STRING "Output directory where doxygen should generate the documentation")

# Retrieves all public header files for specified target
include(target-public-headers)
get_target_public_headers(${TARGET_CORE_NAME} header_files)

# Creates a space separated list of files
set(DOXYGEN_INPUT_FILES "")
foreach(file ${header_files})
    set(DOXYGEN_INPUT_FILES "${DOXYGEN_INPUT_FILES} \"${file}\"")
endforeach()

# Doxygen variables
set(DOXYGEN_OUTPUT_DIR ${DEPTHAI_DOXYGEN_OUTPUT_DIR})
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/html/index.html)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)


# Replace variables inside @@ with the current values
configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

# Create needed directories
# Doxygen won't create this for us
file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR}) 

# Add command to run doxygen documentation
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
                   DEPENDS "${header_files}"
                   COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   MAIN_DEPENDENCY ${DOXYFILE_OUT} ${DOXYFILE_IN}
                   COMMENT "Generating docs"
                   VERBATIM
                   COMMAND_EXPAND_LISTS)

# Add a doxygen target
add_custom_target(doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

# Add property to target, output file
set_target_properties(doxygen PROPERTIES DOXYGEN_OUTPUT_DIR ${DOXYGEN_OUTPUT_DIR}) 