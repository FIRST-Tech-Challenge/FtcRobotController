if(DEPTHAI_SHARED_LOCAL)
    set(DEPTHAI_SHARED_FOLDER ${DEPTHAI_SHARED_LOCAL})
else()
    set(DEPTHAI_SHARED_FOLDER ${CMAKE_CURRENT_LIST_DIR}/depthai-shared)
endif()

set(DEPTHAI_SHARED_3RDPARTY_HEADERS_PATH "depthai-shared/3rdparty")

set(DEPTHAI_SHARED_SOURCES
    ${DEPTHAI_SHARED_FOLDER}/src/datatype/DatatypeEnum.cpp
    ${DEPTHAI_SHARED_FOLDER}/src/utility/Checksum.cpp
)

set(DEPTHAI_SHARED_PUBLIC_INCLUDE
    ${DEPTHAI_SHARED_FOLDER}/include
)

set(DEPTHAI_SHARED_3RDPARTY_INCLUDE
    ${DEPTHAI_SHARED_FOLDER}/3rdparty
)

set(DEPTHAI_SHARED_INCLUDE
    ${DEPTHAI_SHARED_FOLDER}/src
)

# Try retriving depthai-shared commit hash (if cloned and not sources only)
find_package(Git)
if(GIT_FOUND AND NOT DEPTHAI_DOWNLOADED_SOURCES)

    if(NOT DEPTHAI_SHARED_LOCAL)
        # Check that submodule is initialized and updated
        execute_process(
            COMMAND ${GIT_EXECUTABLE} submodule status ${DEPTHAI_SHARED_FOLDER}
            WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
            OUTPUT_VARIABLE statusCommit
            ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        string(SUBSTRING ${statusCommit} 0 1 status)
        if("${status}" STREQUAL "-")
            message(FATAL_ERROR "Submodule 'depthai-shared' not initialized/updated. Run 'git submodule update --init --recursive' first")
        endif()
    endif()

    # Get depthai-shared current commit
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
        WORKING_DIRECTORY ${DEPTHAI_SHARED_FOLDER}
        RESULT_VARIABLE DEPTHAI_SHARED_COMMIT_RESULT
        OUTPUT_VARIABLE DEPTHAI_SHARED_COMMIT_HASH
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(${DEPTHAI_SHARED_COMMIT_RESULT} EQUAL 0)
        set(DEPTHAI_SHARED_COMMIT_FOUND TRUE)
    else()
        set(DEPTHAI_SHARED_COMMIT_FOUND FALSE)
    endif()
endif()

# Make sure files exist
foreach(source_file ${DEPTHAI_SHARED_SOURCES})
    message(STATUS "Checking file: ${source_file}")
    if(NOT EXISTS ${source_file})
        message(FATAL_ERROR "depthai-shared submodule files missing. Make sure to download prepackaged release instead of \"Source code\" on GitHub. Example: depthai-core-vX.Y.Z.tar.gz")
    endif()
endforeach()