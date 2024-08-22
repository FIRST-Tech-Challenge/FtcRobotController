# This script downloads depthai device side artifacts

function(DepthaiBootloaderDownload)

    ### VARIABLES
    # Artifactory
    set(DOWNLOADER_BASE_URL "https://artifacts.luxonis.com/artifactory")

    # Repositories
    set(DOWNLOADER_REPO_SNAPSHOT "luxonis-myriad-snapshot-local")
    set(DOWNLOADER_REPO_RELEASE "luxonis-myriad-release-local")

    # Prefix
    set(DOWNLOADER_ARTIFACT_PREFIX "depthai-bootloader")

    # Errors and retry count
    set(DOWNLOADER_TIMEOUT_S 300)
    set(DOWNLOADER_INACTIVE_TIMEOUT_S 60)
    set(DOWNLOADER_RETRY_NUM 5)
    ### END VARIABLES


    # PARSE ARGUMENTS
    set(_depthai_bootloader_shared_commit ${ARGV0})
    set(_enforce_depthai_bootloader_shared_commit ${ARGV1})
    set(folder "${ARGV2}")
    set(output_list_var "${ARGV3}")
    set(maturity "${ARGV4}")
    set(commit_version_arg "${ARGV5}")

    message(STATUS "Downloading depthai bootloader")

    # END PARSE ARGUMENTS

    #DEBUG
    message(STATUS "folder: ${folder}")
    message(STATUS "maturity: ${maturity}")
    message(STATUS "commit_version_arg: ${commit_version_arg}")

    string(TOLOWER "${maturity}" maturity_lower)

    # Switch between maturity
    if("${maturity_lower}" STREQUAL "snapshot")
        set(_selected_repo "${DOWNLOADER_REPO_SNAPSHOT}")
        set(commit ${commit_version_arg})

        # Create download directory string
        string(CONFIGURE "@DOWNLOADER_BASE_URL@/@DOWNLOADER_REPO_SNAPSHOT@/@DOWNLOADER_ARTIFACT_PREFIX@/@commit@" _download_directory_url)

        # Create _version_commit_identifier
        set(_version_commit_identifier "${commit}")

    elseif(${maturity_lower} STREQUAL "release")
        set(_selected_repo "${DOWNLOADER_REPO_RELEASE}")
        set(version ${commit_version_arg})

        # Create download directory string
        string(CONFIGURE "@DOWNLOADER_BASE_URL@/@DOWNLOADER_REPO_RELEASE@/@DOWNLOADER_ARTIFACT_PREFIX@/@version@" _download_directory_url)

        # Create _version_commit_identifier
        set(_version_commit_identifier "${version}")
    else()
        # Not a recognized maturity level
        message(FATAL_ERROR "Cannot download depthai-bootloader binaries. Maturity level not recognized (${maturity_lower})")
        return()
    endif()

    # Prints error message
    macro(PrintErrorMessage status)
        if(${status} EQUAL 22)
            message(STATUS "Resource not found, check if commit hash is correctly specified.\n")
        elseif(${status} EQUAL 28)
            message(STATUS "Timeout.\n")
        elseif(${status} EQUAL 99)
            message(STATUS "Couldn't retrieve files correctly, checksum mismatch.")
        else()
            message(STATUS "Unknown error.\n")
        endif()
    endmacro()

    # Download files
    function(DownloadAndChecksum url url_checksum output status_var)

        # Check if file already downloaded (in resources)
        if(EXISTS "${output}")
            get_filename_component(_filename "${output}" NAME)
            message(STATUS "File already downloaded (resources): ${_filename}")
            set("${status_var}" "0" PARENT_SCOPE)
            return()
        endif()

        # Retry again if failed
        set(_num_retries_left ${DOWNLOADER_RETRY_NUM})
        # Set error by default
        set("${status_var}" "1" PARENT_SCOPE)

        while(NOT ${_num_retries_left} EQUAL 0)
            math(EXPR _num_retries_left "${_num_retries_left} - 1")

            # Download checksum first
            file(DOWNLOAD "${url_checksum}" "${output}.checksum" STATUS _status TIMEOUT ${DOWNLOADER_TIMEOUT_S})
            # Read checksum to file
            file(READ "${output}.checksum" _file_checksum)
            string(REGEX REPLACE "\n$" "" _file_checksum "${_file_checksum}")
            # Remove checksum file
            file(REMOVE "${output}.checksum")
            #CHECKS
            list(GET _status 0 _status_num)
            if(${_status_num})
                message(STATUS "Status error: ${_status}")
                set("${status_var}" "${_status_num}" PARENT_SCOPE)
                continue()
            endif()


            # Download file and validate checksum
            file(DOWNLOAD "${url}" "${output}" INACTIVITY_TIMEOUT ${DOWNLOADER_INACTIVE_TIMEOUT_S} STATUS _status TIMEOUT ${DOWNLOADER_TIMEOUT_S} SHOW_PROGRESS)

            #CHECKS
            list(GET _status 0 _status_num)
            if(${_status_num})
                message(STATUS "Status error: ${_status}")
                set("${status_var}" "${_status_num}" PARENT_SCOPE)
                continue()
            endif()

            # Now check if hash matches (if both files were downloaded without timeouts)
            file(SHA256 ${output} _downloaded_checksum)

            # if hashes don't match
            if(NOT (_downloaded_checksum STREQUAL _file_checksum))
                message(STATUS "Downloaded file checksum mismatch: ${_downloaded_checksum} != {_file_checksum}")
                set("${status_var}" "99" PARENT_SCOPE)
                continue()
            endif()

            # If no errors happened, set status to 0
            set("${status_var}" "0" PARENT_SCOPE)
            # And break the loop
            break()

        endwhile()

    endfunction()


    # Check if depthai-bootloader-shared matches
    if(_depthai_bootloader_shared_commit)
        message(STATUS "bootloader shared commit: ${_depthai_bootloader_shared_commit}")
        DownloadAndChecksum(
            "${_download_directory_url}/depthai-bootloader-shared-commit-hash-${_version_commit_identifier}.txt" # File
            "${_download_directory_url}/depthai-bootloader-shared-commit-hash-${_version_commit_identifier}.txt.sha256" # File checksum
            "${folder}/depthai-bootloader-shared-commit-hash-${_version_commit_identifier}.txt"
            status
        )
        if(${status})

            message(STATUS "Couldn't check if depthai-bootloader-shared codebase matches between device and host")
            if(${_enforce_depthai_bootloader_shared_commit})
                message(FATAL_ERROR "Aborting.\n")
            endif()

        else()

            set(_message_mode WARNING)
            if(${_enforce_depthai_bootloader_shared_commit})
                set(_message_mode FATAL_ERROR)
            endif()

            # Read commit hash file
            file(READ "${folder}/depthai-bootloader-shared-commit-hash-${_version_commit_identifier}.txt" _device_depthai_bootloader_shared_commit_hash)
            string(REGEX REPLACE "\n$" "" _device_depthai_bootloader_shared_commit_hash "${_device_depthai_bootloader_shared_commit_hash}")
            string(REGEX REPLACE "\n$" "" _depthai_bootloader_shared_commit "${_depthai_bootloader_shared_commit}")
            string(COMPARE EQUAL "${_device_depthai_bootloader_shared_commit_hash}" "${_depthai_bootloader_shared_commit}" _is_same)

            # If commits dont match
            if(NOT ${_is_same})
                message(${_message_mode} "depthai-bootloader-shared codebases differ between device and host. Enforce (CI): ${_enforce_depthai_bootloader_shared_commit} (device: ${_device_depthai_bootloader_shared_commit_hash}, host: ${_depthai_bootloader_shared_commit}")
            else()
                message(STATUS "depthai-bootloader-shared between device and host MATCH!. (device: ${_device_depthai_bootloader_shared_commit_hash}, host: ${_depthai_bootloader_shared_commit}")
            endif()

        endif()

    endif()


    # Download depthai-bootloader firmware package
    message(STATUS "Downloading and checking depthai-bootloader-fwp.tar.xz")
    DownloadAndChecksum(
        "${_download_directory_url}/depthai-bootloader-fwp-${_version_commit_identifier}.tar.xz" # File
        "${_download_directory_url}/depthai-bootloader-fwp-${_version_commit_identifier}.tar.xz.sha256" # File checksum
        "${folder}/depthai-bootloader-fwp-${_version_commit_identifier}.tar.xz"
        status
    )
    if(${status})
        message(STATUS "\nCouldn't download depthai-bootloader-fwp.tar.xz\n")
        PrintErrorMessage(${status})
        message(FATAL_ERROR "Aborting.\n")
    endif()
    # add depthai-bootloader-fwp.tar.xz to list
    list(APPEND "${output_list_var}" "${folder}/depthai-bootloader-fwp-${_version_commit_identifier}.tar.xz")


    # Set list of files as output
    set("${output_list_var}" "${${output_list_var}}" PARENT_SCOPE)

endfunction()
