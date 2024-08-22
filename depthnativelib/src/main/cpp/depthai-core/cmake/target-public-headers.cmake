# CMake helper script which defines a function to retrieve targets public headers (if separated in different directories)

function(get_target_public_headers target output_variable_name)
    
    # Get both interface include directories and system include directories
    get_target_property(header_dirs ${target} INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(header_system_dirs ${target} INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)

    # Remove system include directories
    set(non_system_dirs ${header_dirs})
    foreach(sys_dir ${header_system_dirs})
        list(REMOVE_ITEM non_system_dirs "${sys_dir}")
    endforeach()

    # Get all header files
    header_directories("${non_system_dirs}" header_files)

    set(${output_variable_name} ${header_files} PARENT_SCOPE)

endfunction()


function(get_header_files include_directories output_variable_name)
    
    # Get all header files
    header_directories("${include_directories}" header_files)

    # Set to parent
    set(${output_variable_name} ${header_files} PARENT_SCOPE)

endfunction()

# Internal helper, converts header directory list to list of .hpp files
macro(header_directories header_dirs return_list)

    #message(STATUS "folders: ${header_dirs}")
    foreach(header_dir ${header_dirs})
        # only use build_interface includes

        string(REGEX MATCHALL "\\$<BUILD_INTERFACE:([^ \t\n\r]+)>" tmp_output_variable "${header_dir}")
        #message(STATUS "inspecting folder: ${header_dir}, stripped ${hdir}, regex count ${CMAKE_MATCH_COUNT}, regex match 0 and 1, ${CMAKE_MATCH_0}, ${CMAKE_MATCH_1}")
        # If regex matched and path is present
        if(CMAKE_MATCH_COUNT GREATER 0)
            set(hdir "${CMAKE_MATCH_1}")
        else()    
            # Strip generator expresssions
            string(GENEX_STRIP "${header_dir}" hdir)
        endif()
        
        # if hdir isn't an empty path
        if(hdir)

            #message(STATUS "inspecting folder: ${hdir}")
            file(GLOB_RECURSE new_list "${hdir}/*.hpp")
            set(file_list "")

            foreach(file_path ${new_list})
                #message(STATUS "inspecting file: ${file_path}")
                set(file_list "${file_list}" ${file_path})
            endforeach()
            
            list(REMOVE_DUPLICATES file_list)
            set(${return_list} "${${return_list}}" "${file_list}")
            
        endif()

    endforeach()

endmacro()




