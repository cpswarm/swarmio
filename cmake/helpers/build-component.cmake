# Macro to be called before each component is built
macro(swarmio_build_component_start _name _version)

    # Create an uppercase version of the name
    string(TOUPPER "${_name}" _upcase_name)

    # Save version
    set("${_upcase_name}_VERSION" ${_version})

    # Build target specific argument list with dependency roots
    set(_cmake_args ${SWARMIO_SUBPROJECT_TARGET_ARGS})
    if (NOT "${ARGN}" STREQUAL "")
        set("${_upcase_name}_DEPENDENCIES" ${ARGN})
        foreach (_dependency ${ARGN})
            string(TOUPPER "${_dependency}" _upcase_dependency)
            set(_current "${_upcase_dependency}_ROOT")
            list(APPEND _cmake_args "-D${_upcase_dependency}_ROOT=${${_current}}")
        endforeach()
    endif()
    set("${_upcase_name}_CMAKE_ARGS" ${_cmake_args})

    # Determine correct installation directory
    if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
        set("${_upcase_name}_ROOT" "${CMAKE_CURRENT_BINARY_DIR}/${_name}-${SWARMIO_TARGET_ARCHITECTURE}-output")
    else()
        set("${_upcase_name}_ROOT" ${CMAKE_INSTALL_PREFIX})
    endif()

endmacro()

# Macro to be called after a component has been built
macro(swarmio_build_component_finish _name)

    # Create an uppercase version of the name
    string(TOUPPER "${_name}" _upcase_name)

    # Add target dependencies
    if (NOT "${${_upcase_name}_DEPENDENCIES}" STREQUAL "")
        add_dependencies(${_name} ${${_upcase_name}_DEPENDENCIES})
    endif()

    # Check if a prefix was supplied
    if (${ARGC} EQUAL 1)
        set(_prefix ${CMAKE_INSTALL_PREFIX})
    else()
        set(_prefix "${ARGV1}")
    endif()    

    # If running in packaging mode, build package
    if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
        swarmio_build_package(${_name} "${${_upcase_name}_VERSION}" ${_prefix} ${${_upcase_name}_DEPENDENCIES})
    endif()

endmacro()