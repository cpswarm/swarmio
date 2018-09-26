# Check for the presence of dpkg-deb and fakeroot
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    find_program(TOOL_DPKG_DEB "dpkg-deb")
    if (TOOL_DPKG_DEB STREQUAL "TOOL_DPGK_DEB_NOT_FOUND")
        message(FATAL_ERROR "It seems that dpkg-deb is not available on your system, install it before trying to build packages.")
    endif()
    find_program(TOOL_FAKEROOT "fakeroot")
    if (TOOL_FAKEROOT STREQUAL "TOOL_FAKEROOT_NOT_FOUND")
        message(FATAL_ERROR "It seems that fakeroot is not available on your system, install it before trying to build packages.")
    endif()
    find_program(TOOL_RSYNC "rsync")
    if (TOOL_RSYNC STREQUAL "TOOL_RSYNC_NOT_FOUND")
        message(TOOL_RSYNC "It seems that rsync is not available on your system, install it before trying to build packages.")
    endif()
    find_program(TOOL_MKDIR "mkdir")
    if (TOOL_MKDIR STREQUAL "TOOL_MKDIR_NOT_FOUND")
        message(TOTOOL_MKDIR "It seems that mkdir is not available on your system, install it before trying to build packages.")
    endif()
endif()

# Convert a target name to a package name
function(swarmio_package_name_for_target _target _output)

    # Check if the target name has the project name in it
    string(FIND "${_target}" "swarm" _position)

    # Add the "swarmio-dependency-" prefix if not
    if (_position EQUAL -1)
        set("${_output}" "swarmio-dependency-${_target}" PARENT_SCOPE)
    else()
        set("${_output}" "${_target}" PARENT_SCOPE)
    endif()

endfunction()

# Build a deb package for an ExternalProject
function(swarmio_build_package _target _version _prefix)

    # Get package name
    swarmio_package_name_for_target(${_target} _name)

    # Set package name, version and target architecture
    set(PACKAGE_NAME ${_name})
    set(PACKAGE_VERSION ${_version})
    set(PACKAGE_ARCH ${SWARMIO_TARGET_ARCHITECTURE})

    # Build dependency list
    foreach(_element ${ARGN})
        swarmio_package_name_for_target(${_element} _dependency)
        list(APPEND _dependencies ${_dependency})
    endforeach()
    string(REPLACE ";" ", " PACKAGE_DEPENDS "${_dependencies}")

    # Assemble output file paths
    set(_control_file_path "debian/${_name}-control")
    set(_output_file_path "packages/${_name}-${_version}-${SWARMIO_TARGET_ARCHITECTURE}.deb")

    # Generate package descriptor
    configure_file("cmake/templates/control.tpl" ${_control_file_path})

    # Add package generation step
    ExternalProject_Add_Step(${_target} generate-deb
        DEPENDEES install
        BYPRODUCTS ${_output_file_path}
        COMMAND ${CMAKE_COMMAND} -E remove_directory "<TMP_DIR>/package"
        COMMAND ${TOOL_MKDIR} -p "<TMP_DIR>/package${_prefix}"
        COMMAND ${TOOL_RSYNC} -rtvu --delete --links "<INSTALL_DIR>/" "<TMP_DIR>/package${_prefix}"
        COMMAND ${CMAKE_COMMAND} -E copy ${_control_file_path} "<TMP_DIR>/package/DEBIAN/control"
        COMMAND ${TOOL_FAKEROOT} ${TOOL_DPKG_DEB} --build  "<TMP_DIR>/package" 
        COMMAND ${CMAKE_COMMAND} -E copy "<TMP_DIR>/package.deb" ${_output_file_path}
    )

endfunction()