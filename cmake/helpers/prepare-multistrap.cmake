# Build multistrap environment
if (NOT "${SWARMIO_MULTISTRAP_CONFIGURATION}" STREQUAL "")

    # Ensure that we have the tools
    find_program(TOOL_MULTISTRAP "multistrap")
    if (TOOL_MULTISTRAP STREQUAL "TOOL_MULTISTRAP_NOT_FOUND")
        message(FATAL_ERROR "It seems that multistrap is not available on your system, install it before trying to build a sysroot.")
    endif()
    find_program(TOOL_BASH "bash")
    if (TOOL_BASH STREQUAL "TOOL_BASH_NOT_FOUND")
        message(FATAL_ERROR "It seems that bash is not available on your system, install it before trying to build a sysroot.")
    endif()
    find_program(TOOL_READLINK "readlink")
    if (TOOL_READLINK STREQUAL "TOOL_READLINK_NOT_FOUND")
        message(FATAL_ERROR "It seems that readlink is not available on your system, install it before trying to build a sysroot.")
    endif()
    find_program(TOOL_REALPATH "realpath")
    if (TOOL_REALPATH STREQUAL "TOOL_REALPATH_NOT_FOUND")
        message(FATAL_ERROR "It seems that realpath is not available on your system, install it before trying to build a sysroot.")
    endif()
    find_program(TOOL_DIRNAME "dirname")
    if (TOOL_DIRNAME STREQUAL "TOOL_DIRNAME_NOT_FOUND")
        message(FATAL_ERROR "It seems that dirname is not available on your system, install it before trying to build a sysroot.")
    endif()
    find_program(TOOL_FIND "find")
    if (TOOL_FIND STREQUAL "TOOL_FIND_NOT_FOUND")
        message(FATAL_ERROR "It seems that find is not available on your system, install it before trying to build a sysroot.")
    endif()

    # Set sysroot path
    set(MULTISTRAP_DIR "${CMAKE_BINARY_DIR}/sysroot-${SWARMIO_MULTISTRAP_CONFIGURATION}-${SWARMIO_TARGET_ARCHITECTURE}")
    set(MULTISTRAP_CONF "${MULTISTRAP_DIR}.conf")

    # Select Ubuntu source repository
    if (SWARMIO_TARGET_ARCHITECTURE STREQUAL "amd64")
        set(MULTISTRAP_SOURCE "http://archive.ubuntu.com/ubuntu")
    else()
        set(MULTISTRAP_SOURCE "http://ports.ubuntu.com/ubuntu-ports")
    endif()

    # Set parameters based on selected configuration
    if (SWARMIO_MULTISTRAP_CONFIGURATION STREQUAL "xenial")
        set(MULTISTRAP_UBUNTU_VERSION "xenial")
        set(MULTISTRAP_GCC_VERSION "5")
        configure_file("cmake/templates/multistrap-ubuntu.tpl" ${MULTISTRAP_CONF})
    elseif (SWARMIO_MULTISTRAP_CONFIGURATION STREQUAL "xenial-ros")
        set(MULTISTRAP_UBUNTU_VERSION "xenial")
        set(MULTISTRAP_ROS_VERSION "kinetic")
        set(MULTISTRAP_GCC_VERSION "5")
        configure_file("cmake/templates/multistrap-ubuntu-ros.tpl" ${MULTISTRAP_CONF})
    elseif (SWARMIO_MULTISTRAP_CONFIGURATION STREQUAL "bionic")
        set(MULTISTRAP_UBUNTU_VERSION "bionic")
        set(MULTISTRAP_GCC_VERSION "7")
        configure_file("cmake/templates/multistrap-ubuntu.tpl" ${MULTISTRAP_CONF})
    elseif (SWARMIO_MULTISTRAP_CONFIGURATION STREQUAL "bionic-ros")
        set(MULTISTRAP_UBUNTU_VERSION "bionic")
        set(MULTISTRAP_ROS_VERSION "melodic")
        set(MULTISTRAP_GCC_VERSION "7")
        configure_file("cmake/templates/multistrap-ubuntu-ros.tpl" ${MULTISTRAP_CONF})
    else()
        message(FATAL_ERROR "Unknown multistrap configuration '${SWARMIO_MULTISTRAP_CONFIGURATION}'")
    endif()

    # Add ROS public key beforehand
    if (DEFINED MULTISTRAP_ROS_VERSION)
        file(MAKE_DIRECTORY "${MULTISTRAP_DIR}/etc/apt/trusted.gpg.d")
        file(DOWNLOAD "http://packages.ros.org/ros.key" "${MULTISTRAP_DIR}/etc/apt/trusted.gpg.d/ros.gpg")
    endif()

    # Build root
    execute_process(
        COMMAND ${TOOL_MULTISTRAP} -d ${MULTISTRAP_DIR} -f ${MULTISTRAP_CONF} --arch ${SWARMIO_TARGET_ARCHITECTURE}
        RESULT_VARIABLE _return_value
    )
    if (NOT _return_value EQUAL 0)
        message(FATAL_ERROR "Multistrap has failed to establish the new sysroot.")
    endif()

    # Prepare reroot script
    set(MULTISTRAP_REROOT_SCRIPT "${CMAKE_BINARY_DIR}/reroot-symlinks.sh")
    configure_file("cmake/templates/reroot-symlinks.tpl" ${MULTISTRAP_REROOT_SCRIPT})

    # Reroot symlinks
    execute_process(
        COMMAND ${TOOL_BASH} ${MULTISTRAP_REROOT_SCRIPT} ${MULTISTRAP_DIR}
        RESULT_VARIABLE _return_value
    )
    if (NOT _return_value EQUAL 0)
        message(FATAL_ERROR "Cannot reroot symlinks in new sysroot.")
    endif()

    # Set sysroot
    set(SWARMIO_SYSROOT "${MULTISTRAP_DIR}")

endif()

# Pass sysroot onto subprojects, where it will be used by the toolchain
if (NOT "${SWARMIO_SYSROOT}" STREQUAL "")
    list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS
        "-DSWARMIO_SYSROOT=${SWARMIO_SYSROOT}"
    )
endif()

# Pass GCC version onto the toolchain
if (NOT "${MULTISTRAP_GCC_VERSION}" STREQUAL "")
    list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS
        "-DSWARMIO_GCC_VERSION=${MULTISTRAP_GCC_VERSION}"
    )
endif()