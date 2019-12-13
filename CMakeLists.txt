# Set minimum required version and baseline policy
cmake_minimum_required(VERSION 3.5)

# Set project name
project(swarmio VERSION 0.9)

# Use ExternalProject for the superbuild
include(ExternalProject)

# Define common arguments for projects built for the host architecture
set (SWARMIO_SUBPROJECT_HOST_ARGS
    "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
)

# Define common arguments for projects built for the target architecture
set (SWARMIO_SUBPROJECT_TARGET_ARGS
    "-DCMAKE_STAGING_PREFIX=<INSTALL_DIR>"
    "-DCMAKE_MODULE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/cmake/packages"
)

# If a verbose makefile was requested, pass it on to subprojects
if (CMAKE_VERBOSE_MAKEFILE)
    list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS
        "-DCMAKE_VERBOSE_MAKEFILE=ON"
    )
    list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS
        "-DCMAKE_VERBOSE_MAKEFILE=ON"
    )
endif()

# Include helpers
include("cmake/helpers/prepare-architecture.cmake")
include("cmake/helpers/prepare-mode.cmake")
include("cmake/helpers/prepare-multistrap.cmake")
include("cmake/helpers/build-component.cmake")
include("cmake/helpers/build-package.cmake")

# Build dependencies
include("cmake/dependencies/build-libzmq.cmake")
include("cmake/dependencies/build-czmq.cmake")
include("cmake/dependencies/build-zyre.cmake")
include("cmake/dependencies/build-concurrentqueue.cmake")
include("cmake/dependencies/build-readerwriterqueue.cmake")
include("cmake/dependencies/build-protobuf.cmake")
include("cmake/dependencies/build-replxx.cmake")
include("cmake/dependencies/build-g3log.cmake")
include("cmake/dependencies/build-sodium.cmake")

# Prepare
swarmio_build_component_start(libswarmio ${PROJECT_VERSION} 
    libzmq 
    czmq 
    zyre 
    concurrentqueue
    readerwriterqueue
    protobuf
    replxx 
    g3log
    sodium
)

# Build libswarmio
ExternalProject_Add(libswarmio
    DEPENDS protobuf-host
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libswarmio"
    BUILD_ALWAYS ON
    PREFIX "libswarmio-${SWARMIO_TARGET_ARCHITECTURE}"
    BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
    INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
    INSTALL_DIR ${LIBSWARMIO_ROOT}
    LIST_SEPARATOR "|"
    CMAKE_ARGS  
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
        "-DTOOL_PROTOC=${TOOL_PROTOC}"
        ${LIBSWARMIO_CMAKE_ARGS}
)

# Finish
swarmio_build_component_finish(libswarmio)

# Build ROS node
if (SWARMIO_BUILD_ROS_NODE)

    # Do not try to build the ROS node on Windows
    if (MSVC)
        message(FATAL_ERROR "Building the ROS node is not supported with the MSVC compiler.")
    endif()

    # BUild libconfig
    include("cmake/dependencies/build-libconfig.cmake")

    # Prepare
    swarmio_build_component_start(swarmros ${PROJECT_VERSION} 
        libzmq 
        czmq 
        zyre 
        concurrentqueue
        readerwriterqueue
        protobuf
        replxx 
        g3log
        libconfig 
	    sodium
        libswarmio
    )

    # Determine ROS prefix
    if ("${SWARMIO_ROS_PREFIX}" STREQUAL "")
        if (EXISTS "${SWARMIO_SYSROOT}/opt/ros/melodic")
            set(SWARMIO_ROS_PREFIX "/opt/ros/melodic")
            message(STATUS "Using ROS Melodic Morenia found at /opt/ros/melodic.")
        elseif (EXISTS "${SWARMIO_SYSROOT}/opt/ros/kinetic")
            set(SWARMIO_ROS_PREFIX "/opt/ros/kinetic")
            message(STATUS "Using ROS Kinetic Kame found at /opt/ros/kinetic.")
        else()
            message(FATAL_ERROR "Cannot detect ROS prefix path, set SWARMIO_ROS_PREFIX manually.")
        endif()
    else()
        message(STATUS "Using ROS installation at ${SWARMIO_ROS_PREFIX}.")
    endif()

    # Build
    ExternalProject_Add(swarmros
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/swarmros"
        BUILD_ALWAYS ON
        PREFIX "swarmros-${SWARMIO_TARGET_ARCHITECTURE}"
        BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
        INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
        INSTALL_DIR "swarmros-${SWARMIO_TARGET_ARCHITECTURE}-output"
        LIST_SEPARATOR "|"
        CMAKE_ARGS  
            ${SWARMROS_CMAKE_ARGS}
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
            "-DCATKIN_ENABLE_TESTING=OFF"
            "-DCATKIN_BUILD_BINARY_PACKAGE=1"
            "-DCMAKE_PREFIX_PATH=${SWARMIO_SYSROOT}${SWARMIO_ROS_PREFIX}"
            "-DCMAKE_INSTALL_PREFIX=${SWARMIO_ROS_PREFIX}"
    )

    # Finish
    swarmio_build_component_finish(swarmros ${SWARMIO_ROS_PREFIX})

endif()
