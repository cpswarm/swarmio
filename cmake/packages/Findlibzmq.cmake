# Get include path
find_path(LIBZMQ_INCLUDE_DIRS
    NAMES zmq.h
    HINTS ${LIBZMQ_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Get library path
if (LIBZMQ_INCLUDE_DIRS)
    if (NOT ${CMAKE_CXX_PLATFORM_ID} STREQUAL "Windows")

        # On Unix-like platforms, we are in luck, we can simply do this
        find_library(LIBZMQ_LIBRARIES
            NAMES zmq
            HINTS ${LIBZMQ_ROOT}
            PATH_SUFFIXES "lib"
            NO_CMAKE_FIND_ROOT_PATH
            NO_DEFAULT_PATH
        ) 
        
    else()

        # On Windows, we have to assemble the file name using information from the main header
        set(LIBZMQ_HEADER "${LIBZMQ_INCLUDE_DIRS}/zmq.h")
        function(LIBZMQ_EXTRACT_VERSION _VERSION_COMPONENT _VERSION_OUTPUT)
            set(CMAKE_MATCH_1 "0")
            set(_VERSION_PATTERN "^[ \\t]*#define[ \\t]+${_VERSION_COMPONENT}[ \\t]+([0-9]+)$")
            file(STRINGS "${LIBZMQ_HEADER}" _MATCHING_ROWS REGEX "${_VERSION_PATTERN}")
            string(REGEX MATCH "${_VERSION_PATTERN}" _TEMP "${_MATCHING_ROWS}")
            set(${_VERSION_OUTPUT} "${CMAKE_MATCH_1}" PARENT_SCOPE)
        endfunction()
        LIBZMQ_EXTRACT_VERSION("ZMQ_VERSION_MAJOR" LIBZMQ_VERSION_MAJOR)
        LIBZMQ_EXTRACT_VERSION("ZMQ_VERSION_MINOR" LIBZMQ_VERSION_MINOR)
        LIBZMQ_EXTRACT_VERSION("ZMQ_VERSION_PATCH" LIBZMQ_VERSION_PATCH)
        message(STATUS "ZeroMQ version: ${LIBZMQ_VERSION_MAJOR}.${LIBZMQ_VERSION_MINOR}.${LIBZMQ_VERSION_PATCH}")

        # Save version information
        if (LIBZMQ_FIND_VERSION_COUNT GREATER 2)
            set(LIBZMQ_VERSION "${LIBZMQ_VERSION_MAJOR}.${LIBZMQ_VERSION_MINOR}.${LIBZMQ_VERSION_PATCH}")
        else()
            set(LIBZMQ_VERSION "${LIBZMQ_VERSION_MAJOR}.${LIBZMQ_VERSION_MINOR}")
        endif()

        # Depending on build type, select the appropriate library version
        if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
            set(LIBZMQ_NAMES_LIB "libzmq_d" "libzmq-${CMAKE_VS_PLATFORM_TOOLSET}-mt-gd-${LIBZMQ_VERSION_MAJOR}_${LIBZMQ_VERSION_MINOR}_${LIBZMQ_VERSION_PATCH}")
        else()
            set(LIBZMQ_NAMES_LIB "libzmq" "libzmq-${CMAKE_VS_PLATFORM_TOOLSET}-mt-${LIBZMQ_VERSION_MAJOR}_${LIBZMQ_VERSION_MINOR}_${LIBZMQ_VERSION_PATCH}")
        endif()

        # Find library
        find_library(LIBZMQ_LIBRARIES
            NAMES ${LIBZMQ_NAMES_LIB}
            HINTS ${LIBZMQ_ROOT}
            PATH_SUFFIXES "lib"
            NO_CMAKE_FIND_ROOT_PATH
            NO_DEFAULT_PATH
        )

    endif()
endif()

# Set required variables 
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBZMQ
    REQUIRED_VARS 
        LIBZMQ_LIBRARIES
        LIBZMQ_INCLUDE_DIRS
    VERSION_VAR
        LIBZMQ_VERSION
)
mark_as_advanced(
    LIBZMQ_FOUND
    LIBZMQ_LIBRARIES 
    LIBZMQ_INCLUDE_DIRS
)