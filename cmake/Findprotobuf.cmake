# Get include path
find_path(PROTOBUF_INCLUDE_DIRS
    NAMES "google/protobuf/message.h"
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

# Determine library name
if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(PROTOBUF_NAMES_LIB "protobufd" "libprotobufd.lib")
else()
    set(PROTOBUF_NAMES_LIB "protobuf" "libprotobuf.lib")
endif()

# Get library path
find_library(PROTOBUF_LIBRARIES
    NAMES ${PROTOBUF_NAMES_LIB}
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PROTOBUF
    REQUIRED_VARS 
        PROTOBUF_LIBRARIES 
        PROTOBUF_INCLUDE_DIRS
)
mark_as_advanced(
    PROTOBUF_FOUND
    PROTOBUF_LIBRARIES 
    PROTOBUF_INCLUDE_DIRS
)
