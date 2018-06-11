# Get include path
find_path(PROTOBUF_INCLUDE_DIRS
    NAMES "google/protobuf/message.h"
    HINTS "${CMAKE_PREFIX_PATH}/include"
)

# Get library path
find_library(PROTOBUF_LIBRARIES
    NAMES "protobuf" "libprotobufd.lib"
    HINTS "${CMAKE_PREFIX_PATH}/lib"
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
