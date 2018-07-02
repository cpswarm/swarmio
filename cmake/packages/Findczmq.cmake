# Get include path
find_path(CZMQ_INCLUDE_DIRS
    NAMES "czmq.h"
    HINTS ${CZMQ_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Get library path
find_library(CZMQ_LIBRARIES
    NAMES "czmq"
    HINTS ${CZMQ_ROOT}
    PATH_SUFFIXES "lib"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CZMQ
    REQUIRED_VARS 
        CZMQ_LIBRARIES 
        CZMQ_INCLUDE_DIRS
)

mark_as_advanced(
    CZMQ_FOUND
    CZMQ_LIBRARIES 
    CZMQ_INCLUDE_DIRS
)