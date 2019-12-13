# Get include path
find_path(SODIUM_INCLUDE_DIRS
    NAMES "sodium.h"
    HINTS ${SODIUM_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Get library path
find_library(SODIUM_LIBRARIES
    NAMES "sodium"
    HINTS ${SODIUM_ROOT}
    PATH_SUFFIXES "lib"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SODIUM
    REQUIRED_VARS 
        SODIUM_LIBRARIES 
        SODIUM_INCLUDE_DIRS
)
mark_as_advanced(
    SODIUM_FOUND
    SODIUM_LIBRARIES 
    SODIUM_INCLUDE_DIRS
)

