# Get include path
find_path(ZYRE_INCLUDE_DIRS
    NAMES "zyre.h"
    HINTS ${ZYRE_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Get library path
find_library(ZYRE_LIBRARIES
    NAMES "zyre"
    HINTS ${ZYRE_ROOT}
    PATH_SUFFIXES "lib"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ZYRE
    REQUIRED_VARS 
        ZYRE_LIBRARIES 
        ZYRE_INCLUDE_DIRS
)
mark_as_advanced(
    ZYRE_FOUND
    ZYRE_LIBRARIES 
    ZYRE_INCLUDE_DIRS    
)
