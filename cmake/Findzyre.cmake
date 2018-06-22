# Get include path
find_path(ZYRE_INCLUDE_DIRS
    NAMES "zyre.h"
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

# Get library path
find_library(ZYRE_LIBRARIES
    NAMES "zyre"
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
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
