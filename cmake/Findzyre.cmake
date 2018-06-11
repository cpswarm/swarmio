# Get include path
find_path(ZYRE_INCLUDE_DIRS
    NAMES "zyre.h"
    HINTS "${DEPENDENCIES_INSTALL_DIR}/include"
)

# Get library path
find_library(ZYRE_LIBRARIES
    NAMES "zyre"
    HINTS "${CMAKE_PREFIX_PATH}/lib"
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
