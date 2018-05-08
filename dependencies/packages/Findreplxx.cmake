# Get include path
find_path(REPLXX_INCLUDE_DIRS
    NAMES "replxx.h"
    HINTS "${CMAKE_PREFIX_PATH}/include"
)

# Get library path
find_library(REPLXX_LIBRARIES
    NAMES 
	"replxx-d"
	"libreplxx.a"
    HINTS "${CMAKE_PREFIX_PATH}/lib"
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(REPLXX
    REQUIRED_VARS 
        REPLXX_LIBRARIES 
        REPLXX_INCLUDE_DIRS
)
mark_as_advanced(
    REPLXX_FOUND
    REPLXX_LIBRARIES 
    REPLXX_INCLUDE_DIRS
)
