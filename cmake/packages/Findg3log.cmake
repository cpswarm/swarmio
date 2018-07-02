# Get include path
find_path(G3LOG_INCLUDE_DIRS
    NAMES "g3log/g3log.hpp"
    HINTS ${G3LOG_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Get library path
find_library(G3LOG_LIBRARIES
    NAMES "g3logger"
    HINTS ${G3LOG_ROOT}
    PATH_SUFFIXES "lib"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G3LOG
    REQUIRED_VARS 
        G3LOG_LIBRARIES 
        G3LOG_INCLUDE_DIRS
)
mark_as_advanced(
    G3LOG_FOUND
    G3LOG_LIBRARIES 
    G3LOG_INCLUDE_DIRS
)
