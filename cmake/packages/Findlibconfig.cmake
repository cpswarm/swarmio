# Get include path
find_path(LIBCONFIG_INCLUDE_DIRS
    NAMES "libconfig.h++"
    HINTS ${LIBCONFIG_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Get library path
find_library(LIBCONFIG_LIBRARIES
    NAMES "libconfig++"
    HINTS ${LIBCONFIG_ROOT}
    PATH_SUFFIXES "lib"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBCONFIG
    REQUIRED_VARS 
        LIBCONFIG_LIBRARIES 
        LIBCONFIG_INCLUDE_DIRS
)
mark_as_advanced(
    LIBCONFIG_FOUND
    LIBCONFIG_LIBRARIES 
    LIBCONFIG_INCLUDE_DIRS
)