# Get include path
find_path(REPLXX_INCLUDE_DIRS
    NAMES "replxx.h"
    HINTS ${REPLXX_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Determine library name
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set(REPLXX_LIBNAME "replxx-d")
else()
	set(REPLXX_LIBNAME "replxx")
endif()

# Get library path
find_library(REPLXX_LIBRARIES
    NAMES "libreplxx.a" "${REPLXX_LIBNAME}"
    HINTS ${REPLXX_ROOT}
    PATH_SUFFIXES "lib"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
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
