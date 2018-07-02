# Get include path
find_path(READERWRITERQUEUE_INCLUDE_DIRS
    NAMES "readerwriterqueue.h"
    HINTS ${READERWRITERQUEUE_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(READERWRITERQUEUE
    REQUIRED_VARS 
        READERWRITERQUEUE_INCLUDE_DIRS
)
mark_as_advanced(
    READERWRITERQUEUE_FOUND
    READERWRITERQUEUE_INCLUDE_DIRS
)
