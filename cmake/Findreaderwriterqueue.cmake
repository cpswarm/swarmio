# Get include path
find_path(READERWRITERQUEUE_INCLUDE_DIRS
    NAMES "readerwriterqueue.h"
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
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
