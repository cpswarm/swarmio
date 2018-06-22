# Get include path
find_path(CONCURRENTQUEUE_INCLUDE_DIRS
    NAMES "concurrentqueue.h"
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CONCURRENTQUEUE
    REQUIRED_VARS 
        CONCURRENTQUEUE_INCLUDE_DIRS
)
mark_as_advanced(
    CONCURRENTQUEUE_FOUND
    CONCURRENTQUEUE_INCLUDE_DIRS
)
