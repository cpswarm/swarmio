# Get include path
find_path(CONCURRENTQUEUE_INCLUDE_DIRS
    NAMES "concurrentqueue.h"
    HINTS "${CMAKE_PREFIX_PATH}/include"
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
