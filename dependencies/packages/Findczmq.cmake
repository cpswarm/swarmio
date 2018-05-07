# Get include path
find_path(CZMQ_INCLUDE_DIRS
    NAMES "czmq.h"
    HINTS "${CMAKE_PREFIX_PATH}/include"
)

# Get library path
find_library(CZMQ_LIBRARIES
    NAMES "czmq"
    HINTS "${CMAKE_PREFIX_PATH}/lib"
)

# Get DLL path
find_file(CZMQ_DLL
    NAMES 
        "libczmq.dll"
        "libczmq.so" 
        "libczmq.dylib" 
    HINTS 
        "${CMAKE_PREFIX_PATH}/bin"
        "${CMAKE_PREFIX_PATH}/lib"
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CZMQ
    REQUIRED_VARS 
        CZMQ_LIBRARIES 
        CZMQ_INCLUDE_DIRS
        CZMQ_DLL
)
mark_as_advanced(
    CZMQ_FOUND
    CZMQ_LIBRARIES 
    CZMQ_INCLUDE_DIRS
    CZMQ_DLL
)