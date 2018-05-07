# Get include path
find_path(G3LOG_INCLUDE_DIRS
    NAMES
        "g3log/g3log.hpp"
    HINTS "${CMAKE_PREFIX_PATH}/include"
)

# Get library path
find_library(G3LOG_LIBRARIES
    NAMES 
        "g3logger"
        "libg3logger"
    HINTS "${CMAKE_PREFIX_PATH}/lib"
)

# Get DLL path
find_file(G3LOG_DLL
    NAMES 
        "g3logger.dll"
    HINTS 
        "${CMAKE_PREFIX_PATH}/bin"
        "${CMAKE_PREFIX_PATH}/lib"
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G3LOG
    REQUIRED_VARS 
        G3LOG_LIBRARIES 
        G3LOG_INCLUDE_DIRS
        G3LOG_DLL
)
mark_as_advanced(
    G3LOG_FOUND
    G3LOG_LIBRARIES 
    G3LOG_INCLUDE_DIRS
    G3LOG_DLL
)