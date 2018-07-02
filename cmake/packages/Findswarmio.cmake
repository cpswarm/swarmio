# Get include path
find_path(SWARMIO_INCLUDE_DIRS
    NAMES "swarmio/Node.h"
    HINTS ${LIBSWARMIO_ROOT}
    PATH_SUFFIXES "include"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Get library path
find_library(SWARMIO_LIBRARIES
    NAMES "swarmio"
    HINTS ${LIBSWARMIO_ROOT}
    PATH_SUFFIXES "lib"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
)

# Set required variables
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SWARMIO
    REQUIRED_VARS 
        SWARMIO_LIBRARIES 
        SWARMIO_INCLUDE_DIRS
)
mark_as_advanced(
    SWARMIO_FOUND
    SWARMIO_LIBRARIES 
    SWARMIO_INCLUDE_DIRS
)

# Find dependencies
find_package(libzmq REQUIRED)
find_package(czmq REQUIRED)
find_package(zyre REQUIRED)
find_package(protobuf REQUIRED)
find_package(g3log REQUIRED)
find_package(replxx REQUIRED)
find_package(readerwriterqueue REQUIRED)
find_package(concurrentqueue REQUIRED)
find_package(Threads REQUIRED)

# Build target
add_library(swarmio SHARED IMPORTED)

# Set library location
set_target_properties(swarmio PROPERTIES IMPORTED_LOCATION "${SWARMIO_LIBRARIES}")

# Link dependencies
set(SWARMIO_INTERFACE_LINK
    ${PROTOBUF_LIBRARIES}
    ${LIBZMQ_LIBRARIES}
    ${CZMQ_LIBRARIES}
    ${ZYRE_LIBRARIES}
    ${G3LOG_LIBRARIES}
    Threads::Threads
)
set_target_properties(swarmio PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES "${SWARMIO_INTERFACE_LINK}")

# Add library include paths
set(SWARMIO_INTERFACE_INCLUDE
    ${SWARMIO_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${LIBZMQ_INCLUDE_DIRS}
    ${CZMQ_INCLUDE_DIRS}
    ${ZYRE_INCLUDE_DIRS}
    ${G3LOG_INCLUDE_DIRS}
    ${READERWRITERQUEUE_INCLUDE_DIRS}
    ${CONCURRENTQUEUE_INCLUDE_DIRS}
)
set_target_properties(swarmio PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${SWARMIO_INTERFACE_INCLUDE}")

# Ignore protobuf related warnings
if (MSVC)
    set(SWARMIO_COMPILE_OPTIONS
        "/wd4251"
        "/wd4996"
        "/wd4275"
        "/wd4661"
    )
    set_target_properties(swarmio PROPERTIES INTERFACE_COMPILE_OPTIONS "${SWARMIO_COMPILE_OPTIONS}")
endif()

# Add export/import macros
if (MSVC)
    set(SWARMIO_COMPILE_DEFINITIONS
		"SWARMIO_API=__declspec(dllimport)"
		"PROTOBUF_USE_DLLS"
	)
else()
    set(SWARMIO_COMPILE_DEFINITIONS
		"SWARMIO_API="
		"PROTOBUF_USE_DLLS"
	)
endif()
set_target_properties(swarmio PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${SWARMIO_COMPILE_DEFINITIONS}")