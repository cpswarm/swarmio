# Are we building in development, install or package mode?
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE" OR SWARMIO_BUILD_MODE STREQUAL "INSTALL")

    # Limit to UNIX-like systems
    if (NOT UNIX)
        message(FATAL_ERROR "INSTALL and PACKAGE build modes are only available on UNIX-like systems.")
    endif()

    # Default is to build using a RelWithDebInfo configuration
	if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE "RelWithDebInfo")
    endif()

    # Default install location is /opt/swarmio
    if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "/opt/swarmio" CACHE PATH "Install prefix" FORCE)
    endif()

    # Must be absolute path
    if (NOT CMAKE_INSTALL_PREFIX MATCHES "\/.+")
        message(FATAL_ERROR "CMAKE_INSTALL_PREFIX must be an absolute path for INSTALL or PACKAGE build modes.")
    endif()

    # Show mode
    if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
        message(STATUS "Mode: build DEB packages from project outputs with root ${CMAKE_INSTALL_PREFIX}")
    else()
        message(STATUS "Mode: install all project outputs to ${CMAKE_INSTALL_PREFIX}")
    endif()

else()

    # Default to Development mode
    if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
	
		# Warn if no build mode was specified
		if (UNIX)
			message(STATUS "SWARMIO_BUILD_MODE is not specified or invalid, defaulting to DEVELOPMENT.")
		endif()
		
		# Set mode
        set(SWARMIO_BUILD_MODE "DEVELOPMENT")
		
    endif()

    # Default is to build using a Debug configuration
	if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE "Debug")
    endif()

    # Install into a common directory
    if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/devroot-${SWARMIO_TARGET_ARCHITECTURE}" CACHE PATH "Install prefix" FORCE)
    endif()

    # Show mode
	if (MSVC)
		message(STATUS "Mode: install everything to ${CMAKE_INSTALL_PREFIX}")
	else()
		message(STATUS "Mode: install dependencies to ${CMAKE_INSTALL_PREFIX}, keep core project outputs separate")
	endif()
	
endif()

# Pass paths to subprojects
list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS
    "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
    "-DCMAKE_INSTALL_RPATH=${CMAKE_INSTALL_PREFIX}/lib"
)

# Pass build type specification onto subprojects
list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS
    "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
)

# Tools should be built in release mode
list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo"
)

# Set build commands
if (MSVC)

    # Since MSVC is a multi-config generator, we need to modify the build command
    set(SWARMIO_HOST_BUILD_COMMAND cmake --build . --config RelWithDebInfo)
    set(SWARMIO_TARGET_BUILD_COMMAND cmake --build . --config ${CMAKE_BUILD_TYPE})

else()

    # Just use the default build commands
    set(SWARMIO_HOST_BUILD_COMMAND cmake --build .)
    set(SWARMIO_TARGET_BUILD_COMMAND cmake --build .)

endif()

# Set install commands
set(SWARMIO_HOST_INSTALL_COMMAND ${SWARMIO_HOST_BUILD_COMMAND} --target install)
set(SWARMIO_TARGET_INSTALL_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND} --target install)

# Show build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")