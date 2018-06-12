# Set minimum required version and baseline policy
cmake_minimum_required(VERSION 3.5)

# Set project name
project(swarmio VERSION 0.3)

# Use ExternalProject for the superbuild
include(ExternalProject)

# Define common arguments for projects built for the host architecture
set (SWARMIO_SUBPROJECT_HOST_ARGS
    "-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>"
)

# Define common arguments for projects built for the target architecture
set (SWARMIO_SUBPROJECT_TARGET_ARGS
    "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
    "-DCMAKE_INSTALL_RPATH=${CMAKE_INSTALL_PREFIX}/lib"
    "-DCMAKE_STAGING_PREFIX=<INSTALL_DIR>"
	"-DCMAKE_MODULE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/cmake"
)

# Get host package architecture
if (MSVC)
	
	# Set target platform
	if(SWARMIO_TARGET_ARCHITECTURE STREQUAL "x64" OR CMAKE_GENERATOR_PLATFORM STREQUAL "x64")
		set(SWARMIO_TARGET_ARCHITECTURE "x64")
		list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS 
			"-GVisual Studio 15 2017"
			"-DCMAKE_GENERATOR_PLATFORM=x64"
		)
	elseif (SWARMIO_TARGET_ARCHITECTURE STREQUAL "x86" OR NOT DEFINED SWARMIO_TARGET_ARCHITECTURE)
		set(SWARMIO_TARGET_ARCHITECTURE "x86")
		list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS 
			"-GVisual Studio 15 2017"
		)
	else()
		message(FATAL_ERROR "Unknown target architecture specified")
	endif()
	
	# Set host architecture
	if (SWARMIO_HOST_ARCHITECTURE STREQUAL "x64" OR CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(SWARMIO_HOST_ARCHITECTURE "x64")
		list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS 
			"-GVisual Studio 15 2017"
			"-DCMAKE_GENERATOR_PLATFORM=x64"
			"-Tv141,host=x64"
		)
		list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS 
			"-Tv141,host=x64"
		)
	elseif (SWARMIO_HOST_ARCHITECTURE STREQUAL "x86" OR NOT DEFINED SWARMIO_HOST_ARCHITECTURE)
		set(SWARMIO_HOST_ARCHITECTURE "x86")
		list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS 
			"-GVisual Studio 15 2017"
			"-Tv141"
		)
		list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS 
			"-Tv141"
		)
	else()
		message(FATAL_ERROR "Unknown host architecture specified")
	endif()
	
else()

	# Get host architecture
	execute_process(
		COMMAND dpkg --print-architecture
		OUTPUT_VARIABLE SWARMIO_HOST_ARCHITECTURE
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	
	# Apply architecture
	if (DEFINED SWARMIO_TARGET_ARCHITECTURE)

		# Check support for target architecture
		if (SWARMIO_TARGET_ARCHITECTURE STREQUAL "armhf")
			list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-linux-armhf.cmake")
		elseif (SWARMIO_TARGET_ARCHITECTURE STREQUAL "arm64")
			list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain-linux-arm64.cmake")
		else()
			message(FATAL_ERROR "Unknown target architecture specified")
		endif()

	else()

		# Use host architecture
		set(SWARMIO_TARGET_ARCHITECTURE ${SWARMIO_HOST_ARCHITECTURE})

	endif()
	
endif()

# Show architecture
message(STATUS "Host architecture: ${SWARMIO_HOST_ARCHITECTURE}")
message(STATUS "Target architecture: ${SWARMIO_TARGET_ARCHITECTURE}")

# Are we building in development, install or package mode?
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE" OR SWARMIO_BUILD_MODE STREQUAL "INSTALL")

    # Limit to UNIX-like systems
    if (NOT UNIX)
        message(FATAL_ERROR "INSTALL and PACKAGE build modes are only available on UNIX-like systems.")
    endif()

    # Check for the presence of dpkg-deb
    if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
        find_program(TOOL_DPKG_DEB "dpkg-deb")
        if (TOOL_DPKG_DEB STREQUAL "TOOL_DPGK_DEB_NOT_FOUND")
            message(FATAL_ERROR "It seems that dpkg-deb is not available on your system, install it before trying to build packages.")
        endif()
        find_program(TOOL_FAKEROOT "fakeroot")
        if (TOOL_FAKEROOT STREQUAL "TOOL_FAKEROOT_NOT_FOUND")
            message(FATAL_ERROR "It seems that fakeroot is not available on your system, install it before trying to build packages.")
        endif()
        find_program(TOOL_SED "sed")
        if (TOOL_SED STREQUAL "TOOL_SED_NOT_FOUND")
            message(FATAL_ERROR "It seems that sed is not available on your system, install it before trying to build packages.")
        endif()
    endif()

    # Default install location is /opt
    if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "/opt" CACHE PATH "Install prefix" FORCE)
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
			message(WARNING "SWARMIO_BUILD_MODE is not specified or invalid, defaulting to DEVELOPMENT.")
		endif()
		
		# Set mode
        set(SWARMIO_BUILD_MODE "DEVELOPMENT")
		
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

# If using MSVC, pass build type specification onto subprojects
if (MSVC)
	
	# Default is to build using a Debug configuration
	if (NOT DEFINED CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE "Debug")
	endif()

	# Add to target arguments
	list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS
		"-DCMAKE_CONFIGURATION_TYPES=${CMAKE_BUILD_TYPE}"
		"-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
		"--config ${CMAKE_BUILD_TYPE}"
	)
	
	# Host arguments are always in release mode
	list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS
		"-DCMAKE_CONFIGURATION_TYPES=RelWithDebInfo"
		"-DCMAKE_BUILD_TYPE=RelWithDebInfo"
		"--config RelWithDebInfo"
	)
	
	# Show build type
	message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
	
endif()

# Define a function to build packages
function(ExternalProject_Build_Package TARGET PACKAGE_VERSION PACKAGE_ARCH)

    # Build package name
    string(FIND "${TARGET}" "swarmio-" _position)
    if (_position EQUAL 0)
        set(PACKAGE_NAME ${TARGET})
    else()
        set(PACKAGE_NAME "swarmio-${TARGET}")
    endif()

    # Build output paths
    set(PACKAGE_CONTROL_FILE "debian/${PACKAGE_NAME}-control")
    set(PACKAGE_OUTPUT_FILE "packages/${PACKAGE_NAME}-${SWARMIO_TARGET_ARCHITECTURE}.deb")

    # Build dependency list
    if (ARGC GREATER 3)
        string(REPLACE ";" ", " PACKAGE_DEPENDS "${ARGN}")
    endif()

    # Generate package descriptor
    configure_file("cmake/control.tpl" ${PACKAGE_CONTROL_FILE})

    # Add package generation step
    ExternalProject_Add_Step(${TARGET} generate-deb
        DEPENDEES install
        BYPRODUCTS ${PACKAGE_OUTPUT_FILE}
        COMMAND ${CMAKE_COMMAND} -E remove_directory "<TMP_DIR>/package"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "<INSTALL_DIR>" "<TMP_DIR>/package${CMAKE_INSTALL_PREFIX}"
        COMMAND ${CMAKE_COMMAND} -E remove_directory "<TMP_DIR>/package${CMAKE_INSTALL_PREFIX}/include"
        COMMAND ${CMAKE_COMMAND} -E copy ${PACKAGE_CONTROL_FILE} "<TMP_DIR>/package/DEBIAN/control"
        COMMAND ${TOOL_FAKEROOT} ${TOOL_DPKG_DEB} --build  "<TMP_DIR>/package" 
        COMMAND ${CMAKE_COMMAND} -E copy "<TMP_DIR>/package.deb" ${PACKAGE_OUTPUT_FILE}
    )

endfunction()

# Set libzmq install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(LIBZMQ_DIR "${CMAKE_CURRENT_BINARY_DIR}/libzmq-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(LIBZMQ_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build libzmq
ExternalProject_Add(libzmq
    GIT_REPOSITORY "https://github.com/zeromq/libzmq.git"
    GIT_TAG "3a1e8f070416c11481b847ac752c0a73ea4bc381"
    GIT_SHALLOW ON
    PREFIX "libzmq-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${LIBZMQ_DIR}
    CMAKE_ARGS 
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DWITH_PERF_TOOL=OFF" 
        "-DZMQ_BUILD_TESTS=OFF" 
)

# Build libzmq package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(libzmq "4.2.5-dev" ${SWARMIO_TARGET_ARCHITECTURE})
endif()

# Set czmq install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(CZMQ_DIR "${CMAKE_CURRENT_BINARY_DIR}/czmq-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(CZMQ_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build czmq
ExternalProject_Add(czmq 
    DEPENDS libzmq
    GIT_REPOSITORY "https://github.com/zeromq/czmq.git"
    GIT_TAG "e305dc2136635cfb38a05e090580a833da54de28"
    GIT_SHALLOW ON
    PREFIX "czmq-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${CZMQ_DIR}
    CMAKE_ARGS  
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DCMAKE_PREFIX_PATH=${LIBZMQ_DIR}"
)

# Build czmq package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(czmq "4.1.1" ${SWARMIO_TARGET_ARCHITECTURE} "swarmio-libzmq")
endif()

# Set zyre install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(ZYRE_DIR "${CMAKE_CURRENT_BINARY_DIR}/zyre-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(ZYRE_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build zyre
ExternalProject_Add(zyre 
    DEPENDS czmq libzmq
    GIT_REPOSITORY "https://github.com/zeromq/zyre.git"
    GIT_TAG "ab263dac21250fc85fa8c7239973109eca2251a7"
    GIT_SHALLOW ON
    PREFIX "zyre-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${ZYRE_DIR}
    LIST_SEPARATOR |
    CMAKE_ARGS  
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DCMAKE_PREFIX_PATH=${LIBZMQ_DIR}|${CZMQ_DIR}"
)

# Build zyre package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(zyre "2.0.0" ${SWARMIO_TARGET_ARCHITECTURE} "swarmio-libzmq" "swarmio-czmq")
endif()

# Set replxx install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(REPLXX_DIR "${CMAKE_CURRENT_BINARY_DIR}/replxx-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(REPLXX_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build replxx
ExternalProject_Add(replxx 
    GIT_REPOSITORY "https://github.com/AmokHuginnsson/replxx.git"
    GIT_TAG "3cb884e3fb4b1a28efeb716fac75f77eecc7ea3d"
    GIT_SHALLOW ON
    PREFIX "replxx-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${REPLXX_DIR}
    CMAKE_ARGS  
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
)

# Build replxx package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(replxx "0.0.1" ${SWARMIO_TARGET_ARCHITECTURE})
endif()

# Set protobuf install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(PROTOBUF_DIR "${CMAKE_CURRENT_BINARY_DIR}/protobuf-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(PROTOBUF_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# On Linux, in order to support cross-compilation, built the protoc binary separately
if (MSVC)

	# Build protobuf libraries for target architecture
	ExternalProject_Add(protobuf
		GIT_REPOSITORY "https://github.com/google/protobuf.git"
		GIT_TAG "ce044817c7ba0aea27c3fd8e496635d94d20a755"
		GIT_SHALLOW ON
		SOURCE_SUBDIR "cmake"
		PREFIX "protobuf-${SWARMIO_TARGET_ARCHITECTURE}"
		INSTALL_DIR ${PROTOBUF_DIR}
		CMAKE_ARGS  
			${SWARMIO_SUBPROJECT_TARGET_ARGS}
			"-Dprotobuf_BUILD_TESTS=OFF"
			"-Dprotobuf_BUILD_EXAMPLES=OFF"
			"-Dprotobuf_BUILD_SHARED_LIBS=ON"
	)
	
	# Get protoc location
	ExternalProject_Get_Property(protobuf INSTALL_DIR)
	set(TOOL_PROTOC "${INSTALL_DIR}/bin/protoc")

else()

	# Build protobuf libraries for target architecture
	ExternalProject_Add(protobuf
		GIT_REPOSITORY "https://github.com/google/protobuf.git"
		GIT_TAG "ce044817c7ba0aea27c3fd8e496635d94d20a755"
		GIT_SHALLOW ON
		SOURCE_SUBDIR "cmake"
		PREFIX "protobuf-${SWARMIO_TARGET_ARCHITECTURE}"
		INSTALL_DIR ${PROTOBUF_DIR}
		CMAKE_ARGS  
			${SWARMIO_SUBPROJECT_TARGET_ARGS}
			"-Dprotobuf_BUILD_TESTS=OFF"
			"-Dprotobuf_BUILD_EXAMPLES=OFF"
			"-Dprotobuf_BUILD_SHARED_LIBS=ON"
			"-Dprotobuf_BUILD_PROTOC_BINARIES=OFF"
	)

	# Build protobuf binaries for host architecture
	ExternalProject_Add(protobuf-host
		GIT_REPOSITORY "https://github.com/google/protobuf.git"
		GIT_TAG "ce044817c7ba0aea27c3fd8e496635d94d20a755"
		GIT_SHALLOW ON
		SOURCE_SUBDIR "cmake"
		PREFIX "protobuf-host-${SWARMIO_HOST_ARCHITECTURE}"
		INSTALL_DIR "protobuf-host-${SWARMIO_HOST_ARCHITECTURE}-output"
		CMAKE_ARGS  
			${SWARMIO_SUBPROJECT_HOST_ARGS}
			"-Dprotobuf_BUILD_TESTS=OFF"
			"-Dprotobuf_BUILD_EXAMPLES=OFF"
	)

	# Get protoc location
	ExternalProject_Get_Property(protobuf-host INSTALL_DIR)
	set(TOOL_PROTOC "${INSTALL_DIR}/bin/protoc")

endif()

# Build protobuf package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(protobuf "3.6.0.1" ${SWARMIO_TARGET_ARCHITECTURE})
endif()

# Set g3log install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(G3LOG_DIR "${CMAKE_CURRENT_BINARY_DIR}/g3log-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(G3LOG_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build g3log
if (MSVC)

	# Build using CMake and install manually
	ExternalProject_Add(g3log
		GIT_REPOSITORY "https://github.com/KjellKod/g3log.git"
		GIT_TAG "e988aadc6572769809eaf962cab1824eed62086e"
		GIT_SHALLOW ON
		PREFIX "g3log-${SWARMIO_TARGET_ARCHITECTURE}"
		INSTALL_DIR ${G3LOG_DIR}
		INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory  "<INSTALL_DIR>/bin"
		COMMAND ${CMAKE_COMMAND} -E make_directory  "<INSTALL_DIR>/lib"
		COMMAND ${CMAKE_COMMAND} -E make_directory  "<INSTALL_DIR>/include"
		COMMAND ${CMAKE_COMMAND} -E copy  "<BINARY_DIR>/${CMAKE_BUILD_TYPE}/g3logger.dll" "<INSTALL_DIR>/bin"
	    COMMAND ${CMAKE_COMMAND} -E copy "<BINARY_DIR>/${CMAKE_BUILD_TYPE}/g3logger.lib" "<INSTALL_DIR>/lib"
	    COMMAND ${CMAKE_COMMAND} -E copy_directory "<SOURCE_DIR>/src/g3log" "<INSTALL_DIR>/include/g3log"
	    COMMAND ${CMAKE_COMMAND} -E copy "<BINARY_DIR>/include/g3log/generated_definitions.hpp" "<INSTALL_DIR>/include/g3log"
		CMAKE_ARGS  
			${SWARMIO_SUBPROJECT_TARGET_ARGS}
			"-DCHANGE_G3LOG_DEBUG_TO_DBUG=ON"
			"-DADD_FATAL_EXAMPLE=OFF"
	)

else()
	
	# Build and install using CMake
	ExternalProject_Add(g3log
		GIT_REPOSITORY "https://github.com/KjellKod/g3log.git"
		GIT_TAG "e988aadc6572769809eaf962cab1824eed62086e"
		GIT_SHALLOW ON
		PREFIX "g3log-${SWARMIO_TARGET_ARCHITECTURE}"
		INSTALL_DIR ${G3LOG_DIR}
		CMAKE_ARGS  
			${SWARMIO_SUBPROJECT_TARGET_ARGS}
			"-DCHANGE_G3LOG_DEBUG_TO_DBUG=ON"
			"-DADD_FATAL_EXAMPLE=OFF"
			"-DCPACK_PACKAGING_INSTALL_PREFIX=<INSTALL_DIR>"
	)
	
endif()

# Build g3log package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")

	# Patch g3log to install outputs to the correct location
	ExternalProject_Add_Step(g3log patch-installer
		DEPENDEES update
		DEPENDERS configure
		ALWAYS true
		COMMAND ${TOOL_SED} -i -e "s/CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT/CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND FALSE/g" "<SOURCE_DIR>/CPackLists.txt"
	)

	# Build actual package
	ExternalProject_Build_Package(g3log "1.3.2" ${SWARMIO_TARGET_ARCHITECTURE})

endif()

# Set concurrentqueue install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(CONCURRENTQUEUE_DIR "${CMAKE_CURRENT_BINARY_DIR}/concurrentqueue-output")
else()
    set(CONCURRENTQUEUE_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build concurrentqueue
ExternalProject_Add(concurrentqueue
    GIT_REPOSITORY "https://github.com/cameron314/concurrentqueue.git"
    GIT_TAG "8f7e861dd9411a0bf77a6b9de83a47b3424fafba"
    GIT_SHALLOW ON
    PREFIX "concurrentqueue"
    INSTALL_DIR ${CONCURRENTQUEUE_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory "<INSTALL_DIR>/include"
    COMMAND ${CMAKE_COMMAND} -E copy "<SOURCE_DIR>/blockingconcurrentqueue.h" "<SOURCE_DIR>/concurrentqueue.h" "<INSTALL_DIR>/include"
)

# Set readerwriterqueue install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(READERWRITERQUEUE_DIR "${CMAKE_CURRENT_BINARY_DIR}/readerwriterqueue-output")
else()
    set(READERWRITERQUEUE_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build readerwriterqueue
ExternalProject_Add(readerwriterqueue
    GIT_REPOSITORY "https://github.com/cameron314/readerwriterqueue.git"
    GIT_TAG "07e22ecdf90501df89ead679bb8294a0b7c80c24"
    GIT_SHALLOW ON
    PREFIX "readerwriterqueue"
    INSTALL_DIR ${READERWRITERQUEUE_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory "<INSTALL_DIR>/include"
    COMMAND ${CMAKE_COMMAND} -E copy "<SOURCE_DIR>/readerwriterqueue.h" "<SOURCE_DIR>/atomicops.h" "<INSTALL_DIR>/include"
)

# Set libconfig install dir
if (NOT SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(LIBCONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/libconfig-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(LIBCONFIG_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build libconfig
ExternalProject_Add(libconfig
    GIT_REPOSITORY "https://github.com/hyperrealm/libconfig.git"
    GIT_TAG "9716f6224cdbc9f8bf3c8d3759a9fb3f864155df"
    GIT_SHALLOW ON
    PREFIX "libconfig-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${LIBCONFIG_DIR}
    CMAKE_ARGS 
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DBUILD_TESTS=OFF"
        "-DBUILD_EXAMPLES=OFF"
        "-DBUILD_SHARED_LIBS=ON"
)

# Build libconfig package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(libconfig "1.7.2" ${SWARMIO_TARGET_ARCHITECTURE})
endif()

# Set libswarmio install dir - on Windows, we have to put all binaries together for DLL resolution to work
if (NOT MSVC)
	set(LIBSWARMIO_DIR "${CMAKE_CURRENT_BINARY_DIR}/libswarmio-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(LIBSWARMIO_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build libswarmio
ExternalProject_Add(libswarmio
    DEPENDS libzmq czmq zyre protobuf g3log concurrentqueue readerwriterqueue
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libswarmio"
    PREFIX "libswarmio-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${LIBSWARMIO_DIR}
    LIST_SEPARATOR |
    CMAKE_ARGS  
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DCMAKE_PREFIX_PATH=${LIBZMQ_DIR}|${CZMQ_DIR}|${ZYRE_DIR}|${PROTOBUF_DIR}|${G3LOG_DIR}|${CONCURRENTQUEUE_DIR}|${READERWRITERQUEUE_DIR}"
        "-DTOOL_PROTOC=${TOOL_PROTOC}"
)

# Depend on the host tool on Linux
if (NOT MSVC)
	add_dependencies(libswarmio protobuf-host)
endif()

# Build libswarmio package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(libswarmio ${PROJECT_VERSION} ${SWARMIO_TARGET_ARCHITECTURE} "swarmio-libzmq" "swarmio-czmq" "swarmio-zyre" "swarmio-protobuf" "swarmio-g3log")
endif()

# For DEVELOPMENT builds, add the libswarmio output directory to RPATH
if (SWARMIO_BUILD_MODE STREQUAL "DEVELOPMENT")
    set(SWARMIO_SUBPROJECT_TARGET_ARGS
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DCMAKE_INSTALL_RPATH=${CMAKE_INSTALL_PREFIX}/lib|${LIBSWARMIO_DIR}/lib"
    )
endif()

# Set swarmio-tool install dir - on Windows, we have to put all binaries together for DLL resolution to work
if (NOT MSVC)
	set(SWARMIO_TOOL_DIR "${CMAKE_CURRENT_BINARY_DIR}/swarmio-tool-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(SWARMIO_TOOL_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build swarmio-tool
ExternalProject_Add(swarmio-tool
    DEPENDS libswarmio replxx
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/swarmio-tool"
    PREFIX "swarmio-tool-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${SWARMIO_TOOL_DIR}
    LIST_SEPARATOR |
    CMAKE_ARGS  
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DCMAKE_PREFIX_PATH=${LIBZMQ_DIR}|${CZMQ_DIR}|${ZYRE_DIR}|${PROTOBUF_DIR}|${G3LOG_DIR}|${CONCURRENTQUEUE_DIR}|${READERWRITERQUEUE_DIR}|${REPLXX_DIR}|${LIBSWARMIO_DIR}"
)

# Build swarmio-tool package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(swarmio-tool ${PROJECT_VERSION} ${SWARMIO_TARGET_ARCHITECTURE} "swarmio-libswarmio" "swarmio-replxx")
endif()

# Set swarmio-tool install dir - on Windows, we have to put all binaries together for DLL resolution to work
if (NOT MSVC)
	set(SWARMIO_SIMLUATOR_DIR "${CMAKE_CURRENT_BINARY_DIR}/swarmio-simulator-${SWARMIO_TARGET_ARCHITECTURE}-output")
else()
    set(SWARMIO_SIMLUATOR_DIR ${CMAKE_INSTALL_PREFIX})
endif()

# Build swarmio-simulator
ExternalProject_Add(swarmio-simulator
    DEPENDS libswarmio
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/swarmio-simulator"
    PREFIX "swarmio-simulator-${SWARMIO_TARGET_ARCHITECTURE}"
    INSTALL_DIR ${SWARMIO_SIMLUATOR_DIR}
    LIST_SEPARATOR |
    CMAKE_ARGS  
        ${SWARMIO_SUBPROJECT_TARGET_ARGS}
        "-DCMAKE_PREFIX_PATH=${LIBZMQ_DIR}|${CZMQ_DIR}|${ZYRE_DIR}|${PROTOBUF_DIR}|${G3LOG_DIR}|${CONCURRENTQUEUE_DIR}|${READERWRITERQUEUE_DIR}|${LIBSWARMIO_DIR}"
)

# Build swarmio-simulator package
if (SWARMIO_BUILD_MODE STREQUAL "PACKAGE")
    ExternalProject_Build_Package(swarmio-simulator ${PROJECT_VERSION} ${SWARMIO_TARGET_ARCHITECTURE} "swarmio-libswarmio")
endif()