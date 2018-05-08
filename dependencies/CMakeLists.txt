# Set minimum required version and baseline policy
cmake_minimum_required(VERSION 3.5)

# Declare project
project(swarmio-dependencies)

# Use ExternalProject for the superbuild
include(ExternalProject)

# If not explicitly specified, provide a sane default for the build type
if (NOT CMAKE_BUILD_TYPE)
    if (EXISTS "${CMAKE_SOURCE_DIR}/../.git")
        set (CMAKE_BUILD_TYPE "Debug")
    else ()
        set (CMAKE_BUILD_TYPE "Release")
    endif()
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Assemble common arguments
set(DEPENDENCIES_CMAKE_ARGS 
    "-DCMAKE_CONFIGURATION_TYPES=${CMAKE_BUILD_TYPE}"
    "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
    "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
    "-DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}"
    "-DCMAKE_MODULE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/packages"
)

if (MSVC)

	# Build g3log
	ExternalProject_Add(g3log
	    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/g3log" 
	    INSTALL_COMMAND ""
	    CMAKE_ARGS 
		${DEPENDENCIES_CMAKE_ARGS}
		"-DCHANGE_G3LOG_DEBUG_TO_DBUG=ON"
	)

	# Copy g3log files
	ExternalProject_Add_Step(g3log copy-files 
	    WORKING_DIRECTORY "<BINARY_DIR>/${CMAKE_BUILD_TYPE}"
	    COMMAND ${CMAKE_COMMAND} -E copy "g3logger.dll" "g3logger.pdb" "${CMAKE_INSTALL_PREFIX}/bin"
	    COMMAND ${CMAKE_COMMAND} -E copy "g3logger.lib" "${CMAKE_INSTALL_PREFIX}/lib"
	    COMMAND ${CMAKE_COMMAND} -E copy_directory "<SOURCE_DIR>/src/g3log" "${CMAKE_INSTALL_PREFIX}/include/g3log"
	    COMMAND ${CMAKE_COMMAND} -E copy "<BINARY_DIR>/include/g3log/generated_definitions.hpp" "${CMAKE_INSTALL_PREFIX}/include/g3log"
	    DEPENDEES build
	    ALWAYS TRUE
	)

else()

	# Build g3log
	ExternalProject_Add(g3log
	    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/g3log" 
	    CMAKE_ARGS 
		${DEPENDENCIES_CMAKE_ARGS}
		"-DCHANGE_G3LOG_DEBUG_TO_DBUG=ON"
	)

endif()

# Build replxx
ExternalProject_Add(replxx
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/replxx" 
    CMAKE_ARGS 
        ${DEPENDENCIES_CMAKE_ARGS}
)

# Build ZeroMQ
ExternalProject_Add(libzmq
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libzmq"
    CMAKE_ARGS 
        ${DEPENDENCIES_CMAKE_ARGS}
        "-DWITH_PERF_TOOL=OFF" 
        "-DZMQ_BUILD_TESTS=OFF" 
        "-DENABLE_CPACK=OFF"
)

# Build CZMQ
ExternalProject_Add(czmq
    DEPENDS libzmq
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/czmq" 
    CMAKE_ARGS 
        ${DEPENDENCIES_CMAKE_ARGS}
)

# Copy PDB file
if (MSVC)
    ExternalProject_Add_Step(czmq copy-pdb 
        WORKING_DIRECTORY "<BINARY_DIR>/${CMAKE_BUILD_TYPE}"
        COMMAND ${CMAKE_COMMAND} -E copy "libczmq.pdb" "${CMAKE_INSTALL_PREFIX}/bin"
        DEPENDEES install
        ALWAYS TRUE
    )
endif()

# Build Zyre
ExternalProject_Add(zyre
    DEPENDS czmq
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/zyre"
    CMAKE_ARGS 
        ${DEPENDENCIES_CMAKE_ARGS}
)

# Copy PDB file
if (MSVC)
    ExternalProject_Add_Step(zyre copy-pdb 
        WORKING_DIRECTORY "<BINARY_DIR>/${CMAKE_BUILD_TYPE}"
        COMMAND ${CMAKE_COMMAND} -E copy "zyre.pdb" "${CMAKE_INSTALL_PREFIX}/bin"
        DEPENDEES install
        ALWAYS TRUE
    )
endif()

# Build Protobuf
ExternalProject_Add(protobuf
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/protobuf"
    SOURCE_SUBDIR "cmake"
    CMAKE_ARGS 
        ${DEPENDENCIES_CMAKE_ARGS}
        "-Dprotobuf_BUILD_TESTS=OFF"
        "-Dprotobuf_BUILD_EXAMPLES=OFF"
        "-Dprotobuf_BUILD_SHARED_LIBS=ON"
        "-Dprotobuf_MSVC_STATIC_RUNTIME=OFF"
)

# Copy PDB file
if (MSVC)
    ExternalProject_Add_Step(protobuf copy-pdb 
        WORKING_DIRECTORY "<BINARY_DIR>/${CMAKE_BUILD_TYPE}"
        COMMAND ${CMAKE_COMMAND} -E copy "libprotobufd.pdb" "libprotobuf-lited.pdb" "libprotocd.pdb" "protoc.pdb" "${CMAKE_INSTALL_PREFIX}/bin"
        DEPENDEES install
        ALWAYS TRUE
    )
endif()

# Copy headers for the concurrentqueue library
add_custom_target(queues ALL
    COMMAND ${CMAKE_COMMAND} -E copy 
        "${CMAKE_CURRENT_SOURCE_DIR}/concurrentqueue/concurrentqueue.h" 
        "${CMAKE_CURRENT_SOURCE_DIR}/concurrentqueue/blockingconcurrentqueue.h" 
        "${CMAKE_CURRENT_SOURCE_DIR}/readerwriterqueue/readerwriterqueue.h" 
        "${CMAKE_CURRENT_SOURCE_DIR}/readerwriterqueue/atomicops.h" 
        "${CMAKE_INSTALL_PREFIX}/include"
)
