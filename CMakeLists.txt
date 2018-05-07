# Set minimum required version and baseline policy
cmake_minimum_required(VERSION 3.5)

# Set project name
project(swarmio)

# If not explicitly specified, provide a sane default for the build type
if (NOT CMAKE_BUILD_TYPE)
    if (EXISTS "${CMAKE_SOURCE_DIR}/.git")
        set (CMAKE_BUILD_TYPE "Debug")
    else ()
        set (CMAKE_BUILD_TYPE "Release")
    endif()
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Use custom package find scripts
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/packages")

# Use the include directory of the dependencies as a system location
include_directories(SYSTEM "${CMAKE_PREFIX_PATH}/include")

# Use GNUInstallDirs for determining the correct install locations
include(GNUInstallDirs)

# Find dependencies
find_package(g3log REQUIRED)
find_package(libzmq REQUIRED)
find_package(czmq REQUIRED)
find_package(zyre REQUIRED)
find_package(protobuf REQUIRED)
find_package(replxx REQUIRED)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Build library
add_subdirectory("libswarmio")

# Build simulator
add_subdirectory("swarmio-simulator")

# Build tool
add_subdirectory("swarmio-tool")

# Copy DLLs for debugging
if (ENABLE_TARGET_DEBUGGING)

    # Build list of dependencies
    set(TARGET_DLL_DEPENDENCIES 
        ${LIBZMQ_DLL}
        ${CZMQ_DLL}
        ${ZYRE_DLL}
        ${G3LOG_DLL}
        "${CMAKE_PREFIX_PATH}/bin/libprotobufd.dll"
        "$<TARGET_FILE_DIR:libswarmio>/libswarmio.dll"
    )

    # Copy files
    add_custom_target(target-debugging ALL
        COMMAND ${CMAKE_COMMAND} -E copy ${TARGET_DLL_DEPENDENCIES} "$<TARGET_FILE_DIR:swarmio-tool>"
        COMMAND ${CMAKE_COMMAND} -E copy ${TARGET_DLL_DEPENDENCIES} "$<TARGET_FILE_DIR:swarmio-simulator>"
        DEPENDS swarmio-simulator swarmio-tool
        COMMENT "Copying DLL files to the output directory of the output executables"
    )
    
endif()