# Determine host and target architecture
if (MSVC)

    # Check toolset version
    message(STATUS "Visual C++ toolset version: ${MSVC_TOOLSET_VERSION}")

    # Detect host architecture if not defined
    if (NOT DEFINED SWARMIO_HOST_ARCHITECTURE)
        if ("${CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE}" STREQUAL "x64")
            set(SWARMIO_HOST_ARCHITECTURE "x64")
            list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/windows-msvc-x64.cmake")
        elseif ("${CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE}" STREQUAL "")
            set(SWARMIO_HOST_ARCHITECTURE "x86")
            list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/windows-msvc-x86.cmake")
        else()
            message(FATAL_ERROR "Unknown host architecture: ${CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE}")
        endif()
    endif()

    # Default to host architecture if not defined
    if (NOT DEFINED SWARMIO_TARGET_ARCHITECTURE)
        set(SWARMIO_TARGET_ARCHITECTURE ${SWARMIO_HOST_ARCHITECTURE})
    endif()

    # Set toolchain
    if (SWARMIO_TARGET_ARCHITECTURE STREQUAL "x64")
        list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/windows-msvc-x64.cmake")
    elseif (SWARMIO_TARGET_ARCHITECTURE STREQUAL "x86")
        list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/windows-msvc-x86.cmake")
    else()
        message(FATAL_ERROR "Unknown target architecture: ${SWARMIO_TARGET_ARCHITECTURE}")
    endif()

else()

    # Detect host architecture
    find_program(TOOL_DPKG "dpkg")
    if (TOOL_DPKG STREQUAL "TOOL_DPKG_NOT_FOUND")

        # Use "native" as the name of the host architecture
        message(STATUS "It seems that dpkg is not available on your system, host architecture won't be detected.")
        set(SWARMIO_HOST_ARCHITECTURE "native")

    else()

        # Get host architecture from dpkg
        execute_process(
            COMMAND ${TOOL_DPKG} --print-architecture
            OUTPUT_VARIABLE SWARMIO_HOST_ARCHITECTURE
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

    endif()

    # Default to host architecture
    if (NOT DEFINED SWARMIO_TARGET_ARCHITECTURE)
        set(SWARMIO_TARGET_ARCHITECTURE ${SWARMIO_HOST_ARCHITECTURE})
    endif()

    # Determine toolchain based on preference
    if (SWARMIO_PREFER_GCC)
        set(SWARMIO_TOOLCHAIN "linux-gcc")
    else()
        set(SWARMIO_TOOLCHAIN "linux-clang")
    endif()

    # Add to host subproject args
    list(APPEND SWARMIO_SUBPROJECT_HOST_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/${SWARMIO_TOOLCHAIN}.cmake")

    # Determine architecture for toolchain
    if (SWARMIO_TARGET_ARCHITECTURE STREQUAL "armhf")
        set(SWARMIO_TOOLCHAIN "${SWARMIO_TOOLCHAIN}-armhf")
    elseif (SWARMIO_TARGET_ARCHITECTURE STREQUAL "arm64")
        set(SWARMIO_TOOLCHAIN "${SWARMIO_TOOLCHAIN}-arm64")
    elseif (SWARMIO_TARGET_ARCHITECTURE STREQUAL "amd64")
        set(SWARMIO_TOOLCHAIN "${SWARMIO_TOOLCHAIN}-amd64")
    elseif (NOT SWARMIO_TARGET_ARCHITECTURE STREQUAL "native")
        message(FATAL_ERROR "Unknown target architecture: ${SWARMIO_TARGET_ARCHITECTURE}")
    endif()

    # Add to target subproject args
    list(APPEND SWARMIO_SUBPROJECT_TARGET_ARGS "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchains/${SWARMIO_TOOLCHAIN}.cmake")
	
endif()

# Show architecture
message(STATUS "Host architecture: ${SWARMIO_HOST_ARCHITECTURE}")
message(STATUS "Target architecture: ${SWARMIO_TARGET_ARCHITECTURE}")