# Tools
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

# Set target architecture
if (DEFINED SWARMIO_TARGET_ARCHITECTURE)
    set(CMAKE_C_COMPILER_TARGET ${SWARMIO_TARGET_ARCHITECTURE})
    set(CMAKE_CXX_COMPILER_TARGET ${SWARMIO_TARGET_ARCHITECTURE})
endif()

# If a GCC version was explicitly specified, set toolchain directory
if (NOT "${SWARMIO_GCC_VERSION}" STREQUAL "")

    # Determine toolchain root
    set(CLANG_GCC_TOOLCHAIN "${SWARMIO_SYSROOT}/usr/lib/gcc/${SWARMIO_TARGET_ARCHITECTURE}/${SWARMIO_GCC_VERSION}")
    set(CLANG_GCC_LIBDIR "${SWARMIO_SYSROOT}/usr/lib/gcc/${SWARMIO_TARGET_ARCHITECTURE}/${SWARMIO_GCC_VERSION}")
                                            
    # Set linker flags
    set(CMAKE_EXE_LINKER_FLAGS "--gcc-toolchain=${CLANG_GCC_TOOLCHAIN} -L${CLANG_GCC_LIBDIR} -B${CLANG_GCC_LIBDIR}")
    set(CMAKE_SHARED_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS})

    # Set compiler flags
    add_definitions(
        "--gcc-toolchain=${CLANG_GCC_TOOLCHAIN}"
    )

    # Add include directories
    include_directories(SYSTEM 
        "${SWARMIO_SYSROOT}/usr/include/c++/${SWARMIO_GCC_VERSION}"
        "${SWARMIO_SYSROOT}/usr/include/${SWARMIO_TARGET_ARCHITECTURE}/c++/${SWARMIO_GCC_VERSION}"
    )

endif()

# Include platform definition
include("${CMAKE_CURRENT_LIST_DIR}/linux.cmake")