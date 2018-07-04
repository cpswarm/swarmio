# Prepare
swarmio_build_component_start(g3log "1.3.2")

# Find Git executable
find_package(Git REQUIRED)

# Build g3log
if (MSVC)

	# Build using CMake and install manually
	ExternalProject_Add(g3log
		GIT_REPOSITORY "https://github.com/KjellKod/g3log.git"
		GIT_TAG "e988aadc6572769809eaf962cab1824eed62086e"
		PREFIX "g3log-${SWARMIO_TARGET_ARCHITECTURE}"
		BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
		INSTALL_DIR ${G3LOG_ROOT}
		INSTALL_COMMAND ${CMAKE_COMMAND} -E make_directory  "<INSTALL_DIR>/bin"
				COMMAND ${CMAKE_COMMAND} -E make_directory  "<INSTALL_DIR>/lib"
				COMMAND ${CMAKE_COMMAND} -E make_directory  "<INSTALL_DIR>/include"
				COMMAND ${CMAKE_COMMAND} -E copy  "<BINARY_DIR>/${CMAKE_BUILD_TYPE}/g3logger.dll" "<INSTALL_DIR>/bin"
				COMMAND ${CMAKE_COMMAND} -E copy "<BINARY_DIR>/${CMAKE_BUILD_TYPE}/g3logger.lib" "<INSTALL_DIR>/lib"
				COMMAND ${CMAKE_COMMAND} -E copy_directory "<SOURCE_DIR>/src/g3log" "<INSTALL_DIR>/include/g3log"
				COMMAND ${CMAKE_COMMAND} -E copy "<BINARY_DIR>/include/g3log/generated_definitions.hpp" "<INSTALL_DIR>/include/g3log"
		LIST_SEPARATOR "|"
		CMAKE_ARGS  
			"-DCHANGE_G3LOG_DEBUG_TO_DBUG=ON"
			"-DADD_FATAL_EXAMPLE=OFF"
			${G3LOG_CMAKE_ARGS}
	)

else()
	
	# Build and install using CMake
	ExternalProject_Add(g3log
		GIT_REPOSITORY "https://github.com/KjellKod/g3log.git"
		GIT_TAG "e988aadc6572769809eaf962cab1824eed62086e"
		PREFIX "g3log-${SWARMIO_TARGET_ARCHITECTURE}"
		PATCH_COMMAND ${GIT_EXECUTABLE} apply "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patches/g3log-cpack-prefix.patch"
		BUILD_COMMAND ${SWARMIO_TARGET_BUILD_COMMAND}
		INSTALL_COMMAND ${SWARMIO_TARGET_INSTALL_COMMAND}
		INSTALL_DIR ${G3LOG_ROOT}
		LIST_SEPARATOR "|"
		CMAKE_ARGS  
			"-DCHANGE_G3LOG_DEBUG_TO_DBUG=ON"
			"-DADD_FATAL_EXAMPLE=OFF"
			"-DCPACK_PACKAGING_INSTALL_PREFIX=<INSTALL_DIR>"
			${G3LOG_CMAKE_ARGS}
    )
	
endif()

# Finish
swarmio_build_component_finish(g3log)