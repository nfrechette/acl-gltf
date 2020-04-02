cmake_minimum_required(VERSION 3.2)
project(acl-gltf NONE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

include(CMakeUtils)
include(CMakeCompiler)
include(CMakePlatforms)

set(USE_AVX_INSTRUCTIONS false CACHE BOOL "Use AVX instructions")
set(USE_SIMD_INSTRUCTIONS true CACHE BOOL "Use SIMD instructions")
set(CPU_INSTRUCTION_SET false CACHE STRING "CPU instruction set")

if(CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_CONFIGURATION_TYPES Debug Release)
	set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING "Reset the configurations to what we need" FORCE)
endif()

enable_testing()

# Add other projects
add_subdirectory("${PROJECT_SOURCE_DIR}/acl-gltf")
