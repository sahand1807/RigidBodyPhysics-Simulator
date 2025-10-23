cmake_minimum_required(VERSION 3.12)
project(PhysicsEngine LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable position independent code for shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find pybind11
find_package(pybind11 CONFIG REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Collect all source files
file(GLOB_RECURSE MATH_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/math/*.cpp"
)

file(GLOB_RECURSE PHYSICS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/physics/*.cpp"
)

file(GLOB BINDING_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/bindings/*.cpp"
)

# Create the Python module
pybind11_add_module(physics_engine_core
    ${MATH_SOURCES}
    ${PHYSICS_SOURCES}
    ${BINDING_SOURCES}
)

# Set output directory for the module
set_target_properties(physics_engine_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../python/physics_viz
)

# Compiler warnings
if(MSVC)
    target_compile_options(physics_engine_core PRIVATE /W4)
else()
    target_compile_options(physics_engine_core PRIVATE -Wall -Wextra -pedantic)
endif()
