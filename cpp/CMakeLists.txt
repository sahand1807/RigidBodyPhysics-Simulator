cmake_minimum_required(VERSION 3.12)
project(PhysicsEngine LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable position independent code for shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find Python
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# Find pybind11 - try config first, then use Python's pybind11
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
    # Use pybind11 from Python installation
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE pybind11_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    find_package(pybind11 CONFIG REQUIRED)
endif()

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

# ========================================
# Test Executables (optional)
# ========================================

# Build tests if requested (cmake -DBUILD_TESTS=ON)
option(BUILD_TESTS "Build test executables" OFF)

if(BUILD_TESTS)
    # Vector2 test
    add_executable(test_vector2
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/cpp/test_vector2.cpp
        src/math/Vector2.cpp
    )
    target_include_directories(test_vector2 PRIVATE include)

    # Transform test
    add_executable(test_transform
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/cpp/test_transform.cpp
        src/math/Transform.cpp
        src/math/Vector2.cpp
    )
    target_include_directories(test_transform PRIVATE include)

    # RigidBody test
    add_executable(test_rigidbody
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/cpp/test_rigidbody.cpp
        src/physics/RigidBody.cpp
        src/math/Transform.cpp
        src/math/Vector2.cpp
    )
    target_include_directories(test_rigidbody PRIVATE include)

    # Collider test
    add_executable(test_collider
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/cpp/test_collider.cpp
        src/physics/Collider.cpp
        src/physics/CircleCollider.cpp
        src/math/Transform.cpp
        src/math/Vector2.cpp
    )
    target_include_directories(test_collider PRIVATE include)

    # PhysicsWorld test
    add_executable(test_physicsworld
        ${CMAKE_CURRENT_SOURCE_DIR}/../tests/cpp/test_physicsworld.cpp
        src/physics/PhysicsWorld.cpp
        src/physics/RigidBody.cpp
        src/math/Transform.cpp
        src/math/Vector2.cpp
    )
    target_include_directories(test_physicsworld PRIVATE include)
endif()
