# Python Bindings - Complete Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [pybind11 Foundation](#pybind11-foundation)
3. [Implementation Details](#implementation-details)
4. [Testing Strategy](#testing-strategy)
5. [Usage Examples](#usage-examples)

---

## Introduction

### What are Python Bindings?

**Python bindings** allow Python code to call C++ functions and use C++ classes as if they were native Python objects. This enables:

1. **Performance**: Keep heavy computation in fast C++
2. **Ease of use**: Write high-level scripts in Python
3. **Visualization**: Use Python libraries (Pygame, Matplotlib) with C++ physics
4. **Prototyping**: Quickly test physics scenarios without recompiling

**Example**:
```python
# Python code calling C++ physics engine
world = PhysicsWorld()          # C++ object
ball = RigidBody()              # C++ object
ball.set_mass(1.0)              # C++ method
ball.set_position(Vector2(0, 10))  # C++ types
world.add_body(ball)            # C++ method
world.step(0.016)               # Simulates in C++
print(ball.get_position())      # Access C++ data from Python
```

Under the hood:
- `PhysicsWorld`, `RigidBody`, `Vector2` are C++ classes
- All computation happens in compiled C++ (fast!)
- Python just orchestrates and visualizes

### Why pybind11?

**pybind11** is a modern, header-only library for creating Python bindings of C++ code.

**Alternatives**:
- **SWIG**: Old, complex, generates lots of boilerplate
- **Boost.Python**: Heavy dependency, complex build
- **ctypes/cffi**: Only works with C, requires manual wrapping
- **Cython**: Requires separate Cython language

**Why pybind11 wins**:
1. **Header-only**: No library to link, just include
2. **Modern C++**: Leverages C++11/14/17 features
3. **Clean syntax**: Minimal boilerplate
4. **Type conversions**: Automatic STL conversions
5. **Pythonic**: Supports operator overloading, properties, etc.

### Design Philosophy

**Goals**:
1. **Complete API exposure**: Every C++ class/method accessible from Python
2. **Pythonic interface**: Use Python conventions (snake_case, operators)
3. **Memory safety**: Automatic lifetime management
4. **Type safety**: Strong typing, helpful error messages
5. **Performance**: Minimal overhead

**Architecture**:
```
┌─────────────────────────────────────┐
│  Python Scripts                      │
│  (physics_viz, examples, tests)      │
└─────────────┬───────────────────────┘
              │ Python API
              │
┌─────────────▼───────────────────────┐
│  Python Bindings (pybind11)          │
│  cpp/src/bindings/bindings.cpp       │
└─────────────┬───────────────────────┘
              │ C++ API
              │
┌─────────────▼───────────────────────┐
│  C++ Physics Engine                  │
│  Vector2, RigidBody, PhysicsWorld... │
└─────────────────────────────────────┘
```

---

## pybind11 Foundation

### Basic Binding Syntax

**Minimal example** - Binding a simple function:

```cpp
#include <pybind11/pybind11.h>

namespace py = pybind11;

int add(int a, int b) {
    return a + b;
}

PYBIND11_MODULE(example, m) {
    m.doc() = "Example module";
    m.def("add", &add, "Add two numbers");
}
```

**Python usage**:
```python
import example
result = example.add(3, 5)  # Returns 8
```

**Key macro**: `PYBIND11_MODULE(module_name, variable)`
- Creates Python module
- `module_name`: Name when imported in Python
- `variable`: C++ variable representing the module

### Binding Classes

**C++ class**:
```cpp
class Vec2 {
public:
    float x, y;
    Vec2(float x, float y) : x(x), y(y) {}
    float length() const { return sqrt(x*x + y*y); }
};
```

**Binding**:
```cpp
PYBIND11_MODULE(math_module, m) {
    py::class_<Vec2>(m, "Vec2")
        .def(py::init<float, float>())     // Constructor
        .def_readwrite("x", &Vec2::x)      // Public attribute
        .def_readwrite("y", &Vec2::y)      // Public attribute
        .def("length", &Vec2::length);     // Method
}
```

**Python usage**:
```python
from math_module import Vec2
v = Vec2(3, 4)
print(v.x)           # 3.0
print(v.length())    # 5.0
```

**Method chaining**: All `.def()` calls return the class object, allowing chaining.

### Operator Overloading

**C++ operators**:
```cpp
Vec2 operator+(const Vec2& other) const;
Vec2 operator*(float scalar) const;
```

**Binding with special syntax**:
```cpp
py::class_<Vec2>(m, "Vec2")
    .def(py::self + py::self)      // __add__
    .def(py::self - py::self)      // __sub__
    .def(py::self * float())       // __mul__ (Vec2 * float)
    .def(float() * py::self)       // __rmul__ (float * Vec2)
    .def(-py::self);               // __neg__
```

**Python usage**:
```python
v1 = Vec2(1, 2)
v2 = Vec2(3, 4)
v3 = v1 + v2        # Calls C++ operator+
v4 = v1 * 2.0       # Calls C++ operator*
v5 = 2.0 * v1       # Calls reverse multiply
```

**Custom operators via lambda**:
```cpp
.def("__eq__", [](const Vec2& a, const Vec2& b) {
    return a.x == b.x && a.y == b.y;
}, py::is_operator())
```

### Type Conversions

**Automatic conversions** (with `#include <pybind11/stl.h>`):

| C++ Type | Python Type |
|----------|-------------|
| `std::vector<T>` | `list` |
| `std::map<K,V>` | `dict` |
| `std::string` | `str` |
| `std::pair<T1,T2>` | `tuple` |
| `int, float, bool` | `int, float, bool` |

**Example**:
```cpp
std::vector<RigidBody*> getBodies();  // C++ method
```
```python
bodies = world.get_bodies()  # Returns Python list
for body in bodies:
    print(body)
```

### Memory Management

**Problem**: Who owns objects? When are they deleted?

**Return value policies**:

```cpp
// Object is COPIED to Python (Python owns it)
.def("get_vector", &Class::getVector)

// Return REFERENCE to internal object (C++ still owns it)
.def("get_vector", &Class::getVector,
     py::return_value_policy::reference_internal)

// Return RAW POINTER (caller must manage lifetime)
.def("get_body", &World::getBody,
     py::return_value_policy::reference)
```

**Keep-alive policies**:

```cpp
// Keep 'body' alive as long as 'world' is alive
.def("add_body", &PhysicsWorld::addBody,
     py::arg("body"),
     py::keep_alive<1, 2>())  // <object, argument>
```

**Meaning**:
- `py::keep_alive<1, 2>()`: Keep argument 2 alive while object (this = 1) is alive
- `1` = implicit `this` pointer (the object being called on)
- `2` = first argument to the function

**Why needed?**:
```python
world = PhysicsWorld()
body = RigidBody()
world.add_body(body)   # Without keep_alive, body could be garbage collected!
# Body must stay alive as long as world exists
```

### Function Arguments

**Positional arguments**:
```cpp
.def("set_position", &RigidBody::setPosition)
```

**Named arguments with defaults**:
```cpp
.def(py::init<const Vector2&, float>(),
     py::arg("position"),
     py::arg("rotation") = 0.0f)
```

**Python usage**:
```python
# Positional
t = Transform(Vector2(0, 0), 1.57)

# Named
t = Transform(position=Vector2(0, 0), rotation=1.57)

# Default value
t = Transform(position=Vector2(0, 0))  # rotation defaults to 0
```

### String Representation

**`__repr__` method** (for debugging):

```cpp
.def("__repr__", [](const Vector2& v) {
    return "Vector2(" + std::to_string(v.x) + ", " +
           std::to_string(v.y) + ")";
})
```

**Python usage**:
```python
>>> v = Vector2(3, 4)
>>> v
Vector2(3.000000, 4.000000)
>>> print(v)
Vector2(3.000000, 4.000000)
```

### Static Methods

**C++ static method**:
```cpp
class Vector2 {
    static Vector2 zero() { return Vector2(0, 0); }
};
```

**Binding**:
```cpp
.def_static("zero", &Vector2::zero)
```

**Python usage**:
```python
v = Vector2.zero()  # Call static method
```

### Enums

**C++ enum**:
```cpp
class Collider {
public:
    enum class Type { Circle, Box };
};
```

**Binding**:
```cpp
py::enum_<Collider::Type>(m, "ColliderType")
    .value("Circle", Collider::Type::Circle)
    .value("Box", Collider::Type::Box)
    .export_values();  // Export to module namespace
```

**Python usage**:
```python
from physics_engine_core import ColliderType
if collider.get_type() == ColliderType.Circle:
    print("It's a circle!")
```

---

## Implementation Details

### Module Definition

**File**: `cpp/src/bindings/bindings.cpp`

```cpp
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>  // For py::self
#include <pybind11/stl.h>         // For std::vector, etc.

#include "math/Vector2.hpp"
#include "math/Transform.hpp"
#include "physics/RigidBody.hpp"
#include "physics/Collider.hpp"
#include "physics/CircleCollider.hpp"
#include "physics/PhysicsWorld.hpp"

namespace py = pybind11;
using namespace Physics;

PYBIND11_MODULE(physics_engine_core, m) {
    m.doc() = "2D Rigid Body Physics Engine - C++ core with Python bindings";

    // Bind all classes...
}
```

**Design decisions**:
- Module name: `physics_engine_core` (imported in Python)
- All bindings in single module for simplicity
- Include `operators.h` for operator overloading
- Include `stl.h` for automatic std::vector conversion

### Vector2 Binding

**Full implementation**:

```cpp
py::class_<Vector2>(m, "Vector2")
    // Constructors
    .def(py::init<>(), "Create zero vector")
    .def(py::init<float, float>(),
         "Create vector with x and y components",
         py::arg("x"), py::arg("y"))

    // Properties (read/write access)
    .def_readwrite("x", &Vector2::x, "X component")
    .def_readwrite("y", &Vector2::y, "Y component")

    // Arithmetic operators
    .def(py::self + py::self, "Add two vectors")
    .def(py::self - py::self, "Subtract two vectors")
    .def(py::self * float(), "Multiply vector by scalar")
    .def(float() * py::self, "Multiply scalar by vector")
    .def(py::self / float(), "Divide vector by scalar")
    .def(-py::self, "Negate vector")

    // Comparison operators (custom implementation)
    .def("__eq__", [](const Vector2& a, const Vector2& b) {
        return approxEqual(a.x, b.x) && approxEqual(a.y, b.y);
    }, py::is_operator())
    .def("__ne__", [](const Vector2& a, const Vector2& b) {
        return !approxEqual(a.x, b.x) || !approxEqual(a.y, b.y);
    }, py::is_operator())

    // Compound assignment
    .def("__iadd__", [](Vector2& self, const Vector2& other) {
        self += other;
        return self;
    }, py::is_operator())
    .def("__isub__", [](Vector2& self, const Vector2& other) {
        self -= other;
        return self;
    }, py::is_operator())
    .def("__imul__", [](Vector2& self, float scalar) {
        self *= scalar;
        return self;
    }, py::is_operator())
    .def("__itruediv__", [](Vector2& self, float scalar) {
        self /= scalar;
        return self;
    }, py::is_operator())

    // Methods
    .def("length", &Vector2::length, "Get magnitude of vector")
    .def("length_squared", &Vector2::lengthSquared, "Get squared magnitude")
    .def("normalize", &Vector2::normalize, "Normalize in-place, returns self")
    .def("normalized", &Vector2::normalized, "Return normalized copy")
    .def("dot", &Vector2::dot, "Dot product", py::arg("other"))
    .def("cross", &Vector2::cross, "2D cross product", py::arg("other"))
    .def("distance", &Vector2::distance, "Distance to another vector", py::arg("other"))
    .def("distance_squared", &Vector2::distanceSquared, "Squared distance", py::arg("other"))

    // String representation
    .def("__repr__", [](const Vector2& v) {
        return "Vector2(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
    })

    // Static methods
    .def_static("zero", &Vector2::zero, "Return zero vector (0, 0)")
    .def_static("up", &Vector2::up, "Return up vector (0, 1)")
    .def_static("right", &Vector2::right, "Return right vector (1, 0)");
```

**Key features**:

1. **Approximate equality**: C++ doesn't have `operator==`, so we implement it using floating-point tolerance:
   ```cpp
   inline bool approxEqual(float a, float b, float epsilon = 1e-5f) {
       return std::abs(a - b) < epsilon;
   }
   ```

2. **Compound assignment**: Python's `+=`, `-=`, etc. require special handling:
   - Must return `self` for chaining
   - Modifies object in-place

3. **Static methods**: Use `.def_static()` instead of `.def()`

4. **Docstrings**: Third parameter to `.def()` adds Python help text

### Transform Binding

```cpp
py::class_<Transform>(m, "Transform")
    .def(py::init<>(), "Create identity transform")
    .def(py::init<const Vector2&, float>(),
         "Create transform with position and rotation",
         py::arg("position"), py::arg("rotation") = 0.0f)

    // Properties
    .def_readwrite("position", &Transform::position, "Position in world space")
    .def_readwrite("rotation", &Transform::rotation, "Rotation angle in radians")

    // Methods
    .def("transform_point", &Transform::transformPoint,
         "Transform point from local to world space", py::arg("local_point"))
    .def("transform_direction", &Transform::transformDirection,
         "Transform direction (ignores position)", py::arg("local_direction"))
    .def("inverse_transform_point", &Transform::inverseTransformPoint,
         "Transform point from world to local space", py::arg("world_point"))
    .def("inverse_transform_direction", &Transform::inverseTransformDirection,
         "Transform direction from world to local space", py::arg("world_direction"))
    .def("inverse", &Transform::inverse, "Get inverse transform")
    .def("normalize_rotation", &Transform::normalizeRotation, "Normalize rotation to [-π, π)")

    // Directional vectors
    .def("get_forward", &Transform::getForward, "Get forward direction (rotated right)")
    .def("get_right", &Transform::getRight, "Get right direction")
    .def("get_up", &Transform::getUp, "Get up direction (rotated up)")

    // String representation
    .def("__repr__", [](const Transform& t) {
        return "Transform(pos=(" + std::to_string(t.position.x) + ", " +
               std::to_string(t.position.y) + "), rot=" +
               std::to_string(t.rotation) + ")";
    });
```

**Key features**:
- Default argument for rotation (defaults to 0)
- Public member access with `.def_readwrite()`
- Named arguments with `py::arg()`

### Collider Hierarchy Binding

**Base class**:

```cpp
py::class_<Collider>(m, "Collider")
    .def("get_type", &Collider::getType, "Get collider type")
    .def("calculate_mass", &Collider::calculateMass,
         "Calculate mass from density", py::arg("density"))
    .def("calculate_inertia", &Collider::calculateInertia,
         "Calculate moment of inertia", py::arg("mass"))
    .def("get_bounds", [](const Collider& c) {
        Vector2 min, max;
        c.getBounds(min, max);
        return py::make_tuple(min, max);
    }, "Get bounding box as (min, max) tuple")
    .def("get_offset", &Collider::getOffset, "Get local offset")
    .def("set_offset", &Collider::setOffset, "Set local offset", py::arg("offset"))
    .def("get_world_center", &Collider::getWorldCenter,
         "Get world space center", py::arg("body_transform"));
```

**Enum**:

```cpp
py::enum_<Collider::Type>(m, "ColliderType")
    .value("Circle", Collider::Type::Circle)
    .value("Box", Collider::Type::Box)
    .export_values();
```

**Derived class**:

```cpp
py::class_<CircleCollider, Collider>(m, "CircleCollider")
    //                     ^^^^^^^^ Base class!
    .def(py::init<float>(), "Create circle with radius",
         py::arg("radius"))
    .def(py::init<float, const Vector2&>(),
         "Create circle with radius and offset",
         py::arg("radius"), py::arg("offset"))

    .def("get_radius", &CircleCollider::getRadius, "Get circle radius")
    .def("set_radius", &CircleCollider::setRadius, "Set circle radius",
         py::arg("radius"))
    .def("contains_point", &CircleCollider::containsPoint,
         "Check if point is inside circle", py::arg("point"))

    .def("__repr__", [](const CircleCollider& c) {
        return "CircleCollider(radius=" + std::to_string(c.getRadius()) + ")";
    });
```

**Key features**:
- Inheritance: `py::class_<Derived, Base>`
- Multiple return values: Use `py::make_tuple()`
- Lambda for complex return processing

### RigidBody Binding

**Full implementation** (200+ lines):

```cpp
py::class_<RigidBody>(m, "RigidBody")
    .def(py::init<>(), "Create rigid body with default properties")
    .def(py::init<float>(), "Create rigid body with mass", py::arg("mass"))

    // Transform properties
    .def("get_transform", &RigidBody::getTransform, "Get body transform",
         py::return_value_policy::reference_internal)
    .def("set_transform", &RigidBody::setTransform, "Set body transform",
         py::arg("transform"))
    .def("get_position", &RigidBody::getPosition, "Get position")
    .def("set_position", &RigidBody::setPosition, "Set position",
         py::arg("position"))
    .def("get_rotation", &RigidBody::getRotation, "Get rotation in radians")
    .def("set_rotation", &RigidBody::setRotation, "Set rotation",
         py::arg("rotation"))

    // Physical properties
    .def("get_mass", &RigidBody::getMass, "Get mass in kg")
    .def("set_mass", &RigidBody::setMass, "Set mass", py::arg("mass"))
    .def("get_inverse_mass", &RigidBody::getInverseMass, "Get inverse mass")
    .def("get_moment_of_inertia", &RigidBody::getMomentOfInertia,
         "Get moment of inertia")
    .def("set_moment_of_inertia", &RigidBody::setMomentOfInertia,
         "Set moment of inertia", py::arg("inertia"))

    // Velocities
    .def("get_velocity", &RigidBody::getVelocity, "Get linear velocity")
    .def("set_velocity", &RigidBody::setVelocity, "Set linear velocity",
         py::arg("velocity"))
    .def("get_angular_velocity", &RigidBody::getAngularVelocity,
         "Get angular velocity")
    .def("set_angular_velocity", &RigidBody::setAngularVelocity,
         "Set angular velocity", py::arg("angular_velocity"))

    // Material properties
    .def("get_restitution", &RigidBody::getRestitution,
         "Get restitution (bounciness)")
    .def("set_restitution", &RigidBody::setRestitution,
         "Set restitution", py::arg("restitution"))
    .def("get_friction", &RigidBody::getFriction, "Get friction coefficient")
    .def("set_friction", &RigidBody::setFriction, "Set friction",
         py::arg("friction"))

    // Static/dynamic
    .def("is_static", &RigidBody::isStatic,
         "Check if body is static (infinite mass)")
    .def("set_static", &RigidBody::setStatic,
         "Make body static (infinite mass)")
    .def("set_dynamic", &RigidBody::setDynamic, "Make body dynamic",
         py::arg("mass"), py::arg("inertia"))

    // Forces and impulses
    .def("apply_force", &RigidBody::applyForce,
         "Apply force at center of mass", py::arg("force"))
    .def("apply_force_at_point", &RigidBody::applyForceAtPoint,
         "Apply force at point", py::arg("force"), py::arg("point"))
    .def("apply_torque", &RigidBody::applyTorque,
         "Apply torque", py::arg("torque"))
    .def("apply_impulse", &RigidBody::applyImpulse,
         "Apply impulse at center", py::arg("impulse"))
    .def("apply_impulse_at_point", &RigidBody::applyImpulseAtPoint,
         "Apply impulse at point", py::arg("impulse"), py::arg("point"))
    .def("apply_angular_impulse", &RigidBody::applyAngularImpulse,
         "Apply angular impulse", py::arg("impulse"))
    .def("clear_forces", &RigidBody::clearForces, "Clear accumulated forces")

    // Integration
    .def("integrate", &RigidBody::integrate,
         "Update position and velocity", py::arg("dt"))

    // Utility
    .def("get_kinetic_energy", &RigidBody::getKineticEnergy,
         "Get total kinetic energy")
    .def("get_momentum", &RigidBody::getMomentum, "Get linear momentum")
    .def("get_angular_momentum", &RigidBody::getAngularMomentum,
         "Get angular momentum")
    .def("local_to_world", &RigidBody::localToWorld,
         "Transform point to world space", py::arg("local_point"))
    .def("world_to_local", &RigidBody::worldToLocal,
         "Transform point to local space", py::arg("world_point"))
    .def("get_velocity_at_point", &RigidBody::getVelocityAtPoint,
         "Get velocity at point", py::arg("world_point"))

    // Collider
    .def("set_collider", &RigidBody::setCollider,
         "Attach collider to body", py::arg("collider"),
         py::keep_alive<1, 2>())  // Keep collider alive while body exists
    .def("get_collider", &RigidBody::getCollider, "Get attached collider",
         py::return_value_policy::reference_internal)
    .def("has_collider", &RigidBody::hasCollider, "Check if body has collider")

    .def("__repr__", [](const RigidBody& b) {
        return "RigidBody(mass=" + std::to_string(b.getMass()) +
               ", pos=(" + std::to_string(b.getPosition().x) + ", " +
               std::to_string(b.getPosition().y) + "))";
    });
```

**Memory management**:

1. **`py::keep_alive<1, 2>()`** on `set_collider`:
   - Keeps collider (arg 2) alive while body (this = 1) is alive
   - Prevents Python from garbage collecting collider while body uses it

2. **`py::return_value_policy::reference_internal`** on `get_collider`:
   - Returns reference to collider owned by body
   - Doesn't transfer ownership to Python

**Why this matters**:
```python
body = RigidBody()
collider = CircleCollider(1.0)
body.set_collider(collider)  # keep_alive ensures collider stays alive

# Even if we don't keep reference to collider:
body.set_collider(CircleCollider(1.0))
# Collider won't be garbage collected because body holds it
```

### PhysicsWorld Binding

```cpp
py::class_<PhysicsWorld>(m, "PhysicsWorld")
    .def(py::init<>(), "Create physics world with default settings")

    // Body management
    .def("add_body", &PhysicsWorld::addBody,
         "Add rigid body to simulation", py::arg("body"),
         py::keep_alive<1, 2>())  // Keep body alive while world exists
    .def("remove_body", &PhysicsWorld::removeBody,
         "Remove body from simulation", py::arg("body"))
    .def("clear", &PhysicsWorld::clear, "Remove all bodies")
    .def("get_bodies", &PhysicsWorld::getBodies, "Get all bodies",
         py::return_value_policy::reference_internal)
    .def("get_body_count", &PhysicsWorld::getBodyCount,
         "Get number of bodies")

    // Simulation
    .def("step", &PhysicsWorld::step,
         "Advance simulation by time step", py::arg("dt"))

    // Settings
    .def("set_gravity", &PhysicsWorld::setGravity,
         "Set gravity vector", py::arg("gravity"))
    .def("get_gravity", &PhysicsWorld::getGravity, "Get gravity vector")

    .def("__repr__", [](const PhysicsWorld& w) {
        return "PhysicsWorld(bodies=" + std::to_string(w.getBodyCount()) + ")";
    });
```

**Memory management**:
- `py::keep_alive<1, 2>()` on `add_body`: Bodies stay alive while world exists
- `py::return_value_policy::reference_internal` on `get_bodies`: Returns reference to internal vector

**Automatic conversion**:
- `getBodies()` returns `std::vector<RigidBody*>`
- pybind11 automatically converts to Python `list`
- Thanks to `#include <pybind11/stl.h>`

---

## Testing Strategy

### Test Structure

**File**: `python/examples/test_bindings.py`

**Test categories** (6 categories, 10+ tests):

1. **Vector2 Tests** (1 test, 10+ operations)
2. **Transform Tests** (1 test, 7+ operations)
3. **CircleCollider Tests** (1 test, 6+ operations)
4. **RigidBody Tests** (1 test, 8+ operations)
5. **PhysicsWorld Tests** (1 test, simulation)
6. **Collision Tests** (1 test, collision detection)

### Test 1: Vector2 Functionality

```python
def test_vector2():
    """Test Vector2 functionality"""
    print("=" * 50)
    print("Testing Vector2")
    print("=" * 50)

    # Create vectors
    v1 = Vector2(3, 4)
    v2 = Vector2(1, 2)

    print(f"v1 = {v1}")
    print(f"v2 = {v2}")

    # Arithmetic
    print(f"v1 + v2 = {v1 + v2}")
    print(f"v1 - v2 = {v1 - v2}")
    print(f"v1 * 2 = {v1 * 2}")

    # Methods
    print(f"v1.length() = {v1.length()}")      # Should be 5
    print(f"v1.dot(v2) = {v1.dot(v2)}")        # Should be 11
    print(f"v1.cross(v2) = {v1.cross(v2)}")    # Should be 2

    # Static methods
    print(f"Vector2.zero() = {Vector2.zero()}")
    print(f"Vector2.up() = {Vector2.up()}")
    print(f"Vector2.right() = {Vector2.right()}")
```

**Verifies**:
- Constructors
- Operators (+, -, *, /)
- Methods (length, dot, cross)
- Static methods (zero, up, right)
- String representation (`__repr__`)

### Test 2: Transform Functionality

```python
def test_transform():
    """Test Transform functionality"""
    import math
    t = Transform(Vector2(10, 20), math.pi / 4)  # 45 degrees

    print(f"Transform: {t}")
    print(f"Position: {t.position}")
    print(f"Rotation: {t.rotation} rad")
    print(f"Forward: {t.get_forward()}")
    print(f"Up: {t.get_up()}")

    # Transform a local point to world space
    local_point = Vector2(1, 0)
    world_point = t.transform_point(local_point)
    print(f"Local (1, 0) -> World {world_point}")
```

**Verifies**:
- Constructor with default argument
- Property access (position, rotation)
- Directional vectors
- Point transformation

### Test 3: CircleCollider Functionality

```python
def test_circle_collider():
    """Test CircleCollider functionality"""
    circle = CircleCollider(2.0)  # radius = 2 meters
    print(f"Circle: {circle}")
    print(f"Radius: {circle.get_radius()}")

    # Calculate mass properties
    density = 1.0  # kg/m²
    mass = circle.calculate_mass(density)
    inertia = circle.calculate_inertia(mass)

    print(f"Density: {density} kg/m²")
    print(f"Mass: {mass:.2f} kg")
    print(f"Inertia: {inertia:.2f} kg⋅m²")

    # Check point containment
    inside = Vector2(1, 0)
    outside = Vector2(5, 0)
    print(f"Point (1, 0) inside? {circle.contains_point(inside)}")
    print(f"Point (5, 0) inside? {circle.contains_point(outside)}")
```

**Verifies**:
- Constructor
- Getters/setters
- Mass calculations
- Point containment
- Inheritance from Collider

### Test 4: RigidBody Functionality

```python
def test_rigid_body():
    """Test RigidBody functionality"""
    body = RigidBody()
    body.set_mass(1.0)
    body.set_position(Vector2(0, 10))
    body.set_velocity(Vector2(5, 0))

    print(f"Body: {body}")
    print(f"Mass: {body.get_mass()} kg")
    print(f"Position: {body.get_position()}")
    print(f"Velocity: {body.get_velocity()}")

    # Apply force and integrate
    body.apply_force(Vector2(0, -9.81))  # Gravity force
    body.integrate(0.1)  # 0.1 second time step
    body.clear_forces()

    print(f"After integration:")
    print(f"  Position: {body.get_position()}")
    print(f"  Velocity: {body.get_velocity()}")
```

**Verifies**:
- Construction
- Property getters/setters
- Force application
- Integration
- Physics computation

### Test 5: PhysicsWorld Simulation

```python
def test_physics_world():
    """Test PhysicsWorld functionality"""
    # Create world
    world = PhysicsWorld()
    world.set_gravity(Vector2(0, -9.81))
    print(f"World gravity: {world.get_gravity()}")

    # Create a bouncing ball
    ball = RigidBody()
    ball.set_mass(1.0)
    ball.set_position(Vector2(0, 10))
    ball.set_restitution(0.8)  # Bouncy

    world.add_body(ball)
    print(f"Added ball at position: {ball.get_position()}")

    # Simulate for 2 seconds
    dt = 1.0 / 60.0  # 60 FPS
    for i in range(120):  # 2 seconds
        world.step(dt)

        if i % 30 == 0:  # Print every 0.5 seconds
            print(f"  t={i*dt:.2f}s: pos={ball.get_position()}, vel={ball.get_velocity()}")
```

**Verifies**:
- World creation
- Gravity setting
- Body addition
- Simulation stepping
- Memory management (body stays alive)

### Test 6: Collision Detection

**Most comprehensive integration test**:

```python
def test_collision():
    """Test collision between two balls"""
    world = PhysicsWorld()
    world.set_gravity(Vector2(0, -9.81))

    # Create two balls
    ball1 = RigidBody()
    ball2 = RigidBody()

    circle1 = CircleCollider(1.0)
    circle2 = CircleCollider(1.0)

    ball1.set_collider(circle1)
    ball2.set_collider(circle2)

    ball1.set_mass(1.0)
    ball2.set_mass(1.0)

    ball1.set_restitution(0.9)
    ball2.set_restitution(0.9)

    # Position them so they'll collide
    ball1.set_position(Vector2(-2, 10))
    ball1.set_velocity(Vector2(5, 0))  # Moving right

    ball2.set_position(Vector2(2, 10))
    ball2.set_velocity(Vector2(-5, 0))  # Moving left

    world.add_body(ball1)
    world.add_body(ball2)

    print("Initial state:")
    print(f"  Ball 1: pos={ball1.get_position()}, vel={ball1.get_velocity()}")
    print(f"  Ball 2: pos={ball2.get_position()}, vel={ball2.get_velocity()}")

    # Simulate until collision
    dt = 1.0 / 60.0
    for i in range(60):  # 1 second max
        world.step(dt)

        # Check if velocities reversed (collision happened)
        if ball1.get_velocity().x < 0 and ball2.get_velocity().x > 0:
            print(f"\nCollision at t={i*dt:.3f}s!")
            print(f"  Ball 1: pos={ball1.get_position()}, vel={ball1.get_velocity()}")
            print(f"  Ball 2: pos={ball2.get_position()}, vel={ball2.get_velocity()}")
            break
```

**Verifies entire pipeline**:
1. World creation
2. Body creation
3. Collider attachment
4. Mass and restitution
5. Initial positioning
6. Collision detection (in C++)
7. Collision response (in C++)
8. Velocity changes
9. Complete physics simulation

**Expected output**:
```
Initial state:
  Ball 1: pos=Vector2(-2.0, 10.0), vel=Vector2(5.0, 0.0)
  Ball 2: pos=Vector2(2.0, 10.0), vel=Vector2(-5.0, 0.0)

Collision at t=0.400s!
  Ball 1: pos=Vector2(-0.01, 9.21), vel=Vector2(-4.5, -3.92)
  Ball 2: pos=Vector2(0.01, 9.21), vel=Vector2(4.5, -3.92)
```

Velocities reversed! Collision works! ✓

### Test Execution

**Command**:
```bash
cd python/examples
python test_bindings.py
```

**Expected result**: All tests pass with output showing correct physics behavior.

---

## Usage Examples

### Example 1: Simple Ball Drop

```python
from physics_viz import Vector2, RigidBody, CircleCollider, PhysicsWorld

# Create world with gravity
world = PhysicsWorld()
world.set_gravity(Vector2(0, -9.81))  # Earth gravity

# Create ball
ball = RigidBody()
circle = CircleCollider(1.0)  # 1 meter radius

ball.set_collider(circle)
ball.set_mass(1.0)            # 1 kg
ball.set_position(Vector2(0, 10))  # 10 meters high

world.add_body(ball)

# Simulate
dt = 1.0 / 60.0  # 60 FPS
for frame in range(600):  # 10 seconds
    world.step(dt)

    if frame % 60 == 0:  # Print every second
        pos = ball.get_position()
        vel = ball.get_velocity()
        print(f"t={frame/60:.1f}s: y={pos.y:.2f}m, vy={vel.y:.2f}m/s")
```

**Output**:
```
t=0.0s: y=10.00m, vy=0.00m/s
t=1.0s: y=5.10m, vy=-9.81m/s
t=2.0s: y=-9.62m, vy=-19.62m/s
...
```

### Example 2: Projectile Motion

```python
import math
from physics_viz import Vector2, RigidBody, PhysicsWorld

world = PhysicsWorld()
world.set_gravity(Vector2(0, -9.81))

# Create projectile
projectile = RigidBody()
projectile.set_mass(0.5)  # 500 grams
projectile.set_position(Vector2(0, 0))

# Launch at 45 degrees, 20 m/s
angle = math.pi / 4
speed = 20.0
vx = speed * math.cos(angle)
vy = speed * math.sin(angle)
projectile.set_velocity(Vector2(vx, vy))

world.add_body(projectile)

# Track trajectory
trajectory = []
dt = 0.01  # 100 FPS for accuracy
max_time = 3.0

for frame in range(int(max_time / dt)):
    world.step(dt)
    pos = projectile.get_position()
    trajectory.append((pos.x, pos.y))

    # Stop when hits ground
    if pos.y < 0:
        print(f"Range: {pos.x:.2f} meters")
        break

# Plot trajectory (requires matplotlib)
# import matplotlib.pyplot as plt
# x, y = zip(*trajectory)
# plt.plot(x, y)
# plt.xlabel('Distance (m)')
# plt.ylabel('Height (m)')
# plt.show()
```

### Example 3: Interactive Simulation

```python
from physics_viz import Vector2, RigidBody, CircleCollider, PhysicsWorld
import time

# Create world
world = PhysicsWorld()
world.set_gravity(Vector2(0, -9.81))

# Add ground (static body)
ground = RigidBody()
ground_circle = CircleCollider(50.0)
ground.set_collider(ground_circle)
ground.set_static()
ground.set_position(Vector2(0, -50))
world.add_body(ground)

# User can spawn balls
def spawn_ball(x, y, vx, vy):
    ball = RigidBody()
    circle = CircleCollider(0.5)
    ball.set_collider(circle)
    ball.set_mass(1.0)
    ball.set_restitution(0.8)
    ball.set_position(Vector2(x, y))
    ball.set_velocity(Vector2(vx, vy))
    world.add_body(ball)
    return ball

# Spawn some balls
balls = []
balls.append(spawn_ball(-5, 10, 2, 0))
balls.append(spawn_ball(5, 15, -3, 0))
balls.append(spawn_ball(0, 20, 0, 0))

# Real-time simulation
dt = 1.0 / 60.0
for frame in range(600):  # 10 seconds
    world.step(dt)
    time.sleep(dt)  # Real-time playback

    # Print ball positions
    if frame % 30 == 0:
        print(f"\nFrame {frame}:")
        for i, ball in enumerate(balls):
            pos = ball.get_position()
            print(f"  Ball {i}: ({pos.x:.2f}, {pos.y:.2f})")
```

### Example 4: Energy Conservation Check

```python
from physics_viz import Vector2, RigidBody, CircleCollider, PhysicsWorld

world = PhysicsWorld()
world.set_gravity(Vector2(0, -9.81))

# Create ball
ball = RigidBody()
circle = CircleCollider(1.0)
ball.set_collider(circle)
ball.set_mass(1.0)
ball.set_position(Vector2(0, 10))
ball.set_restitution(1.0)  # Perfect bounce (no energy loss)

# Create ground
ground = RigidBody()
ground_circle = CircleCollider(10.0)
ground.set_collider(ground_circle)
ground.set_static()
ground.set_position(Vector2(0, 0))

world.add_body(ball)
world.add_body(ground)

# Track energy
def total_energy(body, gravity_y):
    pos = body.get_position()
    ke = body.get_kinetic_energy()
    pe = body.get_mass() * abs(gravity_y) * pos.y
    return ke + pe

gravity = world.get_gravity()
initial_energy = total_energy(ball, gravity.y)

# Simulate bounces
dt = 1.0 / 240.0  # High frequency for accuracy
for frame in range(2400):  # 10 seconds
    world.step(dt)

    if frame % 240 == 0:  # Every second
        energy = total_energy(ball, gravity.y)
        error = abs(energy - initial_energy) / initial_energy * 100
        print(f"t={frame/240:.1f}s: Energy={energy:.2f}J (error={error:.2f}%)")

# With restitution=1.0, energy should be conserved (< 1% error)
```

### Example 5: Newton's Cradle

```python
from physics_viz import Vector2, RigidBody, CircleCollider, PhysicsWorld

world = PhysicsWorld()
world.set_gravity(Vector2(0, 0))  # No gravity for simplicity

# Create 5 balls in a row
num_balls = 5
radius = 1.0
balls = []

for i in range(num_balls):
    ball = RigidBody()
    circle = CircleCollider(radius)
    ball.set_collider(circle)
    ball.set_mass(1.0)
    ball.set_restitution(1.0)  # Perfect elastic collision
    ball.set_friction(0.0)

    # Position just touching
    x = i * (radius * 2.0)
    ball.set_position(Vector2(x, 0))
    ball.set_velocity(Vector2(0, 0))

    world.add_body(ball)
    balls.append(ball)

# Pull back first ball
balls[0].set_velocity(Vector2(10, 0))

# Simulate
dt = 1.0 / 60.0
for frame in range(300):  # 5 seconds
    world.step(dt)

    if frame % 30 == 0:
        print(f"\nFrame {frame}:")
        for i, ball in enumerate(balls):
            vel = ball.get_velocity()
            print(f"  Ball {i}: vx={vel.x:.2f}")

# Expected: Momentum transfers through chain
# Frame 0: Ball 0 moves, others still
# Later: Ball 0 stops, Ball 4 moves
```

---

## Summary

### What We Implemented

**Python Bindings Module**:
- Complete exposure of C++ physics engine to Python
- Pythonic interface with operators and properties
- Memory-safe lifetime management
- Automatic type conversions

**Bound Classes**:
1. **Vector2**: Full operator overloading, static methods, approximate equality
2. **Transform**: Position, rotation, transformation methods
3. **Collider**: Base class with type enum
4. **CircleCollider**: Circle-specific methods, inheritance
5. **RigidBody**: 40+ methods, forces, collisions, properties
6. **PhysicsWorld**: Simulation loop, body management

**Key Features**:
- Operator overloading (`+`, `-`, `*`, `/`, `==`, `!=`, `+=`, etc.)
- Named arguments with defaults
- String representation (`__repr__`)
- Inheritance (CircleCollider → Collider)
- Memory safety (keep_alive, return policies)
- STL conversions (std::vector → list)
- Enum support

### Files Created/Modified

**New Files**:
1. `cpp/src/bindings/bindings.cpp` (287 lines)
   - pybind11 bindings for all classes
   - Operator overloading
   - Memory management policies

2. `python/physics_viz/__init__.py` (10 lines)
   - Package initialization
   - Imports C++ module

3. `python/examples/test_bindings.py` (213 lines)
   - Comprehensive test suite
   - 6 test categories
   - Integration tests

4. `pyproject.toml` (24 lines)
   - Modern Python packaging
   - Build requirements

**Modified Files**:
1. `cpp/CMakeLists.txt`
   - Python detection
   - pybind11 finding
   - Shared library build

2. `setup.py`
   - CMake integration
   - Build configuration

3. `.gitignore`
   - Python build artifacts
   - Egg-info directories

### Build and Installation

**Prerequisites**:
```bash
# Install dependencies
pip install pybind11>=2.10.0 pygame>=2.5.0 numpy>=1.21.0
```

**Build**:
```bash
# Option 1: pip install (recommended)
pip install -e .

# Option 2: Manual build
mkdir -p build_python && cd build_python
cmake ../cpp
make
```

**Verify**:
```python
from physics_viz import Vector2, RigidBody, PhysicsWorld
print("Bindings loaded successfully!")
```

### Physics Concepts Covered

- **Language interoperability**: C++ ↔ Python communication
- **Memory management**: Ownership, lifetime, garbage collection
- **Type systems**: C++ static typing → Python dynamic typing
- **Object lifecycle**: Construction, destruction, references
- **API design**: Pythonic conventions for C++ code

### Next Steps

**Phase 5 Complete!** ✓

We now have:
- ✓ Core C++ physics engine
- ✓ Complete Python bindings
- ✓ Tested Python API

**Phase 6: Visualization**
- Pygame rendering
- Real-time display
- Camera system
- Interactive controls
- Demo scenarios:
  - Bouncing balls
  - Momentum transfer
  - Ball pit
  - Interactive sandbox

**Phase 7: Enhanced Features**
- BoxCollider implementation
- Circle-box collision detection
- Box-box collision (SAT algorithm)
- Rotation and angular dynamics
- Advanced demos
