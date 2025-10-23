# Vector2 Class - Complete Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Mathematical Foundation](#mathematical-foundation)
3. [Implementation Details](#implementation-details)
4. [Testing Strategy](#testing-strategy)
5. [Compilation and Execution](#compilation-and-execution)
6. [Usage Examples](#usage-examples)

---

## Introduction

### What is Vector2?

A **Vector2** is a fundamental mathematical structure representing a 2-dimensional vector with two components: `x` and `y`. In physics simulation, vectors are used to represent:

- **Positions**: Where an object is located in 2D space (e.g., `(5.0, 10.0)`)
- **Velocities**: How fast and in what direction an object is moving (e.g., `(2.0, -3.0)` m/s)
- **Accelerations**: Rate of change of velocity (e.g., `(0.0, -9.8)` for gravity)
- **Forces**: Push or pull on an object (e.g., `(100.0, 50.0)` Newtons)
- **Directions**: Unit vectors pointing somewhere (e.g., `(0.707, 0.707)` for northeast)

### Why Do We Need It?

Physics simulations are fundamentally about tracking how objects move and interact. Every object has:
- A **position** (Vector2)
- A **velocity** (Vector2)
- Forces acting on it (Vector2s)


With Vector2, physics equations become natural:
```cpp
// Without Vector2 (error-prone):
position_x = position_x + velocity_x * dt;
position_y = position_y + velocity_y * dt;

// With Vector2 (with Vector2):
position = position + velocity * dt;
```

---

## Mathematical Foundation

### Vector Basics

A 2D vector is represented as:
```
v = (x, y)
```

Where `x` is the horizontal component and `y` is the vertical component.

### Core Operations

#### 1. Addition
**Formula**: `(x₁, y₁) + (x₂, y₂) = (x₁ + x₂, y₁ + y₂)`


**Example**:
```
If an object moves (3, 2) then (1, 4), total displacement is:
(3, 2) + (1, 4) = (4, 6)
```


#### 2. Subtraction
**Formula**: `(x₁, y₁) - (x₂, y₂) = (x₁ - x₂, y₁ - y₂)`


**Example**:
```
To find direction from point A(1,1) to point B(4,5):
direction = B - A = (4,5) - (1,1) = (3,4)
```

#### 3. Scalar Multiplication
**Formula**: `k · (x, y) = (k·x, k·y)`


**Example**:
```
If velocity is (2, 3) m/s, after 5 seconds:
displacement = 5 × (2, 3) = (10, 15) meters
```

#### 4. Dot Product
**Formula**: `v₁ · v₂ = x₁·x₂ + y₁·y₂`

**Alternative formula**: `v₁ · v₂ = |v₁| · |v₂| · cos(θ)`

Where `θ` is the angle between vectors.


**Properties**:
- If `v₁ · v₂ = 0` → vectors are **perpendicular** (90° angle)
- If `v₁ · v₂ > 0` → vectors point in **similar directions** (< 90° angle)
- If `v₁ · v₂ < 0` → vectors point in **opposite directions** (> 90° angle)
- If `v₁ · v₂ = |v₁| · |v₂|` → vectors are **parallel** (same direction)

**Example**:
```
v₁ = (1, 0)  (pointing right)
v₂ = (0, 1)  (pointing up)

v₁ · v₂ = (1)(0) + (0)(1) = 0  → perpendicular!
```

**Use cases in physics**:
- Collision detection: Check if objects are moving toward or away from each other
- Projection: Find component of velocity in a given direction

#### 5. Cross Product (2D)
**Formula**: `v₁ × v₂ = x₁·y₂ - y₁·x₂` (returns a scalar in 2D)

**Physical meaning**: Determines rotational direction and "signed area" of parallelogram.

**Properties**:
- If `v₁ × v₂ > 0` → `v₂` is **counterclockwise** from `v₁`
- If `v₁ × v₂ < 0` → `v₂` is **clockwise** from `v₁`
- If `v₁ × v₂ = 0` → vectors are **parallel**


**Use cases in physics**:
- Torque calculation: `τ = r × F`
- Determining winding order for polygon collision
- Finding which side of a line a point is on

#### 6. Length (Magnitude)
**Formula**: `|v| = √(x² + y²)`


**Example**:
```
Velocity v = (3, 4) m/s
Speed = |v| = √(3² + 4²) = √(9 + 16) = √25 = 5 m/s
```

**Performance note**: Square root is expensive computationally! For comparisons, use `lengthSquared()`:
```cpp
// Slow: if (velocity.length() < 10.0f)
// Fast: if (velocity.lengthSquared() < 100.0f)
```

#### 7. Normalization
**Formula**: `v̂ = v / |v| = (x/|v|, y/|v|)`


**Example**:
```
v = (3, 4)
|v| = 5
v̂ = (3/5, 4/5) = (0.6, 0.8)

Verify: |(0.6, 0.8)| = √(0.36 + 0.64) = √1 = 1 ✓
```

**Use cases**:
- Direction vectors (ignoring magnitude)
- Surface normals in collision detection
- Applying forces in specific directions

**Critical**: Always check for zero vector before normalizing!
```cpp
if (v.length() > 0.0f) {
    v.normalize();  // Safe
}
```

#### 8. Distance
**Formula**: `distance(A, B) = |B - A| = √((x₂-x₁)² + (y₂-y₁)²)`


**Example**:
```
A = (0, 0)
B = (3, 4)
distance = √((3-0)² + (4-0)²) = √(9 + 16) = 5
```

#### 9. Perpendicular Vector
**Formula**: `(x, y)⊥ = (-y, x)` (90° counterclockwise rotation)


**Use cases**:

- Surface normals from edge vectors
- Tangent vectors
- Reflection calculations

---

## Implementation Details

### File Structure

```
cpp/
├── include/math/Vector2.hpp    # Header file (declarations)
└── src/math/Vector2.cpp        # Implementation file (definitions)
```


**Header file (.hpp)**:
- Declares the interface (what the class can do)
- Included by other files that use Vector2
- Changes here require recompiling everything that uses it

**Implementation file (.cpp)**:
- Contains the actual code for each function
- Only recompiled when implementation changes
- Faster compilation for large projects

### Header File (Vector2.hpp)

#### Header Guards
```cpp
#ifndef VECTOR2_HPP
#define VECTOR2_HPP
// ... code ...
#endif
```

**Purpose**: Prevents multiple inclusion errors.

**How it works**:
1. First time: `VECTOR2_HPP` not defined → include content → define `VECTOR2_HPP`
2. Second time: `VECTOR2_HPP` already defined → skip content

**Without guards**:
```cpp
// file1.hpp includes Vector2.hpp
// file2.hpp includes Vector2.hpp
// main.cpp includes both file1.hpp and file2.hpp
// Result: Vector2 class defined twice → compilation error!
```

#### Namespace
```cpp
namespace Physics {
    class Vector2 {
        // ...
    };
}
```

**Purpose**: Avoid naming conflicts.

**Example conflict without namespace**:
```cpp
// Our library
class Vector2 { ... };

// Some graphics library also has:
class Vector2 { ... };

// User's code - which Vector2??
Vector2 v;  // Ambiguous!
```

**With namespace**:
```cpp
Physics::Vector2 physicsVec;
Graphics::Vector2 graphicsVec;  // Clear!
```

#### Class Members

**Public data members**:
```cpp
public:
    float x;
    float y;
```

**Why public?**
- Vectors are simple data structures (like mathematical objects)
- Direct access is convenient: `v.x = 5.0f`
- No invariants to maintain (any x, y values are valid)

**Alternative** (more traditional OOP):
```cpp
private:
    float x;
    float y;
public:
    float getX() const { return x; }
    void setX(float value) { x = value; }
```
This is verbose for simple structures like vectors.

#### Operator Overloading

**Const correctness**:
```cpp
Vector2 operator+(const Vector2& other) const;
//                ^^^^^               ^^^^^
//                |                   |
//          doesn't modify 'other'   doesn't modify 'this'
```

**Benefits**:
- Compiler catches mistakes (trying to modify const objects)
- Can pass/return by reference efficiently
- Self-documenting code

**Example**:
```cpp
const Vector2 v1(1, 2);
Vector2 v2 = v1 + Vector2(3, 4);  // OK, operator+ is const
v1 += Vector2(5, 6);  // Compiler error! v1 is const, operator+= modifies
```

#### Return Types

**Return by value**:
```cpp
Vector2 operator+(const Vector2& other) const {
    return Vector2(x + other.x, y + other.y);
}
```
- Creates and returns new vector
- No aliasing issues
- Modern compilers optimize (RVO - Return Value Optimization)

**Return by reference**:
```cpp
Vector2& operator+=(const Vector2& other) {
    x += other.x;
    y += other.y;
    return *this;  // Return reference to this object
}
```
- Modifies and returns the same object
- Enables chaining: `v1 += v2 += v3`
- More efficient (no copy)

### Implementation File (Vector2.cpp)

#### Floating Point Precision

**Why floats not doubles?**
```cpp
Vector2::Vector2() : x(0.0f), y(0.0f) {}
//                        ^^      ^^
//                     float literals (f suffix)
```

**Reasoning**:
- **Performance**: 32-bit floats are 2× faster on modern CPUs
- **Memory**: Half the size (important for millions of particles)
- **Precision**: ~7 decimal digits is enough for games/simulations
- **GPU compatibility**: Most GPUs prefer 32-bit floats

**When to use doubles**: Scientific simulations requiring high precision.

#### Division by Zero Protection

```cpp
Vector2& Vector2::normalize() {
    float len = length();

    if (len > 0.0f) {  // Protect against division by zero!
        x /= len;
        y /= len;
    }

    return *this;
}
```

**Why critical?**
```cpp
Vector2 v(0, 0);
v.normalize();  // Without check: division by 0 → NaN (Not a Number)
// All future calculations with v produce NaN!
```

**NaN propagation**:
```cpp
Vector2 a = NaN_vector;
Vector2 b = some_normal_vector;
Vector2 c = a + b;  // c is now NaN!
// NaN spreads like a virus, breaking entire simulation
```

#### Floating Point Comparison

**Wrong way**:
```cpp
if (x == 0.0f && y == 0.0f) { ... }  // Dangerous!
```

**Why wrong?**
```cpp
float a = 0.1f + 0.2f;  // Might be 0.300000012, not exactly 0.3!
float b = 0.3f;
if (a == b) { ... }  // Might be false even though mathematically equal
```

**Right way**:
```cpp
bool Vector2::isZero() const {
    const float epsilon = 1e-6f;  // 0.000001
    return std::abs(x) < epsilon && std::abs(y) < epsilon;
}
```

**Epsilon choice**:
- Too small: false negatives (0.0000001 treated as non-zero)
- Too large: false positives (0.01 treated as zero)
- `1e-6f` is good balance for most simulations

#### Standard Library Functions

```cpp
#include <cmath>

float len = std::sqrt(x * x + y * y);  // Square root
float absX = std::abs(x);              // Absolute value
```

**Why `std::`?**
- Explicit namespace prevents conflicts
- Some compilers have both `::sqrt` and `std::sqrt`

---

## Testing Strategy

### Test File Structure

```
tests/cpp/test_vector2.cpp
```

**Why test?**
- Verify correctness of implementation
- Catch bugs early (cheaper to fix now than later)
- Document expected behavior
- Regression testing (ensure fixes don't break existing functionality)

### Test Organization

```cpp
void testConstructors() { ... }
void testArithmeticOperations() { ... }
void testDotProduct() { ... }
// ... more test functions ...

int main() {
    testConstructors();
    testArithmeticOperations();
    testDotProduct();
    // ...
}
```

**Benefits of modular tests**:
- Easy to see which category failed
- Can run specific tests during development
- Clear test organization

### Assertion-Based Testing

```cpp
assert(condition);
```

**How it works**:
- If `condition` is true → continue execution
- If `condition` is false → abort program with error message

**Example**:
```cpp
Vector2 v(3, 4);
assert(v.x == 3.0f);  // Pass
assert(v.x == 5.0f);  // FAIL: aborts with file:line information
```

### Floating Point Test Helper

```cpp
bool approxEqual(float a, float b, float epsilon = 1e-5f) {
    return std::abs(a - b) < epsilon;
}

bool approxEqual(const Vector2& a, const Vector2& b, float epsilon = 1e-5f) {
    return approxEqual(a.x, b.x, epsilon) &&
           approxEqual(a.y, b.y, epsilon);
}
```

**Usage**:
```cpp
Vector2 v = Vector2(3, 4).normalized();
// Don't: assert(v == Vector2(0.6f, 0.8f));  // Might fail due to precision
// Do: assert(approxEqual(v, Vector2(0.6f, 0.8f)));  // Tolerant comparison
```

### Test Coverage

Our tests verify:

1. **Constructors** (2 tests)
   - Default constructor creates (0, 0)
   - Parameterized constructor sets x, y correctly

2. **Arithmetic Operations** (6 tests)
   - Addition: (1,2) + (3,4) = (4,6)
   - Subtraction: (3,4) - (1,2) = (2,2)
   - Scalar multiplication: (1,2) * 2 = (2,4)
   - Commutative multiplication: 2 * (1,2) = (2,4)
   - Division: (3,4) / 2 = (1.5, 2)
   - Negation: -(1,2) = (-1,-2)

3. **Compound Operators** (4 tests)
   - Test +=, -=, *=, /= modify vector in place

4. **Dot Product** (3 tests)
   - Perpendicular vectors: dot = 0
   - Parallel vectors: dot = |v|²
   - General case

5. **Cross Product** (3 tests)
   - Counterclockwise rotation: positive
   - Clockwise rotation: negative
   - Parallel: zero

6. **Length and Normalization** (5 tests)
   - 3-4-5 triangle: length = 5
   - Length squared = 25
   - Normalized vector has length 1
   - Direction preserved after normalization
   - Zero vector normalization (edge case)

7. **Distance** (2 tests)
   - Distance calculation
   - Distance squared (optimization)

8. **Utilities** (3 tests)
   - Zero vector detection
   - Non-zero vector detection
   - Perpendicular vector (verify with dot product)

9. **Static Helpers** (3 tests)
   - Vector2::zero() = (0,0)
   - Vector2::right() = (1,0)
   - Vector2::up() = (0,1)

**Total: 31 distinct tests**

### Test Output Format

```
========================================
      Vector2 Test Suite
========================================

Testing constructors...
  ✓ Default constructor: Vector2(0, 0)
  ✓ Parameterized constructor: Vector2(3, 4)
✓ Constructors passed!
```

**Benefits**:
- Visual feedback (✓ for passed tests)
- Shows intermediate values
- Clear section headers
- Final summary

---

## Compilation and Execution

### Manual Compilation (What We Did)

```bash
g++ -std=c++17 -I cpp/include tests/cpp/test_vector2.cpp cpp/src/math/Vector2.cpp -o test_vector2
```

**Breaking down the command**:

| Component | Explanation |
|-----------|-------------|
| `g++` | GNU C++ compiler |
| `-std=c++17` | Use C++17 standard (for modern features) |
| `-I cpp/include` | Include directory (where to find .hpp files) |
| `tests/cpp/test_vector2.cpp` | Test file to compile |
| `cpp/src/math/Vector2.cpp` | Implementation file to compile |
| `-o test_vector2` | Output executable name |

**Include path explanation**:
```cpp
// In test file:
#include "math/Vector2.hpp"
//        ^^^^^^^^^^^^^^^^^^
//        Looks in: cpp/include/math/Vector2.hpp
//                  (because -I cpp/include)
```

**Compilation process**:
```
Source files (.cpp)
    ↓ [Preprocessing]
Expanded source (includes resolved)
    ↓ [Compilation]
Object files (.o)
    ↓ [Linking]
Executable (test_vector2)
```

### Running the Test

```bash
./test_vector2
```

**Output**: All test results printed to console

**Exit codes**:
- `0` = All tests passed
- `1` = At least one test failed

**Check exit code**:
```bash
./test_vector2
echo $?  # Prints exit code (0 = success)
```

### Cleanup

```bash
rm test_vector2  # Remove executable
```

**Why?**
- Executables are large (~100KB-1MB)
- Can be regenerated from source
- Git should ignore them (.gitignore)

---

## Usage Examples

### Example 1: Projectile Motion

```cpp
#include "math/Vector2.hpp"
using namespace Physics;

int main() {
    // Initial conditions
    Vector2 position(0.0f, 0.0f);           // Start at origin
    Vector2 velocity(10.0f, 15.0f);         // Launch velocity (m/s)
    Vector2 gravity(0.0f, -9.8f);           // Gravitational acceleration

    float dt = 0.016f;  // Time step (60 FPS)

    // Simulate for 100 frames
    for (int i = 0; i < 100; ++i) {
        // Update velocity: v = v + a*dt
        velocity += gravity * dt;

        // Update position: p = p + v*dt
        position += velocity * dt;

        // Check if hit ground
        if (position.y < 0.0f) {
            std::cout << "Hit ground at x = " << position.x << std::endl;
            break;
        }
    }

    return 0;
}
```

### Example 2: Collision Detection

```cpp
// Check if two circles are colliding
bool checkCircleCollision(Vector2 pos1, float radius1,
                         Vector2 pos2, float radius2) {
    // Calculate distance between centers
    float dist = pos1.distance(pos2);

    // Colliding if distance < sum of radii
    return dist < (radius1 + radius2);
}

// Optimized version (avoid sqrt)
bool checkCircleCollisionFast(Vector2 pos1, float radius1,
                              Vector2 pos2, float radius2) {
    float distSq = pos1.distanceSquared(pos2);
    float radiusSum = radius1 + radius2;
    return distSq < (radiusSum * radiusSum);
}
```

### Example 3: Reflection

```cpp
// Reflect velocity off a surface
Vector2 reflect(Vector2 velocity, Vector2 surfaceNormal) {
    // Formula: v' = v - 2(v·n)n
    // Where n is the normalized surface normal

    Vector2 n = surfaceNormal.normalized();
    float dot = velocity.dot(n);
    return velocity - n * (2.0f * dot);
}

// Example: Ball bouncing off floor
Vector2 velocity(5.0f, -10.0f);      // Moving down and right
Vector2 floorNormal(0.0f, 1.0f);     // Floor points up
Vector2 newVelocity = reflect(velocity, floorNormal);
// Result: (5.0, 10.0) - bounced upward!
```

### Example 4: Finding Closest Point

```cpp
// Find closest point on a line segment to a given point
Vector2 closestPointOnSegment(Vector2 point, Vector2 a, Vector2 b) {
    Vector2 ab = b - a;
    Vector2 ap = point - a;

    // Project ap onto ab
    float t = ap.dot(ab) / ab.dot(ab);

    // Clamp t to [0, 1] to stay on segment
    t = std::max(0.0f, std::min(1.0f, t));

    // Return point on segment
    return a + ab * t;
}
```

### Example 5: Steering Behaviors

```cpp
// Seek: Move toward target
Vector2 seek(Vector2 position, Vector2 velocity, Vector2 target,
            float maxSpeed) {
    // Desired velocity: pointing toward target at max speed
    Vector2 desired = (target - position).normalized() * maxSpeed;

    // Steering force: difference between desired and current
    Vector2 steering = desired - velocity;

    return steering;
}

// Flee: Move away from target
Vector2 flee(Vector2 position, Vector2 velocity, Vector2 threat,
            float maxSpeed) {
    // Desired velocity: pointing away from threat
    Vector2 desired = (position - threat).normalized() * maxSpeed;
    Vector2 steering = desired - velocity;
    return steering;
}
```

---

## Performance Considerations

### 1. Pass by Reference
```cpp
// Slow (copies 8 bytes):
void process(Vector2 v);

// Fast (passes 4-8 byte pointer):
void process(const Vector2& v);
```

### 2. Avoid Unnecessary Sqrt
```cpp
// Slow:
if (velocity.length() < 10.0f) { ... }

// Fast (50-100% faster):
if (velocity.lengthSquared() < 100.0f) { ... }
```

### 3. Use Compound Operators
```cpp
// Creates temporary:
position = position + velocity * dt;

// In-place (faster):
position += velocity * dt;
```

### 4. Cache Calculations
```cpp
// Slow (recalculates length twice):
Vector2 v = someVector;
if (v.length() > 0.0f) {
    v = v / v.length();  // Normalizing manually
}

// Fast (calculates length once):
Vector2 v = someVector;
v.normalize();  // Caches length internally
```

---

## Common Pitfalls

### 1. Forgetting to Normalize
```cpp
// Wrong: direction might be very long or very short
Vector2 direction = target - position;
velocity = direction * speed;  // Incorrect!

// Right: normalize to unit vector first
Vector2 direction = (target - position).normalized();
velocity = direction * speed;  // Correct!
```

### 2. Normalizing Zero Vector
```cpp
// Dangerous:
Vector2 v = Vector2::zero();
v.normalize();  // Our implementation handles this, but not all do!

// Safe:
if (!v.isZero()) {
    v.normalize();
}
```

### 3. Using == for Floats
```cpp
// Wrong:
if (velocity.x == 0.0f) { ... }

// Right:
if (std::abs(velocity.x) < 1e-6f) { ... }
// Or use: velocity.isZero()
```

### 4. Confusing Dot and Cross Product
```cpp
// Dot product: returns float (scalar)
float alignment = v1.dot(v2);

// Cross product (2D): also returns float (not vector!)
float rotation = v1.cross(v2);

// In 3D cross product returns vector, but not in 2D!
```

---

## Summary

### What We Implemented
- Complete Vector2 class with all essential operations
- 31 comprehensive tests covering all functionality

### Files Created
1. `cpp/include/math/Vector2.hpp` - Interface (313 lines)
2. `cpp/src/math/Vector2.cpp` - Implementation (174 lines)
3. `tests/cpp/test_vector2.cpp` - Test suite (401 lines)
4. `docs/01_Vector2_Documentation.md`

