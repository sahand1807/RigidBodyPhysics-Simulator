# Transform Class - Complete Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Mathematical Foundation](#mathematical-foundation)
3. [Implementation Details](#implementation-details)
4. [Testing Strategy](#testing-strategy)
5. [Compilation and Execution](#compilation-and-execution)
6. [Usage Examples](#usage-examples)

---

## Introduction

### What is Transform?

A **Transform** represents the position and orientation of an object in 2D space. It combines:
- **Position** (Vector2): Where the object is located
- **Rotation** (float): How the object is oriented (angle in radians)

In physics simulation, every rigid body has a transform that describes its placement in the world.


**Example scenario**: A box collision shape is defined in local space (centered at origin). To check if a world point hits the box:
1. Use transform to convert world point → local space
2. Check if local point is inside the box shape
3. If collision detected, use transform to convert collision normal back to world space

### Coordinate Systems

**World Space**: The global coordinate system. All objects' absolute positions are in world space.

**Local Space**: Each object has its own coordinate system, centered at its position with its own orientation.


---

## Mathematical Foundation

### 2D Rotation

#### Rotation Matrix

To rotate a 2D vector by angle θ:

```
[ cos(θ)  -sin(θ) ] [ x ]   [ x·cos(θ) - y·sin(θ) ]
[ sin(θ)   cos(θ) ] [ y ] = [ x·sin(θ) + y·cos(θ) ]
```

**Intuition**:
- At θ = 0°: (x,y) → (x, y) (no change)
- At θ = 90°: (x,y) → (-y, x) (rotates left)
- At θ = 180°: (x,y) → (-x, -y) (flips)
- At θ = 270°: (x,y) → (y, -x) (rotates right)

**Example**:
```
Rotate point (1, 0) by 90° (π/2 radians):
cos(90°) = 0, sin(90°) = 1

x' = 1·0 - 0·1 = 0
y' = 1·1 + 0·0 = 1

Result: (0, 1) ✓ Points up!
```

#### Angle Convention

We use **standard mathematical convention**:
- Angles measured counterclockwise from positive x-axis (right)
- 0 rad = pointing right (1, 0)
- π/2 rad (90°) = pointing up (0, 1)
- π rad (180°) = pointing left (-1, 0)
- 3π/2 rad (270°) = pointing down (0, -1)


### Transform Operations

#### 1. Transform Point (Local → World)

**Formula**: `worldPoint = position + rotate(localPoint, rotation)`

**Order matters**:
1. First: Rotate the local point
2. Then: Add the world position

**Example**:
```
Transform at position (10, 20), rotation 0°
Local point: (5, 0)

Step 1: Rotate (5, 0) by 0° = (5, 0)
Step 2: Add position: (5, 0) + (10, 20) = (15, 20)

World point: (15, 20)
```

#### 2. Transform Direction (Local → World)

**Formula**: `worldDirection = rotate(localDirection, rotation)`

**Key difference from points**: Directions are **not affected by position**, only rotation.

**Why?** A direction like "forward" doesn't have a position. If you're at (100, 200) facing north, "forward" is still (0, 1), not (100, 201).

**Example**:
```
Transform at position (100, 200), rotation 90°
Local direction: (1, 0) "forward in local space"

Rotate (1, 0) by 90°:
x' = 1·cos(90°) - 0·sin(90°) = 0
y' = 1·sin(90°) + 0·cos(90°) = 1

World direction: (0, 1) "points up in world space"
(Position 100, 200 is ignored!)
```

#### 3. Inverse Transform (World → Local)

**Formula**: `localPoint = rotate(worldPoint - position, -rotation)`

**Order matters** (reverse of forward transform):
1. First: Subtract position
2. Then: Rotate backwards (negative angle)

**Example**:
```
Transform at position (10, 20), rotation 90°
World point: (10, 25)

Step 1: Subtract position: (10, 25) - (10, 20) = (0, 5)
Step 2: Rotate by -90°:
  x' = 0·cos(-90°) - 5·sin(-90°) = 5
  y' = 0·sin(-90°) + 5·cos(-90°) = 0

Local point: (5, 0)
```

**Use case**: Checking if a world point is inside a local collision shape.

#### 4. Combining Transforms

**Formula**:
```
Combined = A.combine(B)
Combined.position = B.position + rotate(A.position, B.rotation)
Combined.rotation = A.rotation + B.rotation
```

**Intuition**: Apply A first, then B. Like matrix multiplication (but reversed order).

**Example: Parent-child**:
```
Parent: position (10, 0), rotation 90°
Child (relative to parent): position (5, 0), rotation 0°

Child in world = Child.combine(Parent)

Rotation: 0° + 90° = 90°
Position:
  - Rotate child's (5, 0) by parent's 90° = (0, 5)
  - Add parent's (10, 0)
  - Result: (10, 5)

Child in world: position (10, 5), rotation 90°
```


#### 5. Inverse Transform

**Formula**:
```
Inverse.rotation = -rotation
Inverse.position = rotate(-position, -rotation)
```

**Property**: `T.inverse()` undoes `T`
```
world = T.transformPoint(local)
local = T.inverse().transformPoint(world)
```

**Example**:
```
T: position (5, 10), rotation 45°

Inverse:
  rotation = -45°
  position = rotate(-(5,10), -45°)
           = rotate(-5, -10) by -45°
           ≈ (-10.6, -3.5)

Verification:
T.transformPoint((0,0)) = (5, 10)
T_inv.transformPoint((5,10)) = (0, 0) ✓
```

### Directional Vectors

Every transform has three orthogonal unit vectors:

#### Forward Vector
**Formula**: `(cos(rotation), sin(rotation))`

The direction the object is facing.

```
Rotation 0°: forward = (1, 0) → right
Rotation 90°: forward = (0, 1) → up
Rotation 180°: forward = (-1, 0) → left
Rotation 270°: forward = (0, -1) → down
```

#### Right Vector
**Formula**: `(sin(rotation), -cos(rotation))`

90° clockwise from forward.

```
Rotation 0°: right = (0, -1) → down
Rotation 90°: right = (1, 0) → right
```

#### Up Vector
**Formula**: `(-sin(rotation), cos(rotation))`

90° counterclockwise from forward (opposite of right).

```
Rotation 0°: up = (0, 1) → up
Rotation 90°: up = (-1, 0) → left
```

**Verification**: These are always perpendicular and unit length:
```
forward · right = 0 (perpendicular)
|forward| = |right| = |up| = 1 (unit length)
```

### Angle Normalization

**Problem**: After many rotations, angle might be 1000π, causing numerical issues.

**Solution**: Normalize to range [-π, π).

**Algorithm**:
```
1. rotation = fmod(rotation, 2π)  // Get into (-2π, 2π)
2. If rotation ≥ π: rotation -= 2π  // Map [π, 2π) to [-π, 0)
3. If rotation < -π: rotation += 2π  // Map (-2π, -π) to (0, π)
```

**Examples**:
```
3π → π → -π (subtract 2π)
-3π → -π (already in range)
2π → 0 (fmod gives ~0)
5.5π → 1.5π → -0.5π (subtract 2π)
```

**Why [-π, π) not [-π, π]?**
- π and -π represent the same angle (180°)
- Choosing [-π, π) makes π map to -π for consistency

---

## Implementation Details

### File Structure

```
cpp/
├── include/math/Transform.hpp    # Header file (declarations)
└── src/math/Transform.cpp        # Implementation file (definitions)
```

### Header File (Transform.hpp)

#### Public Members

```cpp
Vector2 position;  // Position in world space
float rotation;    // Rotation angle in radians
```

**Why public?** Like Vector2, Transform is a simple data structure. Direct access is convenient and appropriate.

#### Constructors

```cpp
Transform();  // Identity: (0,0), 0 rotation
Transform(const Vector2& position, float rotation);
explicit Transform(const Vector2& position);  // No rotation
```

**`explicit`** keyword on single-argument constructor prevents accidental implicit conversions:
```cpp
// Without explicit:
void foo(Transform t);
foo(Vector2(5, 10));  // Accidentally creates Transform! (Implicit conversion)

// With explicit:
foo(Vector2(5, 10));  // Compiler error! Must explicitly write:
foo(Transform(Vector2(5, 10)));
```

#### Const Correctness

All transformation functions are `const`:
```cpp
Vector2 transformPoint(const Vector2& localPoint) const;
//                                                ^^^^^ doesn't modify Transform
```

**Benefits**:
- Can use with const Transform objects
- Compiler catches mistakes
- Documents intent: "this function is read-only"

### Implementation File (Transform.cpp)

#### Rotation Matrix Implementation

```cpp
Vector2 Transform::rotate(const Vector2& v, float angle) {
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);

    return Vector2(
        v.x * cosAngle - v.y * sinAngle,
        v.x * sinAngle + v.y * cosAngle
    );
}
```

**Optimization**: Pre-compute `cos` and `sin` once, use twice. Trigonometric functions are expensive (~100 CPU cycles each).

#### Transform Point vs Direction

```cpp
// Point: position matters
Vector2 Transform::transformPoint(const Vector2& localPoint) const {
    return position + rotate(localPoint, rotation);
    //     ^^^^^^^^ adds position!
}

// Direction: position ignored
Vector2 Transform::transformDirection(const Vector2& localDirection) const {
    return rotate(localDirection, rotation);
    //     ^^^^^^^^ no position added!
}
```

#### Angle Normalization with Floating Point

```cpp
void Transform::normalizeRotation() {
    rotation = std::fmod(rotation, 2.0f * M_PI);

    const float epsilon = 1e-6f;

    if (rotation > M_PI - epsilon) {
        rotation -= 2.0f * M_PI;
    } else if (rotation <= -M_PI - epsilon) {
        rotation += 2.0f * M_PI;
    }
}
```

**Why epsilon?** Floating point precision issues near boundaries.

**Without epsilon**:
```
rotation = exactly π (from fmod)
Check: π > π? False
Result: stays at π (wrong! should be -π)
```

**With epsilon**:
```
rotation ≈ π
Check: π > π - 1e-6? True!
Result: π - 2π = -π ✓
```

#### LookAt Function

```cpp
void Transform::lookAt(const Vector2& target) {
    Vector2 direction = target - position;
    rotation = std::atan2(direction.y, direction.x);
}
```

**`atan2(y, x)`**: Returns angle of vector (x, y) in range [-π, π].

**Why atan2 not atan?**
```
atan(y/x):  Loses sign information, range only [-π/2, π/2]
atan2(y,x): Preserves quadrant, range [-π, π]
```

**Examples**:
```
atan2(1, 1) = π/4     (45°, quadrant I)
atan2(1, -1) = 3π/4   (135°, quadrant II)
atan2(-1, -1) = -3π/4 (-135°, quadrant III)
atan2(-1, 1) = -π/4   (-45°, quadrant IV)
```

---

## Testing Strategy

### Test File Structure

```
tests/cpp/test_transform.cpp
```

### Test Categories

We organized tests into 8 categories, testing different aspects:

#### 1. Constructors (3 tests)
- Default constructor (identity)
- Full constructor (position + rotation)
- Position-only constructor

#### 2. Transform Point (4 tests)
- Translation only (no rotation)
- Rotation only (no translation)
- Combined rotation and translation
- 180° rotation edge case

#### 3. Transform Direction (3 tests)
- Direction unaffected by position
- Direction affected by rotation
- 45° rotation verification

#### 4. Inverse Transform (3 tests)
- Forward + inverse for points (should cancel out)
- Forward + inverse for directions
- Using `inverse()` method

#### 5. Combine Transforms (3 tests)
- Combining translations
- Combining rotations
- Parent-child scenario

#### 6. Directional Vectors (4 tests)
- No rotation (0°)
- 90° rotation
- Perpendicularity verification (dot products = 0)
- Unit length verification

#### 7. Utility Functions (7 tests)
- Normalize 3π → -π
- Normalize -3π → -π
- Normalize 2π → 0
- Rotate by angle
- Translate by vector
- LookAt (45° target)
- LookAt (90° target)

#### 8. Static Helpers (3 tests)
- `identity()`
- `translation()`
- `rotationTransform()`

**Total: 30 tests**

### Special Test Cases

#### Parent-Child Transform Test
```cpp
Transform parent(Vector2(10.0f, 0.0f), M_PI / 2.0f);  // At (10,0), facing up
Transform childLocal(Vector2(5.0f, 0.0f), 0.0f);      // 5 units forward
Transform childWorld = childLocal.combine(parent);

// Expected: child at (10, 5) in world
// Reason: 5 units "forward" in parent's space = 5 units up = (10, 5)
```

#### Rotation Normalization Edge Cases
```cpp
// 3π should normalize to -π (not stay at π)
Transform t(Vector2::zero(), 3.0f * M_PI);
t.normalizeRotation();
assert(approxEqual(t.rotation, -M_PI));  // Tests boundary handling
```

### Floating Point Comparison

Same strategy as Vector2:
```cpp
bool approxEqual(float a, float b, float epsilon = 1e-4f) {
    return std::abs(a - b) < epsilon;
}
```

**Why 1e-4f?**
- Larger than 1e-6f (used in Vector2) because rotations involve trig functions
- Trig functions have slightly more floating point error
- Still tight enough to catch real bugs

---

## Compilation and Execution

### Manual Compilation

```bash
g++ -std=c++17 -I cpp/include \
    tests/cpp/test_transform.cpp \
    cpp/src/math/Vector2.cpp \
    cpp/src/math/Transform.cpp \
    -o test_transform
```

**Breaking down the command**:

| Component | Explanation |
|-----------|-------------|
| `g++` | GNU C++ compiler |
| `-std=c++17` | Use C++17 standard |
| `-I cpp/include` | Include directory for headers |
| `tests/cpp/test_transform.cpp` | Test file |
| `cpp/src/math/Vector2.cpp` | Vector2 implementation (dependency) |
| `cpp/src/math/Transform.cpp` | Transform implementation |
| `-o test_transform` | Output executable name |

**Why include Vector2.cpp?** Transform depends on Vector2, so we need both implementations.

### Running the Test

```bash
./test_transform
```

**Expected output**:
```
========================================
      Transform Test Suite
========================================

Testing constructors...
  ✓ Default constructor: pos=Vector2(0, 0), rot=0
  ...

✓ ALL TESTS PASSED!
========================================
```


---

## Usage Examples

### Example 1: Basic Object Placement

```cpp
#include "math/Transform.hpp"
#include "math/Vector2.hpp"

// Place a box at (50, 100), rotated 45 degrees
Transform boxTransform(Vector2(50.0f, 100.0f), M_PI / 4.0f);

// Box has a local collision point at (10, 10)
Vector2 localCorner(10.0f, 10.0f);

// Where is this corner in world space?
Vector2 worldCorner = boxTransform.transformPoint(localCorner);

std::cout << "Corner in world: " << worldCorner << std::endl;
// Output: Corner in world: Vector2(50.0, 114.14) (approximately)
```

### Example 2: Collision Detection

```cpp
// Check if a world point is inside a box's local space
bool pointInBox(const Vector2& worldPoint, const Transform& boxTransform,
                float boxWidth, float boxHeight) {
    // Convert world point to box's local space
    Vector2 localPoint = boxTransform.inverseTransformPoint(worldPoint);

    // Check if inside box bounds (centered at origin in local space)
    float halfW = boxWidth / 2.0f;
    float halfH = boxHeight / 2.0f;

    return (std::abs(localPoint.x) <= halfW &&
            std::abs(localPoint.y) <= halfH);
}

// Usage:
Transform box(Vector2(100.0f, 100.0f), M_PI / 6.0f);  // 30° rotation
Vector2 testPoint(105.0f, 102.0f);

if (pointInBox(testPoint, box, 20.0f, 10.0f)) {
    std::cout << "Point is inside box!" << std::endl;
}
```

### Example 3: Moving Objects

```cpp
// A simple moving box
Transform box(Vector2(0.0f, 0.0f), 0.0f);
Vector2 velocity(10.0f, 5.0f);  // World space velocity
float angularVelocity = 0.1f;   // Radians per second

// Update loop (60 FPS)
float dt = 1.0f / 60.0f;

for (int frame = 0; frame < 600; ++frame) {
    // Update position and rotation
    box.translate(velocity * dt);
    box.rotate(angularVelocity * dt);

    // Normalize rotation periodically (every 60 frames)
    if (frame % 60 == 0) {
        box.normalizeRotation();
    }

    std::cout << "Frame " << frame
              << ": pos=" << box.position
              << ", rot=" << box.rotation << std::endl;
}
```

### Example 4: Following a Path

```cpp
// Make object follow waypoints, always looking at next waypoint
std::vector<Vector2> waypoints = {
    Vector2(0.0f, 0.0f),
    Vector2(100.0f, 0.0f),
    Vector2(100.0f, 100.0f),
    Vector2(0.0f, 100.0f)
};

Transform follower(waypoints[0], 0.0f);
int currentWaypoint = 1;
float speed = 50.0f;  // Units per second

float dt = 1.0f / 60.0f;

while (currentWaypoint < waypoints.size()) {
    Vector2 target = waypoints[currentWaypoint];

    // Look at target
    follower.lookAt(target);

    // Move toward target
    Vector2 direction = follower.getForward();  // Already pointing at target
    follower.translate(direction * speed * dt);

    // Check if reached target
    if (follower.position.distance(target) < 1.0f) {
        currentWaypoint++;
        std::cout << "Reached waypoint " << currentWaypoint << std::endl;
    }
}
```

### Example 5: Parent-Child Relationships

```cpp
// Create a car with rotating wheels
Transform car(Vector2(100.0f, 100.0f), M_PI / 4.0f);  // Car at (100,100), facing NE

// Front-left wheel position in car's local space
Transform wheelLocal(Vector2(10.0f, 5.0f), 0.0f);  // 10 forward, 5 left
float wheelRotation = 0.0f;  // Wheel's own rotation

// Update loop
float dt = 1.0f / 60.0f;
float wheelSpeed = 5.0f;  // Radians per second

for (int i = 0; i < 180; ++i) {  // 3 seconds at 60 FPS
    // Car moves and rotates
    Vector2 carVelocity = car.getForward() * 20.0f;  // Move forward at 20 units/s
    car.translate(carVelocity * dt);
    car.rotate(0.05f * dt);  // Turn slowly

    // Wheel spins
    wheelRotation += wheelSpeed * dt;

    // Get wheel position in world
    Transform wheel = wheelLocal.combine(car);
    wheel.rotation += wheelRotation;  // Add wheel's own rotation

    std::cout << "Car: " << car.position << " | Wheel: " << wheel.position << std::endl;
}
```


---

## Performance Considerations

### 1. Cache Sine and Cosine

```cpp
// Slow: Calculates sin/cos multiple times
for (int i = 0; i < 100; ++i) {
    Vector2 p = transform.transformPoint(points[i]);
}
// transformPoint calls rotate(), which calls sin/cos each time

// Fast: Pre-calculate rotation matrix
float c = std::cos(transform.rotation);
float s = std::sin(transform.rotation);
for (int i = 0; i < 100; ++i) {
    Vector2& p = points[i];
    points[i] = transform.position + Vector2(
        p.x * c - p.y * s,
        p.x * s + p.y * c
    );
}
```

### 2. Avoid Inverse When Possible

```cpp
// Slow:
Transform inverse = transform.inverse();
Vector2 local = inverse.transformPoint(world);

// Fast (if only doing once):
Vector2 local = transform.inverseTransformPoint(world);
// Slightly faster because no Transform object created
```

### 3. Normalize Periodically, Not Always

```cpp
// Slow: normalize every frame
for (int i = 0; i < 10000; ++i) {
    body.rotation += 0.001f;
    body.normalizeRotation();  // Expensive modulo operation
}

// Fast: normalize occasionally
for (int i = 0; i < 10000; ++i) {
    body.rotation += 0.001f;
    if (i % 100 == 0) {  // Every 100 frames
        body.normalizeRotation();
    }
}
```

### 4. Use Forward Vector for Movement

```cpp
// Slow: recalculate forward each time
velocity = Vector2(std::cos(rotation), std::sin(rotation)) * speed;

// Fast: use built-in
velocity = transform.getForward() * speed;
// Still calculates sin/cos, but intention is clearer
```

---

## Common Pitfalls

### 1. Wrong Order in Transform Combination

```cpp
// WRONG:
Transform childWorld = parent.combine(child);
// This applies parent first, then child!

// CORRECT:
Transform childWorld = child.combine(parent);
// Apply child transform in parent's space
```

**Memory trick**: Read right-to-left like function composition:
`f(g(x))` means "apply g, then f"
`child.combine(parent)` means "apply child, then parent"

### 2. Transforming Direction as Point

```cpp
Vector2 velocity(10.0f, 0.0f);

// WRONG: This adds position to velocity!
velocity = transform.transformPoint(velocity);

// CORRECT: Only rotate, don't translate
velocity = transform.transformDirection(velocity);
```

### 3. Forgetting to Normalize Angles

```cpp
// After many updates, rotation becomes huge
for (int i = 0; i < 100000; ++i) {
    transform.rotate(0.1f);
}
// rotation is now ~10000 radians! Causes precision loss.

// BETTER: Normalize periodically
for (int i = 0; i < 100000; ++i) {
    transform.rotate(0.1f);
    if (i % 1000 == 0) {
        transform.normalizeRotation();
    }
}
```

### 4. Using Degrees Instead of Radians

```cpp
// WRONG: Angles are in radians, not degrees!
Transform t(Vector2::zero(), 90.0f);  // Actually 90 radians (~5156°)

// CORRECT:
Transform t(Vector2::zero(), M_PI / 2.0f);  // 90 degrees in radians

// Or use a helper:
constexpr float DEG_TO_RAD = M_PI / 180.0f;
Transform t(Vector2::zero(), 90.0f * DEG_TO_RAD);
```

### 5. Combining Transforms in Wrong Space

```cpp
// Scenario: Child is at (5, 0) in parent's local space
Transform parent(Vector2(10.0f, 0.0f), M_PI / 2.0f);
Transform child(Vector2(5.0f, 0.0f), 0.0f);

// WRONG: Adding positions in different coordinate systems
Vector2 wrong = parent.position + child.position;  // (15, 0) - not correct!

// CORRECT: Combine transforms properly
Transform childWorld = child.combine(parent);
Vector2 correct = childWorld.position;  // (10, 5) ✓
```

---

## Summary

### What We Implemented
- Complete Transform class for 2D position and rotation
- Point and direction transformation (local ↔ world)
- Transform combination (parent-child)
- Inverse transforms
- Directional vectors (forward, right, up)
- Angle normalization
- 30 comprehensive tests (all passing)

### Files Created
1. `cpp/include/math/Transform.hpp` - Interface (260 lines)
2. `cpp/src/math/Transform.cpp` - Implementation (162 lines)
3. `tests/cpp/test_transform.cpp` - Test suite (390 lines)
4. `docs/02_Transform_Documentation.md` - This document
