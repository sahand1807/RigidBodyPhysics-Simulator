# Collider & CircleCollider - Complete Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Physical Foundation](#physical-foundation)
3. [Implementation Details](#implementation-details)
4. [Testing Strategy](#testing-strategy)
5. [Usage Examples](#usage-examples)

---

## Introduction

### What is a Collider?

A **Collider** defines the **shape** of a RigidBody for collision detection and mass property calculations. It's separate from RigidBody because:

**Separation of Concerns**:
- **RigidBody**: Handles motion (velocity, forces, integration)
- **Collider**: Handles shape (geometry, mass properties)

**Design Pattern**: Abstract Base Class with polymorphism
- Different shapes (Circle, Box, Polygon) implement the same interface
- RigidBody doesn't need to know which shape it has
- Easy to add new shapes without modifying existing code

### Why CircleCollider?

Circles are the simplest and fastest collision shape:

**Advantages**:
- **Fastest collision detection**: Just compare distances
- **Rotationally symmetric**: Looks the same from any angle
- **Natural physics**: Rolls and bounces realistically
- **Simple math**: Basic formulas for mass and inertia

**Use cases**:
- Balls, wheels, coins
- Projectiles (bullets, arrows)
- Character controllers (capsule = circle + circle)
- Simple approximations of complex shapes

---

## Physical Foundation

### 2D Circle Formulas

#### Area
```
A = π × r²
```

**Example**:
```
r = 1m → A = π × 1² ≈ 3.14 m²
r = 2m → A = π × 4 ≈ 12.57 m²
```

**Scaling**: Area ∝ r² (doubling radius → 4× area)

#### Mass (from Density)
```
m = density × area = ρ × π × r²
```

**Typical 2D densities**:
- Wood: 0.6 kg/m²
- Rubber: 1.1 kg/m²
- Metal: 7.8 kg/m²

**Example**:
```
Wooden circle, r=1m:
m = 0.6 × π × 1² ≈ 1.88 kg
```

#### Moment of Inertia
```
I = 0.5 × m × r²
```

**Derivation** (2D disk):
```
I = ∫∫ r² dm
For uniform density in 2D:
I = (1/2) × m × r²
```

**Physical meaning**:
- Resistance to angular acceleration
- Mass far from center → larger I → harder to spin
- I ∝ m × r² (doubling mass OR radius → varies differently)

**Example**:
```
m = 2 kg, r = 1m:
I = 0.5 × 2 × 1² = 1 kg⋅m²

m = 2 kg, r = 2m:
I = 0.5 × 2 × 4 = 4 kg⋅m²  (4× harder to spin!)
```

**Scaling relationships**:
```
If r → 2r:
- Area → 4× area (r²)
- Mass → 4× mass (ρ × area)
- Inertia → 16× inertia! (0.5 × 4m × 4r²)

Inertia scales with r⁴ !
```

### Axis-Aligned Bounding Box (AABB)

**Definition**: Smallest rectangle (aligned with axes) containing the shape.

**For circles**:
```
min = center - (r, r)
max = center + (r, r)
width = height = 2r
```

**Uses**:
1. **Broad-phase collision**: Quick rejection before expensive checks
2. **Spatial partitioning**: Quadtrees, grids
3. **Frustum culling**: Don't render off-screen objects

**Example**:
```
Circle at (5, 10), radius 2:
min = (3, 8)
max = (7, 12)

Quick check: if point (15, 20) outside AABB, it's definitely not in circle
```

---

## Implementation Details

### Collider Base Class

**Design: Abstract Base Class**

```cpp
class Collider {
public:
    enum class Type { Circle, Box };

    // Pure virtual (must override)
    virtual Type getType() const = 0;
    virtual float calculateMass(float density) const = 0;
    virtual float calculateInertia(float mass) const = 0;
    virtual void getBounds(Vector2& min, Vector2& max) const = 0;

    // Virtual destructor (essential!)
    virtual ~Collider() = default;

    // Non-virtual (common to all)
    const Vector2& getOffset() const;
    void setOffset(const Vector2& offset);
    Vector2 getWorldCenter(const Transform& bodyTransform) const;

protected:
    Vector2 offset;  // Local position relative to body
};
```

**Key design decisions**:

1. **Pure virtual functions** (`= 0`):
   - Must be implemented by derived classes
   - Makes Collider abstract (cannot instantiate)
   - Forces consistent interface

2. **Virtual destructor**:
   ```cpp
   Collider* c = new CircleCollider(1.0f);
   delete c;  // Without virtual ~Collider(), memory leak!
   ```

3. **Protected offset**:
   - Derived classes can access it
   - External code uses getters/setters
   - Encapsulation while allowing inheritance

### CircleCollider Implementation

```cpp
class CircleCollider : public Collider {
private:
    float radius;

public:
    CircleCollider(float radius, const Vector2& offset = Vector2::zero());

    // Implement pure virtual
    Type getType() const override { return Type::Circle; }

    float calculateMass(float density) const override {
        return density * M_PI * radius * radius;
    }

    float calculateInertia(float mass) const override {
        return 0.5f * mass * radius * radius;
    }

    void getBounds(Vector2& min, Vector2& max) const override {
        Vector2 r(radius, radius);
        min = offset - r;
        max = offset + r;
    }

    // Circle-specific
    float getRadius() const { return radius; }
    bool containsPoint(const Vector2& point) const;
};
```

**Validation in constructor**:
```cpp
CircleCollider::CircleCollider(float radius, const Vector2& offset)
    : Collider(offset), radius(radius)
{
    if (radius <= 0.0f) {
        this->radius = 1.0f;  // Safe default
    }
}
```

### Polymorphism in Action

```cpp
// Create different shapes through base class pointers
Collider* shapes[2];
shapes[0] = new CircleCollider(1.0f);
shapes[1] = new BoxCollider(2.0f, 3.0f);  // Future!

// Use polymorphically
for (Collider* shape : shapes) {
    float mass = shape->calculateMass(1.0f);  // Calls correct version!
    // Circle uses π × r²
    // Box uses w × h
}
```

**How it works**:
- **Virtual function table (vtable)**: Each object has pointer to function table
- **Runtime dispatch**: Calls resolved at runtime based on actual type
- **Overhead**: One pointer per object + one indirection per virtual call
- **Worth it**: Flexibility and extensibility

### Offset System

**Purpose**: Allow collider to be positioned away from body's center.

**Use cases**:
1. **Asymmetric objects**: Car with offset wheels
2. **Compound shapes**: Multiple colliders on one body
3. **Fine-tuning**: Adjust hitbox without moving body

**Implementation**:
```cpp
Vector2 Collider::getWorldCenter(const Transform& bodyTransform) const {
    // Transform local offset to world space
    return bodyTransform.transformPoint(offset);
}
```

**Example**:
```cpp
RigidBody car;
car.setPosition(Vector2(10, 20));
car.setRotation(M_PI / 4);  // 45°

CircleCollider wheel(0.5f, Vector2(2, 0));  // 2m to the right

Vector2 wheelWorld = wheel.getWorldCenter(car.getTransform());
// offset (2, 0) rotated 45° + car position
// ≈ (10 + 1.41, 20 + 1.41) ≈ (11.41, 21.41)
```

---

## Testing Strategy

### Test Coverage (9 categories, 30+ tests)

1. **Construction** (3 tests)
   - Basic circle creation
   - Circle with offset
   - Invalid radius handling

2. **Mass Calculation** (3 tests)
   - Verify formula: m = ρ × π × r²
   - Different densities
   - Scaling relationship (r²)

3. **Inertia Calculation** (3 tests)
   - Verify formula: I = 0.5 × m × r²
   - Different sizes
   - Scaling relationship (r⁴!)

4. **Bounds Calculation** (3 tests)
   - Circle at origin
   - Circle with offset
   - AABB size verification

5. **Point Containment** (6 tests)
   - Center point
   - Boundary point
   - Interior point
   - Exterior points
   - Offset circles

6. **Offset Functionality** (3 tests)
   - Get/set offset
   - Bounds update with offset
   - Independent of radius

7. **World Center** (4 tests)
   - No transform
   - Position only
   - Rotation only
   - Combined transform

8. **Radius Modification** (4 tests)
   - Set new radius
   - Mass recalculation
   - Invalid radius rejection
   - Zero radius rejection

9. **Polymorphism** (4 tests)
   - Type identification
   - Virtual function calls
   - Downcasting
   - Polymorphic deletion

### Key Test: Inertia Scaling

**Most important insight from tests**:

```cpp
CircleCollider small(1.0f);  // r = 1
CircleCollider large(2.0f);  // r = 2

float m1 = small.calculateMass(1.0f);   // π kg
float I1 = small.calculateInertia(m1);  // 0.5π kg⋅m²

float m2 = large.calculateMass(1.0f);   // 4π kg
float I2 = large.calculateInertia(m2);  // 8π kg⋅m²

// Mass ratio: 4× (r²)
// Inertia ratio: 16× (r⁴)!
assert(I2 / I1 ≈ 16.0);
```

**Why r⁴?**
- Mass ∝ r² (from area)
- Inertia = 0.5 × mass × r²
- Inertia ∝ r² × r² = r⁴

---

## Usage Examples

### Example 1: Creating Bodies with Circles

```cpp
#include "physics/RigidBody.hpp"
#include "physics/CircleCollider.hpp"

// Create a ball
RigidBody* ball = new RigidBody();
CircleCollider* circle = new CircleCollider(1.0f);  // 1m radius

// Calculate mass properties from shape
float density = 0.6f;  // Wood
float mass = circle->calculateMass(density);
float inertia = circle->calculateInertia(mass);

// Configure body
ball->setMass(mass);
ball->setMomentOfInertia(inertia);
ball->setPosition(Vector2(0, 10));
// ball->attachCollider(circle);  // Future: when we add this method

std::cout << "Ball mass: " << mass << " kg" << std::endl;
// Output: Ball mass: 1.88496 kg
```

### Example 2: Different Sizes

```cpp
// Small ball (marble)
CircleCollider marble(0.01f);  // 1cm radius
float marbleMass = marble.calculateMass(7.8f);  // Metal
std::cout << "Marble: " << marbleMass << " kg" << std::endl;
// Output: Marble: 0.00245 kg (2.45 grams)

// Large ball (boulder)
CircleCollider boulder(2.0f);  // 2m radius
float boulderMass = boulder.calculateMass(2.5f);  // Rock
std::cout << "Boulder: " << boulderMass << " kg" << std::endl;
// Output: Boulder: 31.416 kg
```

### Example 3: Wheel with Offset

```cpp
// Car body
RigidBody car;
car.setPosition(Vector2(100, 50));

// Front-right wheel (offset from car center)
CircleCollider frontWheel(0.4f, Vector2(1.5f, -0.5f));
// 1.5m forward, 0.5m down from car center

// Get wheel's world position
Vector2 wheelPos = frontWheel.getWorldCenter(car.getTransform());
std::cout << "Wheel at: " << wheelPos << std::endl;
// Output: Wheel at: Vector2(101.5, 49.5)
```

### Example 4: Point Containment Check

```cpp
CircleCollider circle(2.0f, Vector2(5, 5));

// Check if projectile hit
Vector2 bulletPos(6.5f, 5.0f);

if (circle.containsPoint(bulletPos)) {
    std::cout << "Hit!" << std::endl;
    // Apply damage, remove bullet, etc.
} else {
    std::cout << "Miss!" << std::endl;
}
```

### Example 5: AABB for Broad-Phase

```cpp
// Get bounding boxes for all objects
std::vector<RigidBody*> bodies;
// ... add bodies ...

for (RigidBody* body : bodies) {
    CircleCollider* circle = /* get from body */;

    Vector2 min, max;
    circle->getBounds(min, max);

    // Transform to world space
    Transform& t = body->getTransform();
    min = t.transformPoint(min);
    max = t.transformPoint(max);

    // Quick AABB overlap check
    for (RigidBody* other : bodies) {
        if (body == other) continue;

        Vector2 otherMin, otherMax;
        // ... get other's bounds ...

        // AABB overlap test (cheap!)
        bool overlap = !(max.x < otherMin.x || min.x > otherMax.x ||
                        max.y < otherMin.y || min.y > otherMax.y);

        if (overlap) {
            // Potential collision → do expensive circle-circle test
        }
    }
}
```

### Example 6: Polymorphic Shape Usage

```cpp
void setupPhysicsBody(RigidBody* body, Collider* collider, float density) {
    // Works for ANY collider type!
    float mass = collider->calculateMass(density);
    float inertia = collider->calculateInertia(mass);

    body->setMass(mass);
    body->setMomentOfInertia(inertia);

    std::cout << "Created " << (collider->getType() == Collider::Type::Circle ?
                                "circle" : "box")
              << " with mass " << mass << " kg" << std::endl;
}

// Use with circle
setupPhysicsBody(ball, new CircleCollider(1.0f), 1.0f);

// Future: use with box
setupPhysicsBody(crate, new BoxCollider(2.0f, 3.0f), 1.0f);
```

---

## Summary

### What We Implemented

**Collider Base Class**:
- Abstract interface for all collision shapes
- Pure virtual functions for polymorphism
- Offset system for flexible positioning
- World space transformation support

**CircleCollider**:
- Concrete circle implementation
- Mass formula: m = ρ × π × r²
- Inertia formula: I = 0.5 × m × r²
- AABB calculation
- Point containment test
- Radius validation

### Files Created
1. `cpp/include/physics/Collider.hpp` (interface, 196 lines)
2. `cpp/src/physics/Collider.cpp` (implementation, 28 lines)
3. `cpp/include/physics/CircleCollider.hpp` (interface, 181 lines)
4. `cpp/src/physics/CircleCollider.cpp` (implementation, 78 lines)
5. `tests/cpp/test_collider.cpp` (test suite, 415 lines)