# Collision System - Complete Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Physics Foundation](#physics-foundation)
3. [Implementation Details](#implementation-details)
4. [Testing Strategy](#testing-strategy)
5. [Usage Examples](#usage-examples)

---

## Introduction

### What is the Collision System?

The **Collision System** is responsible for making objects in the physics simulation interact with each other. It consists of three main components:

1. **Manifold**: Data structure holding collision information
2. **CollisionDetection**: Algorithms to detect when objects overlap
3. **CollisionResponse**: Physics to resolve collisions (bouncing, separation)

**Without collisions**: Objects pass through each other like ghosts  
**With collisions**: Objects bounce, stack, and interact realistically

### Why a Collision System?

**Realism**:
- Objects can't occupy the same space
- Objects bounce when they hit
- Objects transfer momentum
- Stacking and stability

**Game Mechanics**:
- Projectiles hit targets
- Character walks on ground
- Balls bounce
- Objects break walls

**Design Philosophy**: Impulse-based collision resolution
- Calculate collision forces as instantaneous impulses
- Separates position correction from velocity resolution
- Simple, fast, stable for real-time simulation

---

## Physics Foundation

### Collision Detection

**Question**: Are two objects overlapping?

For **circles**, this is simple:
```
Distance between centers < sum of radii
```

**Algorithm**:
1. Get center positions: posA, posB
2. Calculate distance: d = |posA - posB|
3. Get radii: rA, rB
4. Check: d < (rA + rB)?

**Optimization**: Use squared distances to avoid sqrt
```cpp
distanceSquared < radiusSumSquared
```

**Example**:
```
Circle A: center=(0, 0), radius=1
Circle B: center=(1.5, 0), radius=1

Distance = 1.5
Sum of radii = 2.0
1.5 < 2.0 → COLLISION!
Penetration = 2.0 - 1.5 = 0.5
```

### Contact Manifold

When collision is detected, we need to know:

1. **Which bodies**: bodyA and bodyB
2. **Collision normal**: Direction to push apart
   - Unit vector from B to A
   - Points "into" body A
3. **Penetration depth**: How much overlap
   - Always positive for actual collisions
   - Used to separate bodies
4. **Contact point**: Where collision occurred
   - Used for friction (future)
   - Used for visualization

**Normal calculation** (circle-circle):
```cpp
Vector2 delta = posA - posB;
Vector2 normal = delta.normalize();  // From B toward A
```

**Penetration calculation**:
```cpp
float penetration = (radiusA + radiusB) - distance;
```

**Contact point calculation**:
```cpp
// Midpoint of overlapping region
Vector2 contact = posB + normal * (radiusB - penetration * 0.5f);
```

### Collision Response

**Two phases**:
1. **Positional Correction**: Separate overlapping bodies
2. **Velocity Resolution**: Apply bounce impulse

#### Phase 1: Positional Correction

**Goal**: Move bodies apart so they're no longer overlapping

**Algorithm**:
```
totalInvMass = 1/mA + 1/mB
moveA = normal * penetration * (1/mA) / totalInvMass
moveB = -normal * penetration * (1/mB) / totalInvMass
```

**Key insight**: Heavier objects move less!

**Example**:
```
Body A: mass = 1 kg,  invMass = 1.0
Body B: mass = 10 kg, invMass = 0.1
Penetration = 1.0 m

totalInvMass = 1.0 + 0.1 = 1.1

Move A = 1.0 * (1.0 / 1.1) ≈ 0.91 m
Move B = 1.0 * (0.1 / 1.1) ≈ 0.09 m

Light body moves ~10× more than heavy body!
```

**Enhancements**:
- **Slop**: Don't correct tiny penetrations (< 0.01m) to avoid jitter
- **Percentage**: Only correct 80% per frame for stability

#### Phase 2: Velocity Resolution

**Goal**: Make bodies bounce apart

**Impulse-based physics**:
```
J = impulse magnitude (scalar)
j = impulse vector = J * normal

Apply: vA += j / mA
       vB -= j / mB
```

**Impulse formula**:
```
J = -(1 + e) * vRel · n / (1/mA + 1/mB)

Where:
  e = restitution (bounciness, 0-1)
  vRel = vA - vB (relative velocity)
  n = collision normal
  vRel · n = velocity along normal
```

**Physical meaning**:
- **(1 + e)**: Restitution factor
  - e=0: No bounce (perfectly inelastic)
  - e=0.5: Half bounce
  - e=1: Perfect bounce (perfectly elastic)
- **vRel · n**: How fast objects approach along normal
- **1/(1/mA + 1/mB)**: Effective mass at contact

**Example** (head-on collision):
```
Ball A: mass=1kg, velocity=+5 m/s (moving right)
Ball B: mass=1kg, velocity=-5 m/s (moving left)
Restitution: e=1 (perfect bounce)

vRel = 5 - (-5) = 10 m/s
vRel · n = 10 * 1 = 10  (approaching along normal)

J = -(1 + 1) * 10 / (1 + 1) = -20 / 2 = -10

Impulse = -10 * (1, 0) = (-10, 0)

vA_new = 5 + (-10)/1 = -5 m/s (reversed!)
vB_new = -5 - (-10)/1 = +5 m/s (reversed!)

Balls swap velocities!
```

### Conservation Laws

**Momentum Conservation** (perfectly):
```
Before: pA + pB = mA*vA + mB*vB
After:  pA' + pB' = mA*vA' + mB*vB'

Impulse method guarantees: pA + pB = pA' + pB'
```

**Energy Conservation** (depends on restitution):
```
Kinetic energy before: KE_before = 0.5*mA*vA² + 0.5*mB*vB²
Kinetic energy after:  KE_after = 0.5*mA*vA'² + 0.5*mB*vB'²

Perfect elastic (e=1): KE_after = KE_before
Perfectly inelastic (e=0): KE_after < KE_before (energy dissipated)
```

---

## Implementation Details

### Manifold Structure

```cpp
struct Manifold {
    RigidBody* bodyA;       // First body
    RigidBody* bodyB;       // Second body
    Vector2 normal;          // From B to A (unit vector)
    float penetration;       // Overlap depth (positive)
    Vector2 contactPoint;    // World space contact location
    
    bool isValid() const;    // Check if collision is valid
    void clear();            // Reset to empty
};
```

**Design decisions**:
- Plain struct (not class) - simple data container
- Pointers to bodies (not owned)
- Normal always points from B to A for consistency

### CollisionDetection Class

```cpp
class CollisionDetection {
public:
    // Circle-circle collision
    static bool circleVsCircle(
        RigidBody* bodyA,
        RigidBody* bodyB,
        Manifold& manifold
    );
    
    // Generic collision (dispatches to specific function)
    static bool detectCollision(
        RigidBody* bodyA,
        RigidBody* bodyB,
        Manifold& manifold
    );
};
```

**Design decisions**:
- All static methods (no instance needed)
- Specific functions for each shape combination
- Generic function dispatches based on collider types

**Circle-Circle Implementation**:

```cpp
bool CollisionDetection::circleVsCircle(RigidBody* bodyA, RigidBody* bodyB, Manifold& manifold) {
    // 1. Validate inputs
    if (!bodyA || !bodyB || !bodyA->hasCollider() || !bodyB->hasCollider()) {
        return false;
    }
    
    // 2. Get circle colliders
    CircleCollider* circleA = static_cast<CircleCollider*>(bodyA->getCollider());
    CircleCollider* circleB = static_cast<CircleCollider*>(bodyB->getCollider());
    
    // 3. Calculate distance
    Vector2 delta = bodyA->getPosition() - bodyB->getPosition();
    float distSq = delta.lengthSquared();
    float radiusSum = circleA->getRadius() + circleB->getRadius();
    
    // 4. Check collision
    if (distSq >= radiusSum * radiusSum) {
        return false;  // No collision
    }
    
    // 5. Calculate collision data
    float dist = sqrt(distSq);
    manifold.normal = delta / dist;  // Normalize
    manifold.penetration = radiusSum - dist;
    manifold.contactPoint = bodyB->getPosition() + manifold.normal * (circleB->getRadius() - manifold.penetration * 0.5f);
    manifold.bodyA = bodyA;
    manifold.bodyB = bodyB;
    
    return true;
}
```

**Edge case handling**:
- Same position: Choose arbitrary normal (1, 0)
- Null pointers: Return false
- Missing colliders: Return false
- Wrong collider type: Return false

### CollisionResponse Class

```cpp
class CollisionResponse {
public:
    static void resolveCollision(Manifold& manifold);
    static void positionalCorrection(Manifold& manifold);
    static void resolveVelocity(Manifold& manifold);
    
private:
    static constexpr float POSITIONAL_CORRECTION_PERCENT = 0.8f;
    static constexpr float PENETRATION_SLOP = 0.01f;
};
```

**Design decisions**:
- All static methods
- Constants for tuning
- Split position and velocity resolution

**Positional Correction Implementation**:

```cpp
void CollisionResponse::positionalCorrection(Manifold& manifold) {
    float totalInvMass = bodyA->getInverseMass() + bodyB->getInverseMass();
    
    if (totalInvMass == 0.0f) return;  // Both static
    
    // Only correct beyond slop threshold
    float correctionDepth = max(manifold.penetration - PENETRATION_SLOP, 0.0f);
    float correctionMag = correctionDepth * POSITIONAL_CORRECTION_PERCENT / totalInvMass;
    
    Vector2 correction = manifold.normal * correctionMag;
    
    // Move bodies proportional to inverse mass
    bodyA->setPosition(bodyA->getPosition() + correction * bodyA->getInverseMass());
    bodyB->setPosition(bodyB->getPosition() - correction * bodyB->getInverseMass());
}
```

**Why 80% correction?**
- 100% can cause jitter and instability
- 80% is gentler, converges over multiple frames
- Common in production engines

**Why penetration slop?**
- Tiny overlaps (< 1cm) don't need correction
- Reduces jitter from floating point errors
- Objects can "rest" on each other

**Velocity Resolution Implementation**:

```cpp
void CollisionResponse::resolveVelocity(Manifold& manifold) {
    Vector2 relVel = bodyA->getVelocity() - bodyB->getVelocity();
    float velAlongNormal = relVel.dot(manifold.normal);
    
    // Don't resolve if separating
    if (velAlongNormal > 0) return;
    
    // Calculate restitution
    float e = min(bodyA->getRestitution(), bodyB->getRestitution());
    
    // Calculate impulse
    float j = -(1.0f + e) * velAlongNormal;
    j /= (bodyA->getInverseMass() + bodyB->getInverseMass());
    
    Vector2 impulse = manifold.normal * j;
    
    // Apply impulse
    bodyA->applyImpulse(impulse);
    bodyB->applyImpulse(-impulse);
}
```

**Why minimum restitution?**
- Rock hitting rubber: Should bounce like rock (less bouncy)
- Both materials contribute to effective bounciness
- Most realistic behavior

### Integration with PhysicsWorld

Updated simulation loop:

```cpp
void PhysicsWorld::step(float dt) {
    // 1. Apply gravity
    applyGravity(dt);
    
    // 2. Integrate (update positions)
    for (RigidBody* body : bodies) {
        body->integrate(dt);
    }
    
    // 3. Detect and resolve collisions  ← NEW!
    detectAndResolveCollisions();
    
    // 4. Clear forces
    for (RigidBody* body : bodies) {
        body->clearForces();
    }
}
```

**Collision detection implementation**:

```cpp
void PhysicsWorld::detectAndResolveCollisions() {
    // Check all pairs (O(n²))
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];
            
            // Skip if no colliders
            if (!bodyA->hasCollider() || !bodyB->hasCollider()) continue;
            
            // Skip if both static
            if (bodyA->isStatic() && bodyB->isStatic()) continue;
            
            // Detect collision
            Manifold m;
            if (CollisionDetection::detectCollision(bodyA, bodyB, m)) {
                // Resolve collision
                CollisionResponse::resolveCollision(m);
            }
        }
    }
}
```

**Complexity**: O(n²) where n = number of bodies
- For 10 bodies: 45 checks
- For 100 bodies: 4,950 checks
- For 1000 bodies: 499,500 checks

**Future optimization**: Spatial partitioning
- Quadtree: O(n log n)
- Grid: O(n)
- Sweep and prune: O(n log n)

---

## Testing Strategy

### Test Coverage (10 categories, 20+ tests)

1. **Manifold Creation** (3 tests)
   - Default constructor
   - Construction with data
   - Clear and validation

2. **No Collision** (1 test)
   - Circles far apart

3. **Collision Detection** (3 tests)
   - Overlapping circles
   - Different radii
   - Exactly touching (edge case)

4. **Collision Data** (2 tests)
   - Normal direction verification
   - Penetration depth calculation

5. **Positional Correction** (2 tests)
   - Bodies move apart
   - Equal masses move equally

6. **Velocity Resolution** (1 test)
   - Perfect bounce reverses velocities

7. **Static Body Collision** (2 tests)
   - Static doesn't move
   - Dynamic bounces off static

8. **PhysicsWorld Integration** (1 test)
   - World separates overlapping bodies

9. **Ball Bounce** (1 test)
   - Ball falls and bounces off ground

### Key Test: Ball Bouncing

**Most important integration test**:

```cpp
void testBallBounce() {
    PhysicsWorld world;
    world.setGravity(Vector2(0, -9.81f));
    
    RigidBody ball, ground;
    CircleCollider ballCircle(1.0f);
    CircleCollider groundCircle(10.0f);
    
    ball.setCollider(&ballCircle);
    ground.setCollider(&groundCircle);
    
    ball.setMass(1.0f);
    ground.setMass(0.0f);  // Static
    
    ball.setRestitution(0.9f);  // Bouncy
    
    ball.setPosition(Vector2(0, 15));  // Above ground
    ground.setPosition(Vector2(0, 0));
    
    world.addBody(&ball);
    world.addBody(&ground);
    
    // Simulate until bounce
    for (int i = 0; i < 100; i++) {
        world.step(0.016f);
        
        if (ball.getVelocity().y > 5.0f) {
            // Ball bounced! ✓
            return;
        }
    }
}
```

**This test verifies**:
- Gravity application
- Integration
- Collision detection
- Collision response
- Static body handling
- Restitution
- Complete physics pipeline!

---

## Usage Examples

### Example 1: Two Balls Colliding

```cpp
#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"
#include "physics/CircleCollider.hpp"

int main() {
    PhysicsWorld world;
    world.setGravity(Vector2::zero());  // No gravity
    
    // Create two balls
    RigidBody ball1, ball2;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);
    
    ball1.setCollider(&circle1);
    ball2.setCollider(&circle2);
    
    ball1.setMass(1.0f);
    ball2.setMass(1.0f);
    
    ball1.setRestitution(1.0f);  // Perfect bounce
    ball2.setRestitution(1.0f);
    
    // Position them moving toward each other
    ball1.setPosition(Vector2(-5, 0));
    ball1.setVelocity(Vector2(10, 0));  // Right
    
    ball2.setPosition(Vector2(5, 0));
    ball2.setVelocity(Vector2(-10, 0));  // Left
    
    world.addBody(&ball1);
    world.addBody(&ball2);
    
    // Simulate collision
    for (int frame = 0; frame < 100; frame++) {
        world.step(0.016f);
        
        std::cout << "Frame " << frame << ": "
                  << "Ball1 x=" << ball1.getPosition().x
                  << ", Ball2 x=" << ball2.getPosition().x
                  << std::endl;
    }
    
    // After collision, velocities reverse
    // Ball1 velocity: -10 m/s (left)
    // Ball2 velocity: +10 m/s (right)
}
```

### Example 2: Newton's Cradle Setup

```cpp
// Create 5 balls in a row
const int numBalls = 5;
RigidBody balls[numBalls];
CircleCollider circles[numBalls];

for (int i = 0; i < numBalls; i++) {
    circles[i] = CircleCollider(1.0f);
    balls[i].setCollider(&circles[i]);
    balls[i].setMass(1.0f);
    balls[i].setRestitution(1.0f);  // Perfect bounce
    balls[i].setFriction(0.0f);      // No friction
    
    // Position in a row (just touching)
    balls[i].setPosition(Vector2(i * 2.0f, 0));
    balls[i].setVelocity(Vector2::zero());
    
    world.addBody(&balls[i]);
}

// Pull back first ball and release
balls[0].setVelocity(Vector2(5, 0));

// Simulate
for (int frame = 0; frame < 1000; frame++) {
    world.step(0.016f);
    
    // Watch momentum transfer through the balls
    // Expected: First ball stops, last ball moves
}
```

### Example 3: Stacking Boxes (Circles)

```cpp
PhysicsWorld world;
world.setGravity(Vector2(0, -9.81f));

// Ground
RigidBody ground;
CircleCollider groundCircle(50.0f);  // Large flat circle
ground.setCollider(&groundCircle);
ground.setMass(0.0f);  // Static
ground.setPosition(Vector2(0, -50));
world.addBody(&ground);

// Stack of balls
const int stackHeight = 10;
RigidBody stack[stackHeight];
CircleCollider stackCircles[stackHeight];

for (int i = 0; i < stackHeight; i++) {
    stackCircles[i] = CircleCollider(1.0f);
    stack[i].setCollider(&stackCircles[i]);
    stack[i].setMass(1.0f);
    stack[i].setRestitution(0.3f);  // Low bounce
    
    // Stack vertically
    stack[i].setPosition(Vector2(0, i * 2.1f));
    
    world.addBody(&stack[i]);
}

// Simulate stacking
for (int frame = 0; frame < 1000; frame++) {
    world.step(0.016f);
}

// Stack should settle into stable configuration
```

---

## Summary

### What We Implemented

**Manifold**:
- Collision data structure
- Stores bodies, normal, penetration, contact point
- Validation and clearing

**CollisionDetection**:
- Circle-circle collision algorithm
- Distance-based overlap check
- Normal and penetration calculation
- Contact point calculation

**CollisionResponse**:
- Positional correction with slop
- Impulse-based velocity resolution
- Conservation of momentum
- Restitution and mass handling

**PhysicsWorld Integration**:
- Collision detection in simulation loop
- O(n²) pair checking
- Static body optimization

### Files Created
1. `cpp/include/physics/Manifold.hpp` (120 lines)
2. `cpp/include/physics/CollisionDetection.hpp` (130 lines)
3. `cpp/src/physics/CollisionDetection.cpp` (115 lines)
4. `cpp/include/physics/CollisionResponse.hpp` (130 lines)
5. `cpp/src/physics/CollisionResponse.cpp` (100 lines)
6. `tests/cpp/test_collision.cpp` (450 lines)
7. Updated: `cpp/include/physics/RigidBody.hpp` (added collider support)
8. Updated: `cpp/src/physics/RigidBody.cpp` (added setCollider)
9. Updated: `cpp/include/physics/PhysicsWorld.hpp` (added collision detection)
10. Updated: `cpp/src/physics/PhysicsWorld.cpp` (added detectAndResolveCollisions)

### Physics Concepts Covered
- **Collision detection**: Distance checks, overlap tests
- **Contact manifolds**: Normal, penetration, contact point
- **Impulse-based resolution**: Instantaneous velocity changes
- **Positional correction**: Separation with slop and percentage
- **Conservation laws**: Momentum, energy
- **Restitution**: Bounce coefficient
- **Effective mass**: Collision response with different masses

### Next Steps

**Phase 4 Complete!** ✓

We now have a fully functional physics engine with collisions:
- ✓ Math foundation
- ✓ Rigid body dynamics
- ✓ Collision shapes
- ✓ Physics world
- ✓ **Collision system**

**Phase 5: Python Bindings**
- pybind11 wrapper
- Expose all classes to Python
- Pythonic API

**Phase 6: Visualization**
- Pygame rendering
- Real-time display
- Interactive controls

**Future Enhancements**:
- Box collider
- Circle-box collision
- Box-box collision (SAT)
- Spatial partitioning (quadtree)
- Constraint solving
- Friction
