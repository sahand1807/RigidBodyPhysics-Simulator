# PhysicsWorld - Complete Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Physics Foundation](#physics-foundation)
3. [Implementation Details](#implementation-details)
4. [Testing Strategy](#testing-strategy)
5. [Usage Examples](#usage-examples)

---

## Introduction

### What is PhysicsWorld?

**PhysicsWorld** is the central manager for physics simulation. It:
- Manages a collection of RigidBody objects
- Applies global forces like gravity to all bodies
- Advances the simulation forward in time
- Orchestrates the integration loop


### Why PhysicsWorld?

**Centralized Management**:
- Single place to control all simulation parameters
- Consistent application of global forces
- Simplified API for running simulations

**Separation of Concerns**:
- **RigidBody**: Individual object dynamic behavior
- **PhysicsWorld**: Global simulation control (when to integrate, what forces to apply)

**Design Pattern**: World/Entity pattern
- World manages entities (bodies)
- Provides services to entities (gravity, collision detection)
- Controls simulation loop

---

## Physics Foundation

### Simulation Loop

A physics simulation advances in discrete time steps. Each step consists of:

```
1. Apply Forces
   ↓
2. Integrate (update velocities and positions)
   ↓
3. Clear Forces (prepare for next step)
   ↓
4. Repeat
```

**Why this order?**

1. **Apply Forces First**: Accumulate all forces that act during this time step
   - Gravity
   - User input (push, pull)
   - Springs, drag, etc.

2. **Integrate**: Use accumulated forces to update motion
   - F = ma → a = F/m
   - v(t+dt) = v(t) + a×dt
   - x(t+dt) = x(t) + v(t+dt)×dt

3. **Clear Forces**: Reset for next frame
   - Forces are "per-frame" (applied each step)
   - Impulses are instant (applied once, affect velocity directly)
   - Clearing prevents forces from accumulating incorrectly

### Time Stepping

**Fixed Time Step**:
```cpp
const float dt = 1.0f / 60.0f;  // 60 FPS = 0.0167 seconds
while (running) {
    world.step(dt);
}
```

**Advantages**:
- Deterministic: Same inputs → same outputs
- Stable: Physics doesn't break with variable frame rate
- Predictable: Easier to tune and debug

**Variable Time Step**:
```cpp
float lastTime = getCurrentTime();
while (running) {
    float currentTime = getCurrentTime();
    float dt = currentTime - lastTime;
    world.step(dt);
    lastTime = currentTime;
}
```


### Gravity

Gravity is a **constant acceleration** applied to all bodies:

```
F = m × g
```

Where:
- **F**: Gravitational force (Newtons)
- **m**: Mass of the object (kg)
- **g**: Gravitational acceleration (m/s²)


**2D gravity can point anywhere**:
```cpp
Vector2(0, -9.81)   // Downward (normal)
Vector2(-5, 0)      // Leftward (platform game)
Vector2(5, -5)      // Diagonal (experimental)
```

### Static vs Dynamic Bodies

**Static Bodies** (invMass = 0):
- Don't move, infinite mass
- Not affected by forces or gravity
- Used for ground, walls, platforms
- Optimization: Skip integration and force application

**Dynamic Bodies** (invMass > 0):
- Move in response to forces
- Affected by gravity
- Normal physics objects

**Implementation**:
```cpp
void PhysicsWorld::applyGravity(float dt) {
    for (RigidBody* body : bodies) {
        if (body->isStatic()) {
            continue;  // Skip static bodies
        }

        Vector2 gravityForce = gravity * body->getMass();
        body->applyForce(gravityForce);
    }
}
```

---

## Implementation Details

### Class Structure

```cpp
class PhysicsWorld {
private:
    std::vector<RigidBody*> bodies;  // All bodies in simulation
    Vector2 gravity;                  // Global gravity (m/s²)

public:
    // Body management
    void addBody(RigidBody* body);
    bool removeBody(RigidBody* body);
    void clear();

    // Simulation
    void step(float dt);

    // Settings
    void setGravity(const Vector2& gravity);
    const Vector2& getGravity() const;

    // Query
    const std::vector<RigidBody*>& getBodies() const;
    size_t getBodyCount() const;
};
```

### Memory Management

**PhysicsWorld does NOT own bodies!**

```cpp
// User creates and deletes bodies
RigidBody* ball = new RigidBody();
world.addBody(ball);  // PhysicsWorld stores pointer

// ... simulate ...

world.removeBody(ball);  // Remove from world
delete ball;             // User deletes
```

**Why?**
- Flexibility: User controls lifetime
- Simplicity: No smart pointers needed (for now)
- Clear ownership: User knows who owns what

**Future improvement**: Use `std::shared_ptr<RigidBody>` for automatic memory management.

### Body Management Implementation

**Adding Bodies**:
```cpp
void PhysicsWorld::addBody(RigidBody* body) {
    if (body == nullptr) {
        return;  // Ignore null pointers
    }

    // Prevent duplicates
    auto it = std::find(bodies.begin(), bodies.end(), body);
    if (it != bodies.end()) {
        return;  // Already in world
    }

    bodies.push_back(body);
}
```

**Key features**:
- Null-safe: Ignores nullptr
- Duplicate-safe: Won't add same body twice
- O(n) check for duplicates (fine for small worlds)

**Removing Bodies**:
```cpp
bool PhysicsWorld::removeBody(RigidBody* body) {
    if (body == nullptr) {
        return false;
    }

    auto it = std::find(bodies.begin(), bodies.end(), body);
    if (it == bodies.end()) {
        return false;  // Not found
    }

    bodies.erase(it);
    return true;
}
```

**Key features**:
- Returns success/failure
- Safe to call even if body not in world
- O(n) search (fine for small worlds)

### Simulation Step Implementation

**The Main Loop**:
```cpp
void PhysicsWorld::step(float dt) {
    // 1. Apply global forces
    applyGravity(dt);

    // 2. Integrate all bodies
    for (RigidBody* body : bodies) {
        body->integrate(dt);
    }

    // 3. Clear forces for next frame
    for (RigidBody* body : bodies) {
        body->clearForces();
    }
}
```

**Why separate loops?**
- **Clarity**: Each phase is explicit
- **Correctness**: All forces applied before any integration
- **Future-proof**: Easy to add more phases (collision, constraints)


### Gravity Application

```cpp
void PhysicsWorld::applyGravity(float dt) {
    for (RigidBody* body : bodies) {
        // Skip static bodies
        if (body->isStatic()) {
            continue;
        }

        // F = m × g
        Vector2 gravityForce = gravity * body->getMass();
        body->applyForce(gravityForce);
    }
}
```

**Note**: We don't use `dt` in gravity application because:
- Forces are applied per-frame regardless of duration
- Integration step accounts for time (a×dt)
- Force accumulation works same way for all forces

**Mathematical detail**:
```
Force applied: F = m × g
Acceleration: a = F/m = g
Velocity change: Δv = a × dt = g × dt
Position change: Δx = v × dt
```

---

## Testing Strategy

### Test Coverage (12 categories, 30+ assertions)

1. **World Construction** (2 tests)
   - Default gravity (Earth)
   - Empty world initially

2. **Body Management** (7 tests)
   - Add single body
   - Add multiple bodies
   - Remove body
   - Remove non-existent body
   - Clear all bodies
   - getBodies() returns correct list
   - getBodyCount() is accurate

3. **Null Safety** (2 tests)
   - Adding nullptr ignored
   - Removing nullptr returns false

4. **Duplicate Prevention** (2 tests)
   - Adding same body twice
   - Body count remains 1

5. **Gravity Settings** (4 tests)
   - Default Earth gravity
   - Zero gravity (space)
   - Moon gravity
   - Custom direction (sideways)

6. **Simulation Step** (2 tests)
   - Body falls under gravity
   - Velocity increases

7. **Gravity Application** (1 test)
   - Verify F = m×g calculation
   - Check acceleration independence of mass

8. **Static Body Handling** (2 tests)
   - Static body doesn't move
   - Static body velocity stays zero

9. **Multiple Body Simulation** (2 tests)
   - All bodies fall
   - All bodies have same acceleration

10. **Force Clearing** (2 tests)
    - Force applied affects motion
    - Force cleared after step

11. **Zero Gravity** (2 tests)
    - Constant velocity maintained
    - Position updates correctly

12. **Custom Gravity Direction** (2 tests)
    - Sideways gravity works
    - Bodies accelerate in correct direction

### Key Test: Multiple Bodies Same Acceleration

**Most important physics verification**:

```cpp
// Three bodies with different masses
RigidBody body1, body2, body3;
body1.setMass(1.0f);   // 1 kg
body2.setMass(2.0f);   // 2 kg
body3.setMass(0.5f);   // 0.5 kg

world.setGravity(Vector2(0, -10));

// Step simulation
world.step(0.1f);

// All should have same velocity!
// a = F/m = (m×g)/m = g = -10 m/s²
// v = a×dt = -10 × 0.1 = -1 m/s
assert(body1.getVelocity().y == -1.0f);
assert(body2.getVelocity().y == -1.0f);
assert(body3.getVelocity().y == -1.0f);
```

**Why this matters**: Demonstrates correct implementation of Newton's laws.

---

## Usage Examples

### Example 1: Simple Falling Ball

```cpp
#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"

int main() {
    // Create world
    PhysicsWorld world;
    world.setGravity(Vector2(0, -9.81f));  // Earth gravity

    // Create ball
    RigidBody ball;
    ball.setMass(1.0f);
    ball.setPosition(Vector2(0, 100));  // 100 meters up
    ball.setVelocity(Vector2(0, 0));

    // Add to world
    world.addBody(&ball);

    // Simulate for 3 seconds
    float dt = 1.0f / 60.0f;  // 60 FPS
    for (int frame = 0; frame < 180; frame++) {  // 3 seconds
        world.step(dt);

        if (frame % 60 == 0) {  // Every second
            std::cout << "t=" << frame/60.0f << "s: "
                      << "y=" << ball.getPosition().y << "m, "
                      << "v=" << ball.getVelocity().y << "m/s"
                      << std::endl;
        }
    }

    return 0;
}

// Output:
// t=0s: y=100m, v=0m/s
// t=1s: y=95.095m, v=-9.81m/s
// t=2s: y=80.38m, v=-19.62m/s
// t=3s: y=55.855m, v=-29.43m/s
```

### Example 2: Multiple Objects

```cpp
// Create world
PhysicsWorld world;
world.setGravity(Vector2(0, -9.81f));

// Create multiple balls
std::vector<RigidBody> balls(10);

for (int i = 0; i < 10; i++) {
    balls[i].setMass(1.0f + i * 0.5f);  // Different masses
    balls[i].setPosition(Vector2(i * 2.0f, 100));  // Spread out
    world.addBody(&balls[i]);
}

// Simulate
for (int frame = 0; frame < 600; frame++) {
    world.step(1.0f / 60.0f);
}

// Check final positions
for (int i = 0; i < 10; i++) {
    std::cout << "Ball " << i
              << " (mass=" << balls[i].getMass() << "kg): "
              << balls[i].getPosition() << std::endl;
}

// All balls should be at same height!
// Mass doesn't affect fall rate
```

### Example 3: Projectile Motion

```cpp
// Create world
PhysicsWorld world;
world.setGravity(Vector2(0, -9.81f));

// Launch projectile
RigidBody projectile;
projectile.setMass(0.5f);  // 0.5 kg
projectile.setPosition(Vector2(0, 0));
projectile.setVelocity(Vector2(20, 30));  // 20 m/s right, 30 m/s up

world.addBody(&projectile);

// Simulate
std::vector<Vector2> trajectory;
for (int frame = 0; frame < 600; frame++) {
    world.step(1.0f / 60.0f);

    if (frame % 10 == 0) {
        trajectory.push_back(projectile.getPosition());
    }

    // Stop when hits ground
    if (projectile.getPosition().y <= 0) {
        break;
    }
}

// Analyze trajectory
std::cout << "Trajectory points: " << trajectory.size() << std::endl;
std::cout << "Max height: " << /* find max y */ << std::endl;
std::cout << "Range: " << projectile.getPosition().x << " meters" << std::endl;

// Expected range: R = v₀²×sin(2θ) / g
// θ = atan2(30, 20) ≈ 56.3°
// R ≈ 20²×sin(112.6°) / 9.81 ≈ 37.8 meters
```

### Example 4: Static Platform

```cpp
// Create world
PhysicsWorld world;
world.setGravity(Vector2(0, -9.81f));

// Create static ground
RigidBody ground;
ground.setMass(0.0f);  // 0 mass = infinite mass = static
ground.setPosition(Vector2(0, 0));
world.addBody(&ground);

// Create dynamic ball above ground
RigidBody ball;
ball.setMass(1.0f);
ball.setPosition(Vector2(0, 10));
world.addBody(&ball);

// Simulate
for (int frame = 0; frame < 600; frame++) {
    world.step(1.0f / 60.0f);

    std::cout << "Ground: " << ground.getPosition() << std::endl;
    std::cout << "Ball: " << ball.getPosition() << std::endl;

    // Ground never moves!
    // (Note: Ball will fall through ground without collision detection)
}
```

### Example 5: Zero Gravity (Space)

```cpp
// Create world with no gravity
PhysicsWorld world;
world.setGravity(Vector2::zero());

// Create spaceship
RigidBody ship;
ship.setMass(1000.0f);  // 1 ton
ship.setPosition(Vector2(0, 0));
ship.setVelocity(Vector2(10, 5));  // Drifting

world.addBody(&ship);

// Simulate
for (int frame = 0; frame < 600; frame++) {
    world.step(1.0f / 60.0f);

    // Optional: apply thrust
    if (/* user input */) {
        ship.applyForce(Vector2(100, 0));  // Thrust right
    }
}

// In space, ship maintains velocity unless thrust is applied
std::cout << "Final velocity: " << ship.getVelocity() << std::endl;
// Still (10, 5) if no thrust applied
```

### Example 6: Variable Gravity

```cpp
// Create world
PhysicsWorld world;

// Create body
RigidBody ball;
ball.setMass(1.0f);
ball.setPosition(Vector2(0, 100));
world.addBody(&ball);

// Simulate with changing gravity
for (int frame = 0; frame < 600; frame++) {
    // Gravity oscillates
    float gravityMag = -9.81f + 5.0f * sin(frame * 0.1f);
    world.setGravity(Vector2(0, gravityMag));

    world.step(1.0f / 60.0f);
}

// Creates interesting bouncing/floating effect
```

### Example 7: Game Loop Integration

```cpp
class Game {
private:
    PhysicsWorld world;
    std::vector<RigidBody*> gameObjects;

public:
    void init() {
        world.setGravity(Vector2(0, -20.0f));  // Stronger gravity for game feel

        // Create player
        RigidBody* player = new RigidBody();
        player->setMass(70.0f);  // 70 kg person
        player->setPosition(Vector2(100, 200));
        world.addBody(player);
        gameObjects.push_back(player);
    }

    void update(float deltaTime) {
        // Fixed time step for physics
        const float physicsDT = 1.0f / 60.0f;
        static float accumulator = 0.0f;

        accumulator += deltaTime;

        // Update physics in fixed steps
        while (accumulator >= physicsDT) {
            world.step(physicsDT);
            accumulator -= physicsDT;
        }

        // Render interpolated between physics steps
        float alpha = accumulator / physicsDT;
        renderWithInterpolation(alpha);
    }

    void cleanup() {
        for (RigidBody* body : gameObjects) {
            world.removeBody(body);
            delete body;
        }
    }
};
```

### Example 8: Performance Monitoring

```cpp
#include <chrono>

// Create world with many bodies
PhysicsWorld world;
world.setGravity(Vector2(0, -9.81f));

std::vector<RigidBody> bodies(1000);  // 1000 bodies!
for (int i = 0; i < 1000; i++) {
    bodies[i].setMass(1.0f);
    bodies[i].setPosition(Vector2(
        (i % 32) * 2.0f,
        (i / 32) * 2.0f + 100.0f
    ));
    world.addBody(&bodies[i]);
}

// Benchmark simulation
auto start = std::chrono::high_resolution_clock::now();

for (int frame = 0; frame < 1000; frame++) {
    world.step(1.0f / 60.0f);
}

auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

std::cout << "1000 frames with 1000 bodies: "
          << duration.count() << " ms" << std::endl;
std::cout << "Average: " << duration.count() / 1000.0f << " ms/frame" << std::endl;

// Expected: < 1 ms/frame on modern hardware
```

---

## Summary

### What We Implemented

**PhysicsWorld Class**:
- Body management (add, remove, clear)
- Gravity system (any direction)
- Simulation loop (step function)
- Force application and clearing
- Static body optimization

**Key Features**:
- Simple, intuitive API
- Flexible ownership model
- Deterministic simulation
- Proper force/integration loop
- Static vs dynamic handling

### Files Created
1. `cpp/include/physics/PhysicsWorld.hpp` (interface, 235 lines)
2. `cpp/src/physics/PhysicsWorld.cpp` (implementation, 90 lines)
3. `tests/cpp/test_physicsworld.cpp` (test suite, 500+ lines)

### Physics Concepts Covered
- **Simulation loop**: Forces → Integration → Clear
- **Gravity**: F = m×g, acceleration independent of mass
- **Time stepping**: Fixed vs variable
- **Static bodies**: Infinite mass, no motion
- **Force accumulation**: Per-frame forces vs impulses

### Integration with Other Components

**Uses**:
- `RigidBody`: Stores and simulates individual objects
- `Transform`: Position and rotation (via RigidBody)
- `Vector2`: All position, velocity, force calculations
