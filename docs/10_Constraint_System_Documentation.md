# Constraint System Documentation

## Overview

The constraint system allows you to connect rigid bodies together or fix them to world positions, creating complex mechanical systems like pendulums, chains, ropes, and Newton's cradles. The system uses **XPBD (Extended Position-Based Dynamics)** for stable and energy-conserving constraint solving.

## Files

### C++ Core
- `cpp/include/physics/Constraint.hpp` - Abstract base class for all constraints
- `cpp/include/physics/DistanceConstraint.hpp` - Distance constraint header
- `cpp/src/physics/DistanceConstraint.cpp` - Distance constraint implementation
- `cpp/include/physics/PhysicsWorld.hpp` - Constraint management added
- `cpp/src/physics/PhysicsWorld.cpp` - Constraint solver integration

### Python Bindings
- `cpp/src/bindings/bindings.cpp` - Python bindings for constraints
- `python/physics_viz/__init__.py` - Python API exports

## Constraint Base Class

### Constraint.hpp

```cpp
class Constraint {
public:
    virtual ~Constraint() = default;

    // Core interface
    virtual void solve(float dt) = 0;      // Solve constraint
    virtual bool isValid() const = 0;       // Check if constraint is valid
    virtual bool involves(const RigidBody* body) const = 0;  // Check if body is part of constraint

    // Enable/disable control
    void setEnabled(bool enabled);
    bool isEnabled() const;

protected:
    bool enabled = true;
};
```

**Key Concepts:**
- All constraints inherit from this base class
- `solve()` is called during physics step to enforce the constraint
- Constraints can be enabled/disabled without removal
- The `involves()` method is used for cleanup when bodies are destroyed

## Distance Constraint

The **DistanceConstraint** maintains a fixed distance between two anchor points. It can be used to create:
- **Rigid rods** (very low compliance)
- **Flexible ropes** (higher compliance)
- **Pendulums** (body-to-world constraint)
- **Chains** (multiple body-to-body constraints)

### Class Definition

```cpp
class DistanceConstraint : public Constraint {
public:
    // Body-to-world constructor (e.g., pendulum)
    DistanceConstraint(RigidBody* bodyA,
                      const Vector2& localAnchorA,
                      const Vector2& worldPoint,
                      float length,
                      float compliance = 0.0f);

    // Body-to-body constructor (e.g., rope segment)
    DistanceConstraint(RigidBody* bodyA,
                      const Vector2& localAnchorA,
                      RigidBody* bodyB,
                      const Vector2& localAnchorB,
                      float length,
                      float compliance = 0.0f);

    // Constraint configuration
    void setLength(float length);
    float getLength() const;

    void setCompliance(float compliance);
    float getCompliance() const;

    float getCurrentDistance() const;

    // Constraint interface
    void solve(float dt) override;
    bool isValid() const override;
    bool involves(const RigidBody* body) const override;

private:
    RigidBody* bodyA;              // First body (always required)
    RigidBody* bodyB;              // Second body (nullptr for world constraint)
    Vector2 localAnchorA;          // Anchor point in bodyA's local space
    Vector2 localAnchorB;          // Anchor point in bodyB's local space (or world position)
    float length;                  // Target distance
    float compliance;              // Stiffness parameter (0 = rigid)
    float lagrangian;              // Accumulated Lagrange multiplier
};
```

### Parameters

#### Length
- The target distance to maintain between anchor points
- Must be positive
- Measured in meters

#### Compliance
- Controls the "stiffness" of the constraint
- **0.0**: Perfectly rigid (may be unstable)
- **0.0001**: Nearly rigid, stable (recommended for rods)
- **0.001 - 0.01**: Flexible (good for ropes)
- **Higher values**: More elastic/stretchy

The compliance parameter α relates to spring stiffness k:
```
α = 1 / (k × dt²)
```

### XPBD Algorithm

The constraint solver uses **XPBD** (Extended Position-Based Dynamics), which provides:
1. **Frame-rate independence**: Behavior doesn't change with different time steps
2. **Energy conservation**: Critical for realistic physics
3. **Stability**: Works with large time steps

#### Solver Steps

1. **Calculate Error**
   ```cpp
   float currentDist = (posA - posB).length();
   float error = currentDist - length;  // Positive = stretched, negative = compressed
   ```

2. **Compute Gradient**
   ```cpp
   Vector2 gradient = delta / currentDist;  // Normalized direction
   ```

3. **Calculate Lagrange Multiplier**
   ```cpp
   float alpha = compliance / (dt * dt);
   float deltaLagrangian = -error / (totalInvMass + alpha);
   ```

4. **Apply Position Correction**
   ```cpp
   Vector2 correction = gradient * deltaLagrangian;
   Vector2 newPosA = oldPosA + correction * invMassA;
   bodyA->setPosition(newPosA);
   ```

5. **Update Velocity (CRITICAL for energy conservation!)**
   ```cpp
   Vector2 deltaVelA = (newPosA - oldPosA) / dt;
   bodyA->setVelocity(bodyA->getVelocity() + deltaVelA);
   ```

**Why Velocity Update is Critical:**
Without the velocity update in step 5, the constraint would only move positions but leave velocities unchanged. This causes massive energy loss as the integration step and constraint solver fight each other every frame.

### Physics World Integration

The PhysicsWorld class manages constraints alongside rigid bodies:

```cpp
class PhysicsWorld {
public:
    // Constraint management
    void addConstraint(Constraint* constraint);
    bool removeConstraint(Constraint* constraint);
    void clearConstraints();

    // Physics step
    void step(float dt);  // Calls solveConstraints() internally

private:
    std::vector<Constraint*> constraints;
    void solveConstraints(float dt, int iterations);
};
```

#### Constraint Solving in Physics Step

```cpp
void PhysicsWorld::step(float dt) {
    // 1. Apply global forces (gravity)
    applyGravity(dt);

    // 2. Integrate all bodies (velocity → position)
    for (RigidBody* body : bodies) {
        body->integrate(dt);
    }

    // 3. Solve constraints (pre-collision)
    solveConstraints(dt, 3);  // 3 iterations

    // 4. Detect and resolve collisions
    detectAndResolveCollisions();

    // 5. Solve constraints (post-collision)
    solveConstraints(dt, 2);  // 2 iterations to fix collision drift

    // 6. Clear force accumulators
    for (RigidBody* body : bodies) {
        body->clearForces();
    }
}
```

**Why Two Constraint Passes?**
- **Pre-collision**: Enforces constraints before collision detection
- **Post-collision**: Fixes any constraint drift caused by collision resolution

**Iteration Count Trade-off:**
- **More iterations**: More accurate, but can damp motion (energy loss)
- **Fewer iterations**: Better energy preservation, but less accurate
- **Recommended**: 3-5 total iterations for most applications

## Python API

### Creating Constraints

#### Body-to-World (Pendulum)

```python
from physics_viz import RigidBody, CircleCollider, Vector2, DistanceConstraint, PhysicsWorld

# Create world and body
world = PhysicsWorld()
ball = RigidBody()
ball.set_collider(CircleCollider(0.5))
ball.set_mass(1.0)
ball.set_position(Vector2(0, 5))
world.add_body(ball)

# Create pendulum rod (hang from world point)
anchor_point = Vector2(0, 10)
rod_length = 5.0
compliance = 0.0001  # Nearly rigid

rod = DistanceConstraint(
    ball,                    # Body to constrain
    Vector2(0, 0),          # Attach at ball center
    anchor_point,           # Fixed world point
    rod_length,             # Length in meters
    compliance              # Stiffness
)

world.add_constraint(rod)
```

#### Body-to-Body (Rope/Chain)

```python
# Create two balls
ball1 = RigidBody()
ball1.set_position(Vector2(0, 10))
ball1.set_collider(CircleCollider(0.5))
ball1.set_mass(1.0)
world.add_body(ball1)

ball2 = RigidBody()
ball2.set_position(Vector2(0, 5))
ball2.set_collider(CircleCollider(0.5))
ball2.set_mass(1.0)
world.add_body(ball2)

# Connect them with a rope
rope = DistanceConstraint(
    ball1,              # First body
    Vector2(0, 0),     # Attach at center
    ball2,             # Second body
    Vector2(0, 0),     # Attach at center
    5.0,               # Length
    0.001              # More flexible (rope-like)
)

world.add_constraint(rope)
```

### Managing Constraints

```python
# Enable/disable
rope.set_enabled(False)  # Temporarily disable
rope.set_enabled(True)   # Re-enable

# Check status
if rope.is_enabled():
    print("Constraint is active")

# Modify properties
rope.set_length(6.0)         # Change length
rope.set_compliance(0.01)    # Make more flexible

# Query state
current_dist = rope.get_current_distance()
print(f"Current distance: {current_dist}m")

# Remove from world
world.remove_constraint(rope)
```

## Example: Newton's Cradle

Here's how to create a Newton's cradle with 5 balls:

```python
from physics_viz import PhysicsWorld, RigidBody, CircleCollider, Vector2, DistanceConstraint

# Setup
world = PhysicsWorld()
world.set_gravity(Vector2(0, -9.81))

num_balls = 5
ball_radius = 0.5
rod_length = 5.0
anchor_y = 10.0
ball_spacing = ball_radius * 2.0  # Balls touching

balls = []
constraints = []

# Create balls and rods
for i in range(num_balls):
    # Position calculation
    x = (i - num_balls / 2) * ball_spacing
    y = anchor_y - rod_length

    # Create ball
    ball = RigidBody()
    ball.set_collider(CircleCollider(ball_radius))
    ball.set_mass(1.0)
    ball.set_position(Vector2(x, y))
    ball.set_restitution(0.99)  # Very elastic
    ball.set_friction(0.0)
    world.add_body(ball)
    balls.append(ball)

    # Create rod constraint
    anchor = Vector2(x, anchor_y)
    rod = DistanceConstraint(
        ball,
        Vector2(0, 0),
        anchor,
        rod_length,
        0.0001  # Nearly rigid
    )
    world.add_constraint(rod)
    constraints.append(rod)

# Pull back first ball
import math
angle = math.radians(30)
x_displaced = (0 - num_balls / 2) * ball_spacing - rod_length * math.sin(angle)
y_displaced = anchor_y - rod_length * math.cos(angle)
balls[0].set_position(Vector2(x_displaced, y_displaced))

# Run simulation
dt = 1.0 / 60.0
for frame in range(600):
    world.step(dt)
    # Render or process results
```

## Best Practices

### 1. Compliance Values
- **Rigid rods**: `0.0001`
- **Stiff ropes**: `0.001`
- **Flexible ropes**: `0.01`
- **Very elastic**: `0.1+`

### 2. Iteration Count
- Start with 3-5 total iterations
- Increase if constraints are unstable
- Decrease if motion is over-damped

### 3. Energy Conservation
- Always use small compliance (never exactly 0)
- Keep iteration count low
- Ensure velocity updates are enabled (they are by default in our implementation)

### 4. Collision + Constraints
- Set high restitution (0.9-0.99) for elastic systems
- Set friction to 0 for frictionless constraints
- Use exactly touching geometries (no gaps) for collision chains

### 5. Material Properties
```python
# For constrained systems demonstrating momentum transfer
ball.set_restitution(0.99)  # Very elastic
ball.set_friction(0.0)       # Frictionless
```

## Common Issues

### Problem: Constraints are stretchy/not rigid
**Solution**: Decrease compliance (try 0.0001 or lower)

### Problem: System loses energy over time
**Solution**:
- Reduce constraint solver iterations
- Ensure compliance is small but non-zero
- Check that velocity updates are applied (built into our solver)

### Problem: Balls don't collide in Newton's cradle
**Solution**: Ensure ball spacing equals `radius * 2.0` exactly (balls touching)

### Problem: Unstable/jittery motion
**Solution**:
- Increase compliance slightly (0.0001 → 0.001)
- Increase solver iterations (3 → 5)
- Decrease time step (dt)

## Performance Considerations

- Constraints are solved iteratively: O(iterations × num_constraints)
- Each constraint processes 1-2 bodies per solve
- For large systems (100+ constraints), consider:
  - Reducing iteration count
  - Using spatial partitioning
  - Solving only active/nearby constraints

## References

- **XPBD**: M. Macklin et al., "XPBD: Position-Based Simulation of Compliant Constrained Dynamics" (2016)
- **PBD**: M. M

üller et al., "Position Based Dynamics" (2007)
- **Lagrange Multipliers**: Classic constrained optimization theory

## See Also

- `03_RigidBody_Documentation.md` - Rigid body physics
- `05_PhysicsWorld_Documentation.md` - Physics world management
- `11_Newtons_Cradle_Example.md` - Complete Newton's cradle implementation
