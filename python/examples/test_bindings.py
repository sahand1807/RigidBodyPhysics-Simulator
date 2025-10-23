"""
Test script for Python bindings

Demonstrates the basic functionality of the physics engine
through the Python API.
"""

import sys
sys.path.insert(0, '..')

from physics_viz import Vector2, Transform, CircleCollider, RigidBody, PhysicsWorld

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
    print(f"v1 + v2 = {v1 + v2}")
    print(f"v1 - v2 = {v1 - v2}")
    print(f"v1 * 2 = {v1 * 2}")
    print(f"v1.length() = {v1.length()}")
    print(f"v1.dot(v2) = {v1.dot(v2)}")
    print(f"v1.cross(v2) = {v1.cross(v2)}")

    # Static methods
    print(f"Vector2.zero() = {Vector2.zero()}")
    print(f"Vector2.up() = {Vector2.up()}")
    print(f"Vector2.right() = {Vector2.right()}")
    print()

def test_transform():
    """Test Transform functionality"""
    print("=" * 50)
    print("Testing Transform")
    print("=" * 50)

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
    print()

def test_circle_collider():
    """Test CircleCollider functionality"""
    print("=" * 50)
    print("Testing CircleCollider")
    print("=" * 50)

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
    print()

def test_rigid_body():
    """Test RigidBody functionality"""
    print("=" * 50)
    print("Testing RigidBody")
    print("=" * 50)

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
    print()

def test_physics_world():
    """Test PhysicsWorld functionality"""
    print("=" * 50)
    print("Testing PhysicsWorld")
    print("=" * 50)

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

    print()

def test_collision():
    """Test collision between two balls"""
    print("=" * 50)
    print("Testing Collision")
    print("=" * 50)

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

    print()

def main():
    """Run all tests"""
    print("\n" + "=" * 50)
    print("Physics Engine - Python Bindings Test")
    print("=" * 50 + "\n")

    test_vector2()
    test_transform()
    test_circle_collider()
    test_rigid_body()
    test_physics_world()
    test_collision()

    print("=" * 50)
    print("All tests completed successfully!")
    print("=" * 50)

if __name__ == "__main__":
    main()
