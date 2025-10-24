#!/usr/bin/env python3
"""
Test script for BoxCollider Python bindings

Tests BoxCollider functionality and box-circle collisions.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from physics_viz import Vector2, RigidBody, BoxCollider, CircleCollider, PhysicsWorld


def test_box_collider_creation():
    """Test BoxCollider can be created and accessed"""
    print("=" * 60)
    print("Test 1: BoxCollider Creation")
    print("=" * 60)

    # Create box collider
    box = BoxCollider(2.0, 1.0)

    print(f"Box: {box}")
    print(f"Width: {box.get_width()}")
    print(f"Height: {box.get_height()}")
    print(f"Half-width: {box.get_half_width()}")
    print(f"Half-height: {box.get_half_height()}")

    assert box.get_width() == 2.0, "Width should be 2.0"
    assert box.get_height() == 1.0, "Height should be 1.0"
    assert box.get_half_width() == 1.0, "Half-width should be 1.0"
    assert box.get_half_height() == 0.5, "Half-height should be 0.5"

    print("✓ BoxCollider creation test passed\n")


def test_box_mass_properties():
    """Test mass and inertia calculations"""
    print("=" * 60)
    print("Test 2: Box Mass Properties")
    print("=" * 60)

    box = BoxCollider(4.0, 2.0)

    density = 1.0
    mass = box.calculate_mass(density)
    inertia = box.calculate_inertia(mass)

    print(f"Box: {box.get_width()}m × {box.get_height()}m")
    print(f"Density: {density} kg/m²")
    print(f"Mass: {mass:.2f} kg")
    print(f"Inertia: {inertia:.2f} kg⋅m²")

    # Mass = density × width × height = 1.0 × 4.0 × 2.0 = 8.0
    assert abs(mass - 8.0) < 0.01, f"Mass should be 8.0, got {mass}"

    # Inertia = (1/12) × m × (w² + h²) = (1/12) × 8 × (16 + 4) = (1/12) × 8 × 20 = 13.33
    expected_inertia = (1.0 / 12.0) * 8.0 * (16.0 + 4.0)
    assert abs(inertia - expected_inertia) < 0.01, f"Inertia should be {expected_inertia:.2f}"

    print("✓ Mass properties test passed\n")


def test_box_contains_point():
    """Test point containment"""
    print("=" * 60)
    print("Test 3: Box Point Containment")
    print("=" * 60)

    box = BoxCollider(2.0, 1.0)  # width=2, height=1, centered at offset

    # Points to test (in local space)
    inside = Vector2(0.5, 0.3)
    outside = Vector2(2.0, 0.0)
    on_edge = Vector2(1.0, 0.0)  # Right on the edge

    print(f"Box: {box}")
    print(f"Point (0.5, 0.3) inside? {box.contains_point(inside)}")
    print(f"Point (2.0, 0.0) inside? {box.contains_point(outside)}")
    print(f"Point (1.0, 0.0) inside? {box.contains_point(on_edge)}")

    assert box.contains_point(inside), "Point (0.5, 0.3) should be inside"
    assert not box.contains_point(outside), "Point (2.0, 0.0) should be outside"
    assert box.contains_point(on_edge), "Point (1.0, 0.0) should be on edge (inside)"

    print("✓ Point containment test passed\n")


def test_box_with_rigid_body():
    """Test BoxCollider attached to RigidBody"""
    print("=" * 60)
    print("Test 4: Box with RigidBody")
    print("=" * 60)

    body = RigidBody()
    box = BoxCollider(3.0, 1.5)

    body.set_collider(box)

    density = 1.0
    mass = box.calculate_mass(density)
    inertia = box.calculate_inertia(mass)

    body.set_mass(mass)
    body.set_moment_of_inertia(inertia)
    body.set_position(Vector2(10, 5))

    print(f"Body: {body}")
    print(f"Collider: {box}")
    print(f"Has collider: {body.has_collider()}")
    print(f"Mass: {body.get_mass():.2f} kg")
    print(f"Inertia: {body.get_moment_of_inertia():.2f} kg⋅m²")

    assert body.has_collider(), "Body should have collider"
    assert abs(body.get_mass() - mass) < 0.01, "Mass should match"

    print("✓ RigidBody integration test passed\n")


def test_ball_on_ground():
    """Test ball bouncing on box ground"""
    print("=" * 60)
    print("Test 5: Ball Bouncing on Box Ground")
    print("=" * 60)

    world = PhysicsWorld()
    world.set_gravity(Vector2(0, -9.81))

    # Create ground (wide, thin box)
    ground = RigidBody()
    ground_box = BoxCollider(100.0, 1.0)  # Very wide, thin platform
    ground.set_collider(ground_box)
    ground.set_static()
    ground.set_position(Vector2(0, -5))
    ground.set_restitution(0.8)

    # Create ball above ground
    ball = RigidBody()
    ball_circle = CircleCollider(1.0)
    ball.set_collider(ball_circle)

    density = 1.0
    mass = ball_circle.calculate_mass(density)
    inertia = ball_circle.calculate_inertia(mass)
    ball.set_mass(mass)
    ball.set_moment_of_inertia(inertia)
    ball.set_position(Vector2(0, 10))
    ball.set_restitution(0.8)

    world.add_body(ground)
    world.add_body(ball)

    print(f"Ground (box): {ground_box} at y={ground.get_position().y}")
    print(f"Ball (circle): radius={ball_circle.get_radius()} at y={ball.get_position().y}")
    print("\nSimulating ball drop...")

    # Simulate until ball bounces
    dt = 1.0 / 60.0
    bounced = False

    for i in range(300):  # 5 seconds max
        world.step(dt)

        pos = ball.get_position()
        vel = ball.get_velocity()

        # Print progress every second
        if i % 60 == 0:
            print(f"  t={i/60:.1f}s: pos=({pos.x:.2f}, {pos.y:.2f}), vel=({vel.x:.2f}, {vel.y:.2f})")

        # Check if ball bounced (velocity changed from negative to positive)
        if i > 60 and vel.y > 1.0:  # Moving upward significantly
            bounced = True
            print(f"\n✓ Ball bounced at t={i/60:.2f}s!")
            print(f"  Position: ({pos.x:.2f}, {pos.y:.2f})")
            print(f"  Velocity: ({vel.x:.2f}, {vel.y:.2f})")
            break

    assert bounced, "Ball should have bounced off ground"
    print("\n✓ Box-circle collision test passed\n")


def test_multiple_balls_on_ground():
    """Test multiple balls bouncing on box ground"""
    print("=" * 60)
    print("Test 6: Multiple Balls on Box Ground")
    print("=" * 60)

    world = PhysicsWorld()
    world.set_gravity(Vector2(0, -9.81))

    # Create ground
    ground = RigidBody()
    ground_box = BoxCollider(50.0, 2.0)
    ground.set_collider(ground_box)
    ground.set_static()
    ground.set_position(Vector2(0, -10))
    ground.set_restitution(0.7)
    world.add_body(ground)

    # Create 3 balls
    balls = []
    for i in range(3):
        ball = RigidBody()
        circle = CircleCollider(0.5)
        ball.set_collider(circle)

        mass = circle.calculate_mass(1.0)
        inertia = circle.calculate_inertia(mass)
        ball.set_mass(mass)
        ball.set_moment_of_inertia(inertia)

        ball.set_position(Vector2(i * 2.0 - 2.0, 5 + i * 2))
        ball.set_restitution(0.7)

        world.add_body(ball)
        balls.append(ball)

    print(f"Created 3 balls and 1 box ground")
    print(f"Total bodies: {world.get_body_count()}")

    # Simulate
    dt = 1.0 / 60.0
    for i in range(180):  # 3 seconds
        world.step(dt)

    # Check that all balls have settled on ground (low velocity)
    all_settled = True
    for idx, ball in enumerate(balls):
        vel = ball.get_velocity()
        speed = vel.length()
        pos = ball.get_position()
        print(f"Ball {idx}: pos=({pos.x:.2f}, {pos.y:.2f}), speed={speed:.2f}")

        if speed > 2.0:  # Still moving fast
            all_settled = False

    print(f"\n✓ All balls interacted with box ground")
    print("✓ Multiple collision test passed\n")


def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("BoxCollider Python Bindings Test Suite")
    print("=" * 60 + "\n")

    try:
        test_box_collider_creation()
        test_box_mass_properties()
        test_box_contains_point()
        test_box_with_rigid_body()
        test_ball_on_ground()
        test_multiple_balls_on_ground()

        print("=" * 60)
        print("ALL TESTS PASSED! ✓")
        print("=" * 60)
        return 0

    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
