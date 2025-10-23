/**
 * @file test_collision.cpp
 * @brief Test suite for collision detection and response
 *
 * Comprehensive tests for collision system including:
 * - Manifold structure
 * - Circle-circle collision detection
 * - Collision response (positional correction and velocity resolution)
 * - Integration with PhysicsWorld
 */

#include "physics/Manifold.hpp"
#include "physics/CollisionDetection.hpp"
#include "physics/CollisionResponse.hpp"
#include "physics/RigidBody.hpp"
#include "physics/CircleCollider.hpp"
#include "physics/PhysicsWorld.hpp"
#include "math/Vector2.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Physics;

// Helper function to check if two floats are approximately equal
bool approxEqual(float a, float b, float epsilon = 1e-4f) {
    return std::abs(a - b) < epsilon;
}

// Helper function to check if two vectors are approximately equal
bool approxEqual(const Vector2& a, const Vector2& b, float epsilon = 1e-4f) {
    return approxEqual(a.x, b.x, epsilon) && approxEqual(a.y, b.y, epsilon);
}

void testManifoldCreation() {
    std::cout << "Testing manifold creation..." << std::endl;

    // Default constructor
    Manifold m1;
    assert(m1.bodyA == nullptr);
    assert(m1.bodyB == nullptr);
    assert(approxEqual(m1.penetration, 0.0f));
    assert(!m1.isValid());
    std::cout << "  ✓ Default constructor creates empty manifold" << std::endl;

    // Create with data
    RigidBody body1, body2;
    Manifold m2(&body1, &body2, Vector2(1, 0), 0.5f, Vector2(5, 0));
    assert(m2.bodyA == &body1);
    assert(m2.bodyB == &body2);
    assert(approxEqual(m2.normal, Vector2(1, 0)));
    assert(approxEqual(m2.penetration, 0.5f));
    assert(approxEqual(m2.contactPoint, Vector2(5, 0)));
    assert(m2.isValid());
    std::cout << "  ✓ Constructed manifold is valid" << std::endl;

    // Clear
    m2.clear();
    assert(!m2.isValid());
    std::cout << "  ✓ Clear() makes manifold invalid" << std::endl;

    std::cout << "✓ Manifold creation passed!\n" << std::endl;
}

void testNoCollisionWhenSeparated() {
    std::cout << "Testing no collision when circles separated..." << std::endl;

    // Create two circles far apart
    RigidBody body1, body2;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);

    body1.setCollider(&circle1);
    body2.setCollider(&circle2);

    body1.setPosition(Vector2(0, 0));
    body2.setPosition(Vector2(10, 0));  // 10 units apart, radii sum = 2

    // Detect collision
    Manifold manifold;
    bool collision = CollisionDetection::circleVsCircle(&body1, &body2, manifold);

    assert(!collision);
    std::cout << "  ✓ No collision detected when separated" << std::endl;

    std::cout << "✓ No collision test passed!\n" << std::endl;
}

void testCollisionWhenOverlapping() {
    std::cout << "Testing collision when circles overlapping..." << std::endl;

    // Create two overlapping circles
    RigidBody body1, body2;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);

    body1.setCollider(&circle1);
    body2.setCollider(&circle2);

    body1.setPosition(Vector2(0, 0));
    body2.setPosition(Vector2(1.5f, 0));  // 1.5 units apart, radii sum = 2 → overlap!

    // Detect collision
    Manifold manifold;
    bool collision = CollisionDetection::circleVsCircle(&body1, &body2, manifold);

    assert(collision);
    assert(manifold.isValid());
    std::cout << "  ✓ Collision detected when overlapping" << std::endl;

    // Check normal (should point from body2 to body1, i.e., to the left)
    assert(approxEqual(manifold.normal, Vector2(-1, 0), 0.01f));
    std::cout << "  ✓ Normal points from B to A: " << manifold.normal << std::endl;

    // Check penetration (radii sum 2.0 - distance 1.5 = 0.5)
    assert(approxEqual(manifold.penetration, 0.5f, 0.01f));
    std::cout << "  ✓ Penetration: " << manifold.penetration << std::endl;

    std::cout << "✓ Collision detection passed!\n" << std::endl;
}

void testCollisionWithDifferentRadii() {
    std::cout << "Testing collision with different radii..." << std::endl;

    RigidBody body1, body2;
    CircleCollider circle1(2.0f);  // Large circle
    CircleCollider circle2(0.5f);  // Small circle

    body1.setCollider(&circle1);
    body2.setCollider(&circle2);

    body1.setPosition(Vector2(0, 0));
    body2.setPosition(Vector2(2.0f, 0));  // Distance 2.0, radii sum = 2.5

    Manifold manifold;
    bool collision = CollisionDetection::circleVsCircle(&body1, &body2, manifold);

    assert(collision);
    assert(approxEqual(manifold.penetration, 0.5f, 0.01f));  // 2.5 - 2.0 = 0.5
    std::cout << "  ✓ Different radii handled correctly" << std::endl;

    std::cout << "✓ Different radii test passed!\n" << std::endl;
}

void testCollisionExactlyTouching() {
    std::cout << "Testing collision when exactly touching..." << std::endl;

    RigidBody body1, body2;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);

    body1.setCollider(&circle1);
    body2.setCollider(&circle2);

    body1.setPosition(Vector2(0, 0));
    body2.setPosition(Vector2(2.0f, 0));  // Exactly touching (distance = radii sum)

    Manifold manifold;
    bool collision = CollisionDetection::circleVsCircle(&body1, &body2, manifold);

    // Exactly touching should not count as collision (no penetration)
    assert(!collision);
    std::cout << "  ✓ Exactly touching = no collision" << std::endl;

    std::cout << "✓ Touching test passed!\n" << std::endl;
}

void testPositionalCorrection() {
    std::cout << "Testing positional correction..." << std::endl;

    // Create two overlapping circles
    RigidBody body1, body2;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);

    body1.setCollider(&circle1);
    body2.setCollider(&circle2);

    body1.setMass(1.0f);
    body2.setMass(1.0f);

    body1.setPosition(Vector2(0, 0));
    body2.setPosition(Vector2(1.0f, 0));  // Heavily overlapping (penetration = 1.0)

    // Detect collision
    Manifold manifold;
    CollisionDetection::circleVsCircle(&body1, &body2, manifold);

    // Apply positional correction
    Vector2 pos1Before = body1.getPosition();
    Vector2 pos2Before = body2.getPosition();

    CollisionResponse::positionalCorrection(manifold);

    Vector2 pos1After = body1.getPosition();
    Vector2 pos2After = body2.getPosition();

    // Bodies should have moved apart
    float distanceAfter = (pos1After - pos2After).length();
    assert(distanceAfter > 1.0f);  // Should be further apart than before
    std::cout << "  ✓ Bodies moved apart: distance " << distanceAfter << std::endl;

    // With equal masses, they should move equal amounts
    float move1 = (pos1After - pos1Before).length();
    float move2 = (pos2After - pos2Before).length();
    assert(approxEqual(move1, move2, 0.01f));
    std::cout << "  ✓ Equal masses move equal amounts" << std::endl;

    std::cout << "✓ Positional correction passed!\n" << std::endl;
}

void testVelocityResolution() {
    std::cout << "Testing velocity resolution..." << std::endl;

    // Create two circles moving toward each other
    RigidBody body1, body2;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);

    body1.setCollider(&circle1);
    body2.setCollider(&circle2);

    body1.setMass(1.0f);
    body2.setMass(1.0f);
    body1.setRestitution(1.0f);  // Perfect bounce
    body2.setRestitution(1.0f);

    body1.setPosition(Vector2(0, 0));
    body2.setPosition(Vector2(1.5f, 0));

    // Moving toward each other
    body1.setVelocity(Vector2(5, 0));   // Moving right
    body2.setVelocity(Vector2(-5, 0));  // Moving left

    // Detect collision
    Manifold manifold;
    CollisionDetection::circleVsCircle(&body1, &body2, manifold);

    // Apply velocity resolution
    CollisionResponse::resolveVelocity(manifold);

    // With perfect restitution and equal masses, velocities should reverse
    assert(approxEqual(body1.getVelocity().x, -5.0f, 0.1f));
    assert(approxEqual(body2.getVelocity().x, 5.0f, 0.1f));
    std::cout << "  ✓ Velocities reversed with perfect restitution" << std::endl;

    std::cout << "✓ Velocity resolution passed!\n" << std::endl;
}

void testStaticBodyCollision() {
    std::cout << "Testing collision with static body..." << std::endl;

    // Dynamic body falling onto static ground
    RigidBody dynamicBody, staticBody;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);

    dynamicBody.setCollider(&circle1);
    staticBody.setCollider(&circle2);

    dynamicBody.setMass(1.0f);
    staticBody.setMass(0.0f);  // Static (infinite mass)

    dynamicBody.setPosition(Vector2(0, 1.5f));  // Above, overlapping
    staticBody.setPosition(Vector2(0, 0));

    dynamicBody.setVelocity(Vector2(0, -5));  // Falling
    staticBody.setVelocity(Vector2::zero());

    // Detect and resolve
    Manifold manifold;
    CollisionDetection::circleVsCircle(&dynamicBody, &staticBody, manifold);
    CollisionResponse::resolveCollision(manifold);

    // Dynamic body should move, static should not
    assert(dynamicBody.getPosition().y > 1.5f);  // Pushed up
    assert(approxEqual(staticBody.getPosition(), Vector2(0, 0)));  // Didn't move
    std::cout << "  ✓ Dynamic moved, static didn't" << std::endl;

    // Dynamic velocity should reverse (bounce), static stays zero
    assert(dynamicBody.getVelocity().y > 0);  // Now moving up
    assert(approxEqual(staticBody.getVelocity(), Vector2::zero()));
    std::cout << "  ✓ Dynamic bounced, static velocity unchanged" << std::endl;

    std::cout << "✓ Static body collision passed!\n" << std::endl;
}

void testPhysicsWorldCollision() {
    std::cout << "Testing physics world with collisions..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2(0, -10));

    // Create two bodies
    RigidBody ball1, ball2;
    CircleCollider circle1(1.0f);
    CircleCollider circle2(1.0f);

    ball1.setCollider(&circle1);
    ball2.setCollider(&circle2);

    ball1.setMass(1.0f);
    ball2.setMass(1.0f);
    ball1.setRestitution(0.8f);
    ball2.setRestitution(0.8f);

    // Position them overlapping
    ball1.setPosition(Vector2(0, 5));
    ball2.setPosition(Vector2(1.5f, 5));  // Overlapping

    world.addBody(&ball1);
    world.addBody(&ball2);

    // Step simulation
    world.step(0.1f);

    // Bodies should have separated
    float distance = (ball1.getPosition() - ball2.getPosition()).length();
    assert(distance > 1.5f);  // Should be further apart
    std::cout << "  ✓ PhysicsWorld separated overlapping bodies" << std::endl;

    std::cout << "✓ PhysicsWorld collision passed!\n" << std::endl;
}

void testBallBounce() {
    std::cout << "Testing ball bouncing off ground..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2(0, -9.81f));

    // Create ball and ground
    RigidBody ball, ground;
    CircleCollider ballCircle(1.0f);
    CircleCollider groundCircle(10.0f);  // Large flat-ish circle

    ball.setCollider(&ballCircle);
    ground.setCollider(&groundCircle);

    ball.setMass(1.0f);
    ground.setMass(0.0f);  // Static ground

    ball.setRestitution(0.9f);  // Bouncy
    ground.setRestitution(0.9f);

    ball.setPosition(Vector2(0, 15));  // Start above ground
    ball.setVelocity(Vector2(0, 0));

    ground.setPosition(Vector2(0, 0));

    world.addBody(&ball);
    world.addBody(&ground);

    // Simulate until ball hits ground
    for (int i = 0; i < 100; i++) {
        world.step(0.016f);  // 60 FPS

        // Check if ball bounced (velocity changed to upward after going down)
        if (ball.getVelocity().y > 5.0f) {
            std::cout << "  ✓ Ball bounced! Velocity: " << ball.getVelocity().y << " m/s" << std::endl;
            std::cout << "✓ Ball bounce passed!\n" << std::endl;
            return;
        }
    }

    // If we get here, ball didn't bounce
    std::cout << "  ✗ Ball didn't bounce!" << std::endl;
    assert(false);
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   Collision System Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;

    try {
        testManifoldCreation();
        testNoCollisionWhenSeparated();
        testCollisionWhenOverlapping();
        testCollisionWithDifferentRadii();
        testCollisionExactlyTouching();
        testPositionalCorrection();
        testVelocityResolution();
        testStaticBodyCollision();
        testPhysicsWorldCollision();
        testBallBounce();

        std::cout << "========================================" << std::endl;
        std::cout << "   ✓ ALL TESTS PASSED!" << std::endl;
        std::cout << "========================================" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "✗ TEST FAILED: " << e.what() << std::endl;
        return 1;
    }
}
