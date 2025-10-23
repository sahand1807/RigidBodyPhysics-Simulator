/**
 * @file test_physicsworld.cpp
 * @brief Test suite for PhysicsWorld class
 *
 * Comprehensive tests for physics simulation world including:
 * - Body management (add, remove, clear)
 * - Gravity settings
 * - Simulation stepping
 * - Integration loop verification
 * - Static vs dynamic body handling
 */

#include "physics/PhysicsWorld.hpp"
#include "physics/RigidBody.hpp"
#include "math/Vector2.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Physics;

// Mathematical constants
#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

// Helper function to check if two floats are approximately equal
bool approxEqual(float a, float b, float epsilon = 1e-4f) {
    return std::abs(a - b) < epsilon;
}

// Helper function to check if two vectors are approximately equal
bool approxEqual(const Vector2& a, const Vector2& b, float epsilon = 1e-4f) {
    return approxEqual(a.x, b.x, epsilon) && approxEqual(a.y, b.y, epsilon);
}

void testWorldConstruction() {
    std::cout << "Testing world construction..." << std::endl;

    PhysicsWorld world;

    // Default gravity should be Earth gravity (downward)
    assert(approxEqual(world.getGravity(), Vector2(0.0f, -9.81f)));
    std::cout << "  ✓ Default gravity: " << world.getGravity() << " m/s²" << std::endl;

    // World should start empty
    assert(world.getBodyCount() == 0);
    assert(world.getBodies().empty());
    std::cout << "  ✓ World starts empty: " << world.getBodyCount() << " bodies" << std::endl;

    std::cout << "✓ World construction passed!\n" << std::endl;
}

void testBodyManagement() {
    std::cout << "Testing body management..." << std::endl;

    PhysicsWorld world;
    RigidBody body1, body2, body3;

    // Add first body
    world.addBody(&body1);
    assert(world.getBodyCount() == 1);
    std::cout << "  ✓ Added body 1: count = " << world.getBodyCount() << std::endl;

    // Add second body
    world.addBody(&body2);
    assert(world.getBodyCount() == 2);
    std::cout << "  ✓ Added body 2: count = " << world.getBodyCount() << std::endl;

    // Add third body
    world.addBody(&body3);
    assert(world.getBodyCount() == 3);
    std::cout << "  ✓ Added body 3: count = " << world.getBodyCount() << std::endl;

    // Verify getBodies() returns correct list
    const std::vector<RigidBody*>& bodies = world.getBodies();
    assert(bodies.size() == 3);
    assert(bodies[0] == &body1);
    assert(bodies[1] == &body2);
    assert(bodies[2] == &body3);
    std::cout << "  ✓ getBodies() returns correct list" << std::endl;

    // Remove middle body
    bool removed = world.removeBody(&body2);
    assert(removed);
    assert(world.getBodyCount() == 2);
    std::cout << "  ✓ Removed body 2: count = " << world.getBodyCount() << std::endl;

    // Verify remaining bodies
    const std::vector<RigidBody*>& bodies2 = world.getBodies();
    assert(bodies2[0] == &body1);
    assert(bodies2[1] == &body3);
    std::cout << "  ✓ Remaining bodies are correct" << std::endl;

    // Try to remove body that's not in world
    RigidBody outsider;
    removed = world.removeBody(&outsider);
    assert(!removed);
    assert(world.getBodyCount() == 2);
    std::cout << "  ✓ Removing non-existent body returns false" << std::endl;

    // Clear all bodies
    world.clear();
    assert(world.getBodyCount() == 0);
    assert(world.getBodies().empty());
    std::cout << "  ✓ Clear removes all bodies: count = " << world.getBodyCount() << std::endl;

    std::cout << "✓ Body management passed!\n" << std::endl;
}

void testNullBodyHandling() {
    std::cout << "Testing null body handling..." << std::endl;

    PhysicsWorld world;

    // Try to add null pointer
    world.addBody(nullptr);
    assert(world.getBodyCount() == 0);
    std::cout << "  ✓ Adding nullptr is ignored" << std::endl;

    // Try to remove null pointer
    bool removed = world.removeBody(nullptr);
    assert(!removed);
    std::cout << "  ✓ Removing nullptr returns false" << std::endl;

    std::cout << "✓ Null body handling passed!\n" << std::endl;
}

void testDuplicateBodyPrevention() {
    std::cout << "Testing duplicate body prevention..." << std::endl;

    PhysicsWorld world;
    RigidBody body;

    // Add body first time
    world.addBody(&body);
    assert(world.getBodyCount() == 1);
    std::cout << "  ✓ Added body: count = " << world.getBodyCount() << std::endl;

    // Try to add same body again
    world.addBody(&body);
    assert(world.getBodyCount() == 1);  // Should still be 1
    std::cout << "  ✓ Duplicate add prevented: count = " << world.getBodyCount() << std::endl;

    std::cout << "✓ Duplicate body prevention passed!\n" << std::endl;
}

void testGravitySettings() {
    std::cout << "Testing gravity settings..." << std::endl;

    PhysicsWorld world;

    // Default gravity
    assert(approxEqual(world.getGravity(), Vector2(0.0f, -9.81f)));
    std::cout << "  ✓ Default: " << world.getGravity() << std::endl;

    // Set zero gravity (space)
    world.setGravity(Vector2::zero());
    assert(approxEqual(world.getGravity(), Vector2::zero()));
    std::cout << "  ✓ Zero gravity: " << world.getGravity() << std::endl;

    // Set Moon gravity
    world.setGravity(Vector2(0.0f, -1.62f));
    assert(approxEqual(world.getGravity(), Vector2(0.0f, -1.62f)));
    std::cout << "  ✓ Moon gravity: " << world.getGravity() << std::endl;

    // Set custom gravity (sideways)
    world.setGravity(Vector2(-5.0f, 0.0f));
    assert(approxEqual(world.getGravity(), Vector2(-5.0f, 0.0f)));
    std::cout << "  ✓ Custom gravity: " << world.getGravity() << std::endl;

    std::cout << "✓ Gravity settings passed!\n" << std::endl;
}

void testSimulationStep() {
    std::cout << "Testing simulation step..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2(0.0f, -10.0f));  // Simple gravity for testing

    // Create a falling body
    RigidBody body;
    body.setMass(1.0f);
    body.setPosition(Vector2(0.0f, 100.0f));
    body.setVelocity(Vector2::zero());

    world.addBody(&body);

    // Step the simulation
    float dt = 0.1f;  // 0.1 seconds
    Vector2 initialPos = body.getPosition();
    world.step(dt);

    // Body should have moved downward
    Vector2 newPos = body.getPosition();
    assert(newPos.y < initialPos.y);
    std::cout << "  ✓ Body fell from " << initialPos.y << " to " << newPos.y << std::endl;

    // Velocity should have increased (downward)
    Vector2 velocity = body.getVelocity();
    assert(velocity.y < 0.0f);  // Negative = downward
    std::cout << "  ✓ Velocity after step: " << velocity << " m/s" << std::endl;

    std::cout << "✓ Simulation step passed!\n" << std::endl;
}

void testGravityApplication() {
    std::cout << "Testing gravity application..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2(0.0f, -10.0f));  // 10 m/s² downward

    // Create body with known mass
    RigidBody body;
    body.setMass(2.0f);  // 2 kg
    body.setPosition(Vector2(0.0f, 10.0f));
    body.setVelocity(Vector2::zero());

    world.addBody(&body);

    // Step once
    float dt = 0.1f;
    world.step(dt);

    // Expected: gravity force = m × g = 2 × (-10) = -20 N
    // Expected: acceleration = F/m = -20/2 = -10 m/s²
    // Expected: velocity change = a × dt = -10 × 0.1 = -1 m/s
    // Expected: velocity = 0 + (-1) = -1 m/s
    Vector2 velocity = body.getVelocity();
    assert(approxEqual(velocity.y, -1.0f, 0.01f));
    std::cout << "  ✓ Velocity after gravity: " << velocity.y << " m/s (expected -1.0)" << std::endl;

    std::cout << "✓ Gravity application passed!\n" << std::endl;
}

void testStaticBodyNotAffectedByGravity() {
    std::cout << "Testing static body not affected by gravity..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2(0.0f, -10.0f));

    // Create static body (infinite mass)
    RigidBody staticBody;
    staticBody.setMass(0.0f);  // 0 mass → infinite mass → static
    staticBody.setPosition(Vector2(0.0f, 10.0f));
    staticBody.setVelocity(Vector2::zero());

    world.addBody(&staticBody);

    // Step simulation
    float dt = 1.0f;  // 1 full second
    Vector2 initialPos = staticBody.getPosition();
    world.step(dt);

    // Static body should not move
    Vector2 newPos = staticBody.getPosition();
    assert(approxEqual(newPos, initialPos));
    std::cout << "  ✓ Static body didn't move: " << newPos << std::endl;

    // Velocity should still be zero
    assert(approxEqual(staticBody.getVelocity(), Vector2::zero()));
    std::cout << "  ✓ Static body velocity: " << staticBody.getVelocity() << std::endl;

    std::cout << "✓ Static body handling passed!\n" << std::endl;
}

void testMultipleBodySimulation() {
    std::cout << "Testing multiple body simulation..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2(0.0f, -10.0f));

    // Create three bodies at different heights
    RigidBody body1, body2, body3;

    body1.setMass(1.0f);
    body1.setPosition(Vector2(0.0f, 100.0f));

    body2.setMass(2.0f);
    body2.setPosition(Vector2(5.0f, 50.0f));

    body3.setMass(0.5f);
    body3.setPosition(Vector2(10.0f, 25.0f));

    world.addBody(&body1);
    world.addBody(&body2);
    world.addBody(&body3);

    // Step simulation
    float dt = 0.1f;
    world.step(dt);

    // All bodies should have fallen
    assert(body1.getPosition().y < 100.0f);
    assert(body2.getPosition().y < 50.0f);
    assert(body3.getPosition().y < 25.0f);
    std::cout << "  ✓ Body 1: " << body1.getPosition() << std::endl;
    std::cout << "  ✓ Body 2: " << body2.getPosition() << std::endl;
    std::cout << "  ✓ Body 3: " << body3.getPosition() << std::endl;

    // All bodies should have same velocity (gravity acceleration is independent of mass)
    // a = F/m = (m×g)/m = g = -10 m/s²
    // v = a × dt = -10 × 0.1 = -1 m/s
    assert(approxEqual(body1.getVelocity().y, -1.0f, 0.01f));
    assert(approxEqual(body2.getVelocity().y, -1.0f, 0.01f));
    assert(approxEqual(body3.getVelocity().y, -1.0f, 0.01f));
    std::cout << "  ✓ All bodies have same velocity (mass-independent acceleration)" << std::endl;

    std::cout << "✓ Multiple body simulation passed!\n" << std::endl;
}

void testForceClearing() {
    std::cout << "Testing force clearing..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2::zero());  // No gravity for this test

    RigidBody body;
    body.setMass(1.0f);
    body.setPosition(Vector2::zero());
    body.setVelocity(Vector2::zero());

    world.addBody(&body);

    // Manually apply a force
    body.applyForce(Vector2(10.0f, 0.0f));

    // Step once (should integrate and clear forces)
    world.step(0.1f);

    // Velocity should have changed (force was applied)
    assert(!approxEqual(body.getVelocity(), Vector2::zero()));
    std::cout << "  ✓ Force was applied: velocity = " << body.getVelocity() << std::endl;

    // Step again without applying new force
    Vector2 velocityAfterFirstStep = body.getVelocity();
    world.step(0.1f);

    // Velocity should stay the same (no forces, just momentum)
    assert(approxEqual(body.getVelocity(), velocityAfterFirstStep));
    std::cout << "  ✓ Force was cleared: velocity unchanged = " << body.getVelocity() << std::endl;

    std::cout << "✓ Force clearing passed!\n" << std::endl;
}

void testZeroGravitySimulation() {
    std::cout << "Testing zero gravity simulation..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2::zero());  // Space!

    RigidBody body;
    body.setMass(1.0f);
    body.setPosition(Vector2(0.0f, 10.0f));
    body.setVelocity(Vector2(5.0f, 0.0f));  // Moving horizontally

    world.addBody(&body);

    // Step simulation multiple times
    for (int i = 0; i < 10; i++) {
        world.step(0.1f);
    }

    // Body should maintain constant velocity (Newton's first law)
    assert(approxEqual(body.getVelocity(), Vector2(5.0f, 0.0f)));
    std::cout << "  ✓ Constant velocity maintained: " << body.getVelocity() << std::endl;

    // Position should have moved horizontally
    // x = x0 + v×t = 0 + 5×(0.1×10) = 5 m
    assert(approxEqual(body.getPosition().x, 5.0f, 0.01f));
    assert(approxEqual(body.getPosition().y, 10.0f));  // No vertical movement
    std::cout << "  ✓ Position after 1 second: " << body.getPosition() << std::endl;

    std::cout << "✓ Zero gravity simulation passed!\n" << std::endl;
}

void testCustomGravityDirection() {
    std::cout << "Testing custom gravity direction..." << std::endl;

    PhysicsWorld world;
    world.setGravity(Vector2(-10.0f, 0.0f));  // Gravity pulls to the left!

    RigidBody body;
    body.setMass(1.0f);
    body.setPosition(Vector2(10.0f, 5.0f));
    body.setVelocity(Vector2::zero());

    world.addBody(&body);

    // Step simulation
    world.step(0.1f);

    // Body should accelerate to the left (negative x)
    assert(body.getVelocity().x < 0.0f);
    assert(approxEqual(body.getVelocity().y, 0.0f));
    std::cout << "  ✓ Leftward velocity: " << body.getVelocity() << std::endl;

    // Position should move left
    assert(body.getPosition().x < 10.0f);
    assert(approxEqual(body.getPosition().y, 5.0f));
    std::cout << "  ✓ Position moved left: " << body.getPosition() << std::endl;

    std::cout << "✓ Custom gravity direction passed!\n" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   PhysicsWorld Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;

    try {
        testWorldConstruction();
        testBodyManagement();
        testNullBodyHandling();
        testDuplicateBodyPrevention();
        testGravitySettings();
        testSimulationStep();
        testGravityApplication();
        testStaticBodyNotAffectedByGravity();
        testMultipleBodySimulation();
        testForceClearing();
        testZeroGravitySimulation();
        testCustomGravityDirection();

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
