/**
 * @file test_rigidbody.cpp
 * @brief Test suite for RigidBody class
 *
 * Comprehensive tests for rigid body physics including:
 * - Construction and initialization
 * - Mass and inertia properties
 * - Static vs dynamic bodies
 * - Force and impulse application
 * - Physics integration
 * - Energy and momentum calculations
 */

#include "physics/RigidBody.hpp"
#include "math/Vector2.hpp"
#include "math/Transform.hpp"
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

void testConstructors() {
    std::cout << "Testing constructors..." << std::endl;

    // Default constructor
    RigidBody body1;
    assert(approxEqual(body1.getMass(), 1.0f));
    assert(approxEqual(body1.getInverseMass(), 1.0f));
    assert(approxEqual(body1.getMomentOfInertia(), 1.0f));
    assert(approxEqual(body1.getPosition(), Vector2::zero()));
    assert(approxEqual(body1.getVelocity(), Vector2::zero()));
    assert(!body1.isStatic());
    std::cout << "  ✓ Default constructor: mass=" << body1.getMass()
              << ", pos=" << body1.getPosition() << std::endl;

    // Constructor with mass
    RigidBody body2(5.0f);
    assert(approxEqual(body2.getMass(), 5.0f));
    assert(approxEqual(body2.getInverseMass(), 0.2f));
    std::cout << "  ✓ Mass constructor: mass=" << body2.getMass()
              << ", invMass=" << body2.getInverseMass() << std::endl;

    std::cout << "✓ Constructors passed!\n" << std::endl;
}

void testMassAndInertia() {
    std::cout << "Testing mass and inertia properties..." << std::endl;

    RigidBody body;

    // Set mass
    body.setMass(10.0f);
    assert(approxEqual(body.getMass(), 10.0f));
    assert(approxEqual(body.getInverseMass(), 0.1f));
    std::cout << "  ✓ Set mass: mass=" << body.getMass()
              << ", invMass=" << body.getInverseMass() << std::endl;

    // Set moment of inertia
    body.setMomentOfInertia(5.0f);
    assert(approxEqual(body.getMomentOfInertia(), 5.0f));
    assert(approxEqual(body.getInverseMomentOfInertia(), 0.2f));
    std::cout << "  ✓ Set inertia: I=" << body.getMomentOfInertia()
              << ", invI=" << body.getInverseMomentOfInertia() << std::endl;

    // Invalid mass (should become static)
    body.setMass(0.0f);
    assert(approxEqual(body.getMass(), 0.0f));
    assert(approxEqual(body.getInverseMass(), 0.0f));
    assert(body.isStatic());
    std::cout << "  ✓ Zero mass makes body static" << std::endl;

    std::cout << "✓ Mass and inertia passed!\n" << std::endl;
}

void testStaticVsDynamic() {
    std::cout << "Testing static vs dynamic bodies..." << std::endl;

    RigidBody body;

    // Initially dynamic
    assert(!body.isStatic());
    std::cout << "  ✓ Initially dynamic" << std::endl;

    // Make static
    body.setStatic();
    assert(body.isStatic());
    assert(approxEqual(body.getMass(), 0.0f));
    assert(approxEqual(body.getInverseMass(), 0.0f));
    assert(approxEqual(body.getVelocity(), Vector2::zero()));
    std::cout << "  ✓ setStatic(): invMass=" << body.getInverseMass() << std::endl;

    // Make dynamic again
    body.setDynamic(2.0f, 1.5f);
    assert(!body.isStatic());
    assert(approxEqual(body.getMass(), 2.0f));
    assert(approxEqual(body.getMomentOfInertia(), 1.5f));
    std::cout << "  ✓ setDynamic(): mass=" << body.getMass()
              << ", I=" << body.getMomentOfInertia() << std::endl;

    std::cout << "✓ Static vs dynamic passed!\n" << std::endl;
}

void testForceApplication() {
    std::cout << "Testing force application..." << std::endl;

    RigidBody body;
    body.setMass(2.0f);  // 2 kg

    // Apply force at center
    body.applyForce(Vector2(10.0f, 0.0f));  // 10 N to the right
    float dt = 1.0f;  // 1 second for easy calculation

    // Integrate: a = F/m = 10/2 = 5 m/s²
    // v = v0 + a*dt = 0 + 5*1 = 5 m/s
    body.integrate(dt);
    assert(approxEqual(body.getVelocity(), Vector2(5.0f, 0.0f)));
    std::cout << "  ✓ Force application: F=10N, m=2kg → v=" << body.getVelocity()
              << " (expected 5 m/s)" << std::endl;

    // Clear forces and apply again
    body.clearForces();
    body.setVelocity(Vector2::zero());

    // Multiple forces should accumulate
    body.applyForce(Vector2(6.0f, 0.0f));
    body.applyForce(Vector2(4.0f, 0.0f));
    body.integrate(dt);
    assert(approxEqual(body.getVelocity(), Vector2(5.0f, 0.0f)));  // Total 10N
    std::cout << "  ✓ Force accumulation: 6N + 4N = 10N → v=" << body.getVelocity() << std::endl;

    // Static bodies don't respond to forces
    body.clearForces();
    body.setStatic();
    body.applyForce(Vector2(100.0f, 0.0f));
    body.integrate(dt);
    assert(approxEqual(body.getVelocity(), Vector2::zero()));
    std::cout << "  ✓ Static body ignores forces" << std::endl;

    std::cout << "✓ Force application passed!\n" << std::endl;
}

void testTorqueApplication() {
    std::cout << "Testing torque application..." << std::endl;

    RigidBody body;
    body.setMomentOfInertia(2.0f);  // 2 kg⋅m²

    // Apply torque
    body.applyTorque(10.0f);  // 10 N⋅m counterclockwise
    float dt = 1.0f;

    // Integrate: α = τ/I = 10/2 = 5 rad/s²
    // ω = ω0 + α*dt = 0 + 5*1 = 5 rad/s
    body.integrate(dt);
    assert(approxEqual(body.getAngularVelocity(), 5.0f));
    std::cout << "  ✓ Torque application: τ=10N⋅m, I=2kg⋅m² → ω=" << body.getAngularVelocity()
              << " rad/s" << std::endl;

    // Check rotation changed
    // Note: 5.0 rad gets normalized to 5.0 - 2π ≈ -1.28 rad
    float expectedRotation = 5.0f - 2.0f * M_PI;
    assert(approxEqual(body.getRotation(), expectedRotation));
    std::cout << "  ✓ Rotation updated: θ=" << body.getRotation() << " rad (normalized)" << std::endl;

    std::cout << "✓ Torque application passed!\n" << std::endl;
}

void testForceAtPoint() {
    std::cout << "Testing force at point..." << std::endl;

    RigidBody body;
    body.setMass(1.0f);
    body.setMomentOfInertia(1.0f);
    body.setPosition(Vector2(0.0f, 0.0f));

    // Apply force at point (1, 0) - to the right of center
    // Force pointing up: (0, 10)
    Vector2 force(0.0f, 10.0f);
    Vector2 point(1.0f, 0.0f);

    body.applyForceAtPoint(force, point);

    // Expected:
    // - Linear acceleration: a = F/m = 10/1 = 10 m/s² upward
    // - Torque: τ = r × F = (1,0) × (0,10) = 1*10 - 0*0 = 10 N⋅m
    // - Angular acceleration: α = τ/I = 10/1 = 10 rad/s²

    float dt = 1.0f;
    body.integrate(dt);

    assert(approxEqual(body.getVelocity(), Vector2(0.0f, 10.0f)));
    assert(approxEqual(body.getAngularVelocity(), 10.0f));

    std::cout << "  ✓ Force at point: v=" << body.getVelocity()
              << ", ω=" << body.getAngularVelocity() << " rad/s" << std::endl;

    std::cout << "✓ Force at point passed!\n" << std::endl;
}

void testImpulseApplication() {
    std::cout << "Testing impulse application..." << std::endl;

    RigidBody body;
    body.setMass(2.0f);

    // Apply impulse (instantaneous velocity change)
    // Impulse = 10 kg⋅m/s to the right
    body.applyImpulse(Vector2(10.0f, 0.0f));

    // Velocity should change immediately: Δv = J/m = 10/2 = 5 m/s
    assert(approxEqual(body.getVelocity(), Vector2(5.0f, 0.0f)));
    std::cout << "  ✓ Linear impulse: J=10 kg⋅m/s, m=2kg → v=" << body.getVelocity() << std::endl;

    // Angular impulse
    body.setMomentOfInertia(4.0f);
    body.applyAngularImpulse(8.0f);  // 8 kg⋅m²/s

    // Angular velocity should change: Δω = impulse/I = 8/4 = 2 rad/s
    assert(approxEqual(body.getAngularVelocity(), 2.0f));
    std::cout << "  ✓ Angular impulse: J=8 kg⋅m²/s, I=4kg⋅m² → ω=" << body.getAngularVelocity()
              << " rad/s" << std::endl;

    std::cout << "✓ Impulse application passed!\n" << std::endl;
}

void testIntegration() {
    std::cout << "Testing physics integration..." << std::endl;

    RigidBody body;
    body.setMass(1.0f);
    body.setPosition(Vector2(0.0f, 10.0f));  // 10m above ground

    // Apply gravity: F = mg = 1 * 9.8 = 9.8 N downward
    float gravity = -9.8f;
    float dt = 0.1f;  // 100ms time step

    // Simulate falling for 1 second (10 steps)
    for (int i = 0; i < 10; ++i) {
        body.applyForce(Vector2(0.0f, gravity));
        body.integrate(dt);
        body.clearForces();
    }

    // After 1 second:
    // Velocity: v = v0 + a*t = 0 + (-9.8)*1 = -9.8 m/s (exact)
    //
    // Position with semi-implicit Euler:
    // Each step: v_new = v + a*dt, y_new = y + v_new*dt
    // This gives different result than analytical y = y0 + 0.5*a*t²
    // Semi-implicit Euler: y ≈ 4.61 m
    // Analytical solution: y = 5.1 m
    // Semi-implicit loses less energy, so object falls further

    assert(approxEqual(body.getVelocity().y, -9.8f, 0.01f));
    assert(approxEqual(body.getPosition().y, 4.61f, 0.05f));  // Semi-implicit Euler result

    std::cout << "  ✓ Falling body: y=" << body.getPosition().y
              << "m, vy=" << body.getVelocity().y << " m/s" << std::endl;

    std::cout << "✓ Integration passed!\n" << std::endl;
}

void testEnergyAndMomentum() {
    std::cout << "Testing energy and momentum..." << std::endl;

    RigidBody body;
    body.setMass(2.0f);
    body.setMomentOfInertia(3.0f);
    body.setVelocity(Vector2(4.0f, 0.0f));  // 4 m/s to the right
    body.setAngularVelocity(2.0f);           // 2 rad/s

    // Kinetic energy: KE = 0.5*m*v² + 0.5*I*ω²
    //                    = 0.5*2*16 + 0.5*3*4
    //                    = 16 + 6 = 22 J
    float ke = body.getKineticEnergy();
    assert(approxEqual(ke, 22.0f));
    std::cout << "  ✓ Kinetic energy: KE=" << ke << " J" << std::endl;

    // Linear momentum: p = m*v = 2*4 = 8 kg⋅m/s
    Vector2 momentum = body.getMomentum();
    assert(approxEqual(momentum, Vector2(8.0f, 0.0f)));
    std::cout << "  ✓ Linear momentum: p=" << momentum << " kg⋅m/s" << std::endl;

    // Angular momentum: L = I*ω = 3*2 = 6 kg⋅m²/s
    float angularMomentum = body.getAngularMomentum();
    assert(approxEqual(angularMomentum, 6.0f));
    std::cout << "  ✓ Angular momentum: L=" << angularMomentum << " kg⋅m²/s" << std::endl;

    std::cout << "✓ Energy and momentum passed!\n" << std::endl;
}

void testVelocityAtPoint() {
    std::cout << "Testing velocity at point..." << std::endl;

    RigidBody body;
    body.setPosition(Vector2(0.0f, 0.0f));
    body.setVelocity(Vector2(5.0f, 0.0f));      // Moving right at 5 m/s
    body.setAngularVelocity(2.0f);               // Rotating CCW at 2 rad/s

    // Point at (0, 1) - 1 meter above center
    Vector2 point(0.0f, 1.0f);
    Vector2 velocityAtPoint = body.getVelocityAtPoint(point);

    // Expected velocity:
    // v_point = v_center + ω × r
    // r = (0, 1)
    // ω × r = 2 * (-1, 0) = (-2, 0)  [perpendicular to r, scaled by ω]
    // v_point = (5, 0) + (-2, 0) = (3, 0)

    assert(approxEqual(velocityAtPoint, Vector2(3.0f, 0.0f)));
    std::cout << "  ✓ Velocity at point (0,1): v=" << velocityAtPoint << " m/s" << std::endl;

    // Point at (1, 0) - 1 meter to the right
    point = Vector2(1.0f, 0.0f);
    velocityAtPoint = body.getVelocityAtPoint(point);

    // r = (1, 0)
    // ω × r = 2 * (0, 1) = (0, 2)
    // v_point = (5, 0) + (0, 2) = (5, 2)

    assert(approxEqual(velocityAtPoint, Vector2(5.0f, 2.0f)));
    std::cout << "  ✓ Velocity at point (1,0): v=" << velocityAtPoint << " m/s" << std::endl;

    std::cout << "✓ Velocity at point passed!\n" << std::endl;
}

void testCoordinateTransforms() {
    std::cout << "Testing coordinate transformations..." << std::endl;

    RigidBody body;
    body.setPosition(Vector2(10.0f, 20.0f));
    body.setRotation(M_PI / 2.0f);  // 90° rotation

    // Local point (1, 0) - "forward" in body's space
    Vector2 localPoint(1.0f, 0.0f);
    Vector2 worldPoint = body.localToWorld(localPoint);

    // After 90° rotation, (1,0) becomes (0,1), then add position
    assert(approxEqual(worldPoint, Vector2(10.0f, 21.0f)));
    std::cout << "  ✓ Local to world: " << localPoint << " → " << worldPoint << std::endl;

    // Inverse: world to local
    Vector2 backToLocal = body.worldToLocal(worldPoint);
    assert(approxEqual(backToLocal, localPoint));
    std::cout << "  ✓ World to local: " << worldPoint << " → " << backToLocal << std::endl;

    std::cout << "✓ Coordinate transformations passed!\n" << std::endl;
}

void testMaterialProperties() {
    std::cout << "Testing material properties..." << std::endl;

    RigidBody body;

    // Set restitution
    body.setRestitution(0.8f);
    assert(approxEqual(body.getRestitution(), 0.8f));
    std::cout << "  ✓ Restitution: " << body.getRestitution() << std::endl;

    // Restitution should be clamped to [0, 1]
    body.setRestitution(1.5f);
    assert(approxEqual(body.getRestitution(), 1.0f));
    std::cout << "  ✓ Restitution clamped: 1.5 → " << body.getRestitution() << std::endl;

    body.setRestitution(-0.5f);
    assert(approxEqual(body.getRestitution(), 0.0f));
    std::cout << "  ✓ Restitution clamped: -0.5 → " << body.getRestitution() << std::endl;

    // Set friction
    body.setFriction(0.6f);
    assert(approxEqual(body.getFriction(), 0.6f));
    std::cout << "  ✓ Friction: " << body.getFriction() << std::endl;

    // Friction can be > 1, but not negative
    body.setFriction(1.5f);
    assert(approxEqual(body.getFriction(), 1.5f));
    std::cout << "  ✓ Friction allows > 1: " << body.getFriction() << std::endl;

    body.setFriction(-0.3f);
    assert(approxEqual(body.getFriction(), 0.0f));
    std::cout << "  ✓ Friction clamped: -0.3 → " << body.getFriction() << std::endl;

    std::cout << "✓ Material properties passed!\n" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "      RigidBody Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;

    try {
        testConstructors();
        testMassAndInertia();
        testStaticVsDynamic();
        testForceApplication();
        testTorqueApplication();
        testForceAtPoint();
        testImpulseApplication();
        testIntegration();
        testEnergyAndMomentum();
        testVelocityAtPoint();
        testCoordinateTransforms();
        testMaterialProperties();

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
