/**
 * @file test_transform.cpp
 * @brief Test suite for Transform class
 *
 * Comprehensive tests for all Transform operations including:
 * - Point and direction transformations
 * - Inverse transformations
 * - Transform combination
 * - Directional vectors
 * - Utility functions
 */

#include "math/Transform.hpp"
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

void testConstructors() {
    std::cout << "Testing constructors..." << std::endl;

    // Default constructor (identity)
    Transform t1;
    assert(approxEqual(t1.position, Vector2::zero()));
    assert(approxEqual(t1.rotation, 0.0f));
    std::cout << "  ✓ Default constructor: pos=" << t1.position
              << ", rot=" << t1.rotation << std::endl;

    // Parameterized constructor (position + rotation)
    Transform t2(Vector2(5.0f, 10.0f), M_PI / 4.0f);
    assert(approxEqual(t2.position, Vector2(5.0f, 10.0f)));
    assert(approxEqual(t2.rotation, M_PI / 4.0f));
    std::cout << "  ✓ Parameterized constructor: pos=" << t2.position
              << ", rot=" << t2.rotation << " rad" << std::endl;

    // Position-only constructor
    Transform t3(Vector2(3.0f, 7.0f));
    assert(approxEqual(t3.position, Vector2(3.0f, 7.0f)));
    assert(approxEqual(t3.rotation, 0.0f));
    std::cout << "  ✓ Position-only constructor: pos=" << t3.position
              << ", rot=" << t3.rotation << std::endl;

    std::cout << "✓ Constructors passed!\n" << std::endl;
}

void testTransformPoint() {
    std::cout << "Testing point transformation..." << std::endl;

    // Test 1: No rotation, only translation
    Transform t1(Vector2(10.0f, 20.0f), 0.0f);
    Vector2 local1(1.0f, 2.0f);
    Vector2 world1 = t1.transformPoint(local1);
    assert(approxEqual(world1, Vector2(11.0f, 22.0f)));
    std::cout << "  ✓ Translation only: " << local1 << " -> " << world1 << std::endl;

    // Test 2: Rotation only (90 degrees = π/2)
    Transform t2(Vector2::zero(), M_PI / 2.0f);
    Vector2 local2(1.0f, 0.0f);  // Point to the right
    Vector2 world2 = t2.transformPoint(local2);
    assert(approxEqual(world2, Vector2(0.0f, 1.0f)));  // Should point up after 90° rotation
    std::cout << "  ✓ Rotation 90°: " << local2 << " -> " << world2 << std::endl;

    // Test 3: Both rotation and translation
    Transform t3(Vector2(5.0f, 5.0f), M_PI / 2.0f);
    Vector2 local3(1.0f, 0.0f);
    Vector2 world3 = t3.transformPoint(local3);
    assert(approxEqual(world3, Vector2(5.0f, 6.0f)));  // Rotated up, then translated
    std::cout << "  ✓ Rotation + translation: " << local3 << " -> " << world3 << std::endl;

    // Test 4: 180 degree rotation
    Transform t4(Vector2::zero(), M_PI);
    Vector2 local4(1.0f, 0.0f);
    Vector2 world4 = t4.transformPoint(local4);
    assert(approxEqual(world4, Vector2(-1.0f, 0.0f)));  // Should flip
    std::cout << "  ✓ Rotation 180°: " << local4 << " -> " << world4 << std::endl;

    std::cout << "✓ Point transformation passed!\n" << std::endl;
}

void testTransformDirection() {
    std::cout << "Testing direction transformation..." << std::endl;

    // Test 1: Direction is not affected by position
    Transform t1(Vector2(100.0f, 200.0f), 0.0f);
    Vector2 dir1(1.0f, 0.0f);
    Vector2 worldDir1 = t1.transformDirection(dir1);
    assert(approxEqual(worldDir1, Vector2(1.0f, 0.0f)));  // Unchanged
    std::cout << "  ✓ Position doesn't affect direction: " << dir1
              << " -> " << worldDir1 << std::endl;

    // Test 2: Direction IS affected by rotation
    Transform t2(Vector2(100.0f, 200.0f), M_PI / 2.0f);
    Vector2 dir2(1.0f, 0.0f);
    Vector2 worldDir2 = t2.transformDirection(dir2);
    assert(approxEqual(worldDir2, Vector2(0.0f, 1.0f)));  // Rotated 90°
    std::cout << "  ✓ Rotation affects direction: " << dir2
              << " -> " << worldDir2 << std::endl;

    // Test 3: 45 degree rotation
    Transform t3(Vector2::zero(), M_PI / 4.0f);
    Vector2 dir3(1.0f, 0.0f);
    Vector2 worldDir3 = t3.transformDirection(dir3);
    float sqrt2_inv = 1.0f / std::sqrt(2.0f);
    assert(approxEqual(worldDir3, Vector2(sqrt2_inv, sqrt2_inv)));
    std::cout << "  ✓ Rotation 45°: " << dir3 << " -> " << worldDir3 << std::endl;

    std::cout << "✓ Direction transformation passed!\n" << std::endl;
}

void testInverseTransform() {
    std::cout << "Testing inverse transformation..." << std::endl;

    // Test 1: Transform and inverse should cancel out (points)
    Transform t1(Vector2(5.0f, 10.0f), M_PI / 3.0f);
    Vector2 original(3.0f, 7.0f);
    Vector2 world = t1.transformPoint(original);
    Vector2 back = t1.inverseTransformPoint(world);
    assert(approxEqual(back, original));
    std::cout << "  ✓ Transform + inverse (point): " << original
              << " -> " << world << " -> " << back << std::endl;

    // Test 2: Transform and inverse should cancel out (directions)
    Vector2 originalDir(1.0f, 1.0f);
    Vector2 worldDir = t1.transformDirection(originalDir);
    Vector2 backDir = t1.inverseTransformDirection(worldDir);
    assert(approxEqual(backDir, originalDir));
    std::cout << "  ✓ Transform + inverse (direction): " << originalDir
              << " -> " << worldDir << " -> " << backDir << std::endl;

    // Test 3: Using inverse() method
    Transform t2(Vector2(10.0f, 20.0f), M_PI / 4.0f);
    Transform t2inv = t2.inverse();
    Vector2 point(5.0f, 5.0f);
    Vector2 transformed = t2.transformPoint(point);
    Vector2 backAgain = t2inv.transformPoint(transformed);
    assert(approxEqual(backAgain, point));
    std::cout << "  ✓ Using inverse() method: " << point
              << " -> " << transformed << " -> " << backAgain << std::endl;

    std::cout << "✓ Inverse transformation passed!\n" << std::endl;
}

void testCombineTransforms() {
    std::cout << "Testing transform combination..." << std::endl;

    // Test 1: Combining two translations
    Transform t1(Vector2(5.0f, 10.0f), 0.0f);
    Transform t2(Vector2(3.0f, 7.0f), 0.0f);
    Transform combined = t1.combine(t2);
    assert(approxEqual(combined.position, Vector2(8.0f, 17.0f)));
    assert(approxEqual(combined.rotation, 0.0f));
    std::cout << "  ✓ Combined translations: " << combined.position << std::endl;

    // Test 2: Combining rotations
    Transform t3(Vector2::zero(), M_PI / 4.0f);
    Transform t4(Vector2::zero(), M_PI / 4.0f);
    Transform combined2 = t3.combine(t4);
    assert(approxEqual(combined2.rotation, M_PI / 2.0f));
    std::cout << "  ✓ Combined rotations: " << combined2.rotation
              << " rad (π/2)" << std::endl;

    // Test 3: Complex combination (parent-child)
    // Parent at (10, 0), rotated 90°
    Transform parent(Vector2(10.0f, 0.0f), M_PI / 2.0f);
    // Child at (5, 0) relative to parent (5 units "forward")
    Transform childLocal(Vector2(5.0f, 0.0f), 0.0f);
    Transform childWorld = childLocal.combine(parent);
    // After parent rotation of 90°, child's (5,0) becomes (0,5)
    // Then add parent position (10,0): result should be (10, 5)
    assert(approxEqual(childWorld.position, Vector2(10.0f, 5.0f)));
    std::cout << "  ✓ Parent-child: child at " << childWorld.position << std::endl;

    std::cout << "✓ Transform combination passed!\n" << std::endl;
}

void testDirectionalVectors() {
    std::cout << "Testing directional vectors..." << std::endl;

    // Test 1: No rotation (pointing right)
    Transform t1(Vector2::zero(), 0.0f);
    Vector2 forward1 = t1.getForward();
    Vector2 right1 = t1.getRight();
    Vector2 up1 = t1.getUp();
    assert(approxEqual(forward1, Vector2(1.0f, 0.0f)));   // Right
    assert(approxEqual(right1, Vector2(0.0f, -1.0f)));    // Down
    assert(approxEqual(up1, Vector2(0.0f, 1.0f)));        // Up
    std::cout << "  ✓ No rotation: forward=" << forward1
              << ", right=" << right1 << ", up=" << up1 << std::endl;

    // Test 2: 90 degree rotation (pointing up)
    Transform t2(Vector2::zero(), M_PI / 2.0f);
    Vector2 forward2 = t2.getForward();
    Vector2 right2 = t2.getRight();
    Vector2 up2 = t2.getUp();
    assert(approxEqual(forward2, Vector2(0.0f, 1.0f)));   // Up
    assert(approxEqual(right2, Vector2(1.0f, 0.0f)));     // Right
    assert(approxEqual(up2, Vector2(-1.0f, 0.0f)));       // Left
    std::cout << "  ✓ 90° rotation: forward=" << forward2
              << ", right=" << right2 << ", up=" << up2 << std::endl;

    // Test 3: Verify perpendicularity (dot product should be 0)
    Transform t3(Vector2::zero(), M_PI / 3.0f);  // 60 degrees
    Vector2 forward3 = t3.getForward();
    Vector2 right3 = t3.getRight();
    Vector2 up3 = t3.getUp();
    assert(approxEqual(forward3.dot(right3), 0.0f));
    assert(approxEqual(forward3.dot(up3), 0.0f));
    std::cout << "  ✓ Perpendicularity verified (dot products ≈ 0)" << std::endl;

    // Test 4: Verify unit length
    assert(approxEqual(forward3.length(), 1.0f));
    assert(approxEqual(right3.length(), 1.0f));
    assert(approxEqual(up3.length(), 1.0f));
    std::cout << "  ✓ Unit length verified (all ≈ 1)" << std::endl;

    std::cout << "✓ Directional vectors passed!\n" << std::endl;
}

void testUtilityFunctions() {
    std::cout << "Testing utility functions..." << std::endl;

    // Test 1: Rotation normalization
    Transform t1(Vector2::zero(), 3.0f * M_PI);  // 540 degrees
    t1.normalizeRotation();
    assert(approxEqual(t1.rotation, -M_PI));  // Should normalize to -180°
    std::cout << "  ✓ Normalize 3π -> -π: " << t1.rotation << " rad" << std::endl;

    Transform t2(Vector2::zero(), -3.0f * M_PI);
    t2.normalizeRotation();
    assert(approxEqual(t2.rotation, -M_PI));
    std::cout << "  ✓ Normalize -3π -> -π: " << t2.rotation << " rad" << std::endl;

    Transform t3(Vector2::zero(), 2.0f * M_PI);  // 360 degrees
    t3.normalizeRotation();
    assert(approxEqual(t3.rotation, 0.0f));
    std::cout << "  ✓ Normalize 2π -> 0: " << t3.rotation << " rad" << std::endl;

    // Test 2: Rotate function
    Transform t4(Vector2::zero(), M_PI / 4.0f);
    t4.rotate(M_PI / 4.0f);
    assert(approxEqual(t4.rotation, M_PI / 2.0f));
    std::cout << "  ✓ Rotate by π/4: " << t4.rotation << " rad (π/2)" << std::endl;

    // Test 3: Translate function
    Transform t5(Vector2(5.0f, 10.0f), 0.0f);
    t5.translate(Vector2(3.0f, 7.0f));
    assert(approxEqual(t5.position, Vector2(8.0f, 17.0f)));
    std::cout << "  ✓ Translate: " << t5.position << std::endl;

    // Test 4: LookAt function
    Transform t6(Vector2(0.0f, 0.0f), 0.0f);
    t6.lookAt(Vector2(1.0f, 1.0f));  // Look toward (1, 1) = 45 degrees
    assert(approxEqual(t6.rotation, M_PI / 4.0f));
    std::cout << "  ✓ LookAt (1,1): rotation=" << t6.rotation << " rad (π/4)" << std::endl;

    Transform t7(Vector2(0.0f, 0.0f), 0.0f);
    t7.lookAt(Vector2(0.0f, 1.0f));  // Look up = 90 degrees
    assert(approxEqual(t7.rotation, M_PI / 2.0f));
    std::cout << "  ✓ LookAt (0,1): rotation=" << t7.rotation << " rad (π/2)" << std::endl;

    std::cout << "✓ Utility functions passed!\n" << std::endl;
}

void testStaticHelpers() {
    std::cout << "Testing static helper functions..." << std::endl;

    // Test 1: Identity transform
    Transform identity = Transform::identity();
    assert(approxEqual(identity.position, Vector2::zero()));
    assert(approxEqual(identity.rotation, 0.0f));
    std::cout << "  ✓ identity(): pos=" << identity.position
              << ", rot=" << identity.rotation << std::endl;

    // Test 2: Translation transform
    Transform trans = Transform::translation(Vector2(10.0f, 20.0f));
    assert(approxEqual(trans.position, Vector2(10.0f, 20.0f)));
    assert(approxEqual(trans.rotation, 0.0f));
    std::cout << "  ✓ translation(): pos=" << trans.position
              << ", rot=" << trans.rotation << std::endl;

    // Test 3: Rotation transform
    Transform rot = Transform::rotationTransform(M_PI / 3.0f);
    assert(approxEqual(rot.position, Vector2::zero()));
    assert(approxEqual(rot.rotation, M_PI / 3.0f));
    std::cout << "  ✓ rotationTransform(): pos=" << rot.position
              << ", rot=" << rot.rotation << " rad" << std::endl;

    std::cout << "✓ Static helpers passed!\n" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "      Transform Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;

    try {
        testConstructors();
        testTransformPoint();
        testTransformDirection();
        testInverseTransform();
        testCombineTransforms();
        testDirectionalVectors();
        testUtilityFunctions();
        testStaticHelpers();

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
