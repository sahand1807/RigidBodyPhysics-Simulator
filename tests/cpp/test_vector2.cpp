/**
 * @file test_vector2.cpp
 * @brief Test suite for Vector2 class
 *
 * This file tests all Vector2 operations to ensure correctness.
 */

#include "math/Vector2.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace Physics;

// Helper function to check if two floats are approximately equal
bool approxEqual(float a, float b, float epsilon = 1e-5f) {
    return std::abs(a - b) < epsilon;
}

// Helper function to check if two vectors are approximately equal
bool approxEqual(const Vector2& a, const Vector2& b, float epsilon = 1e-5f) {
    return approxEqual(a.x, b.x, epsilon) && approxEqual(a.y, b.y, epsilon);
}

void testConstructors() {
    std::cout << "Testing constructors..." << std::endl;

    // Default constructor
    Vector2 v1;
    assert(v1.x == 0.0f && v1.y == 0.0f);
    std::cout << "  ✓ Default constructor: " << v1 << std::endl;

    // Parameterized constructor
    Vector2 v2(3.0f, 4.0f);
    assert(v2.x == 3.0f && v2.y == 4.0f);
    std::cout << "  ✓ Parameterized constructor: " << v2 << std::endl;

    std::cout << "✓ Constructors passed!\n" << std::endl;
}

void testArithmeticOperations() {
    std::cout << "Testing arithmetic operations..." << std::endl;

    Vector2 v1(1.0f, 2.0f);
    Vector2 v2(3.0f, 4.0f);

    // Addition
    Vector2 sum = v1 + v2;
    assert(approxEqual(sum, Vector2(4.0f, 6.0f)));
    std::cout << "  ✓ Addition: " << v1 << " + " << v2 << " = " << sum << std::endl;

    // Subtraction
    Vector2 diff = v2 - v1;
    assert(approxEqual(diff, Vector2(2.0f, 2.0f)));
    std::cout << "  ✓ Subtraction: " << v2 << " - " << v1 << " = " << diff << std::endl;

    // Scalar multiplication
    Vector2 scaled = v1 * 2.0f;
    assert(approxEqual(scaled, Vector2(2.0f, 4.0f)));
    std::cout << "  ✓ Scalar multiply: " << v1 << " * 2 = " << scaled << std::endl;

    // Scalar multiplication (commutative)
    Vector2 scaled2 = 2.0f * v1;
    assert(approxEqual(scaled2, Vector2(2.0f, 4.0f)));
    std::cout << "  ✓ Scalar multiply (reverse): 2 * " << v1 << " = " << scaled2 << std::endl;

    // Division
    Vector2 divided = v2 / 2.0f;
    assert(approxEqual(divided, Vector2(1.5f, 2.0f)));
    std::cout << "  ✓ Division: " << v2 << " / 2 = " << divided << std::endl;

    // Negation
    Vector2 negated = -v1;
    assert(approxEqual(negated, Vector2(-1.0f, -2.0f)));
    std::cout << "  ✓ Negation: -" << v1 << " = " << negated << std::endl;

    std::cout << "✓ Arithmetic operations passed!\n" << std::endl;
}

void testCompoundOperators() {
    std::cout << "Testing compound operators..." << std::endl;

    Vector2 v(1.0f, 2.0f);

    // +=
    v += Vector2(2.0f, 3.0f);
    assert(approxEqual(v, Vector2(3.0f, 5.0f)));
    std::cout << "  ✓ += : " << v << std::endl;

    // -=
    v -= Vector2(1.0f, 1.0f);
    assert(approxEqual(v, Vector2(2.0f, 4.0f)));
    std::cout << "  ✓ -= : " << v << std::endl;

    // *=
    v *= 2.0f;
    assert(approxEqual(v, Vector2(4.0f, 8.0f)));
    std::cout << "  ✓ *= : " << v << std::endl;

    // /=
    v /= 2.0f;
    assert(approxEqual(v, Vector2(2.0f, 4.0f)));
    std::cout << "  ✓ /= : " << v << std::endl;

    std::cout << "✓ Compound operators passed!\n" << std::endl;
}

void testDotProduct() {
    std::cout << "Testing dot product..." << std::endl;

    Vector2 v1(1.0f, 0.0f);
    Vector2 v2(0.0f, 1.0f);
    Vector2 v3(1.0f, 1.0f);

    // Perpendicular vectors (should be 0)
    float dot1 = v1.dot(v2);
    assert(approxEqual(dot1, 0.0f));
    std::cout << "  ✓ Perpendicular: " << v1 << " · " << v2 << " = " << dot1 << std::endl;

    // Parallel vectors (same direction)
    float dot2 = v1.dot(v1);
    assert(approxEqual(dot2, 1.0f));
    std::cout << "  ✓ Parallel: " << v1 << " · " << v1 << " = " << dot2 << std::endl;

    // General case
    float dot3 = v1.dot(v3);
    assert(approxEqual(dot3, 1.0f));
    std::cout << "  ✓ General: " << v1 << " · " << v3 << " = " << dot3 << std::endl;

    std::cout << "✓ Dot product passed!\n" << std::endl;
}

void testCrossProduct() {
    std::cout << "Testing cross product..." << std::endl;

    Vector2 v1(1.0f, 0.0f);
    Vector2 v2(0.0f, 1.0f);
    Vector2 v3(1.0f, 1.0f);

    // Right-hand rule: (1,0) x (0,1) = positive (counterclockwise)
    float cross1 = v1.cross(v2);
    assert(cross1 > 0.0f);
    std::cout << "  ✓ Counterclockwise: " << v1 << " × " << v2 << " = " << cross1 << std::endl;

    // Left-hand rule: (0,1) x (1,0) = negative (clockwise)
    float cross2 = v2.cross(v1);
    assert(cross2 < 0.0f);
    std::cout << "  ✓ Clockwise: " << v2 << " × " << v1 << " = " << cross2 << std::endl;

    // Parallel vectors (should be 0)
    float cross3 = v1.cross(v1);
    assert(approxEqual(cross3, 0.0f));
    std::cout << "  ✓ Parallel: " << v1 << " × " << v1 << " = " << cross3 << std::endl;

    std::cout << "✓ Cross product passed!\n" << std::endl;
}

void testLengthAndNormalization() {
    std::cout << "Testing length and normalization..." << std::endl;

    // Classic 3-4-5 triangle
    Vector2 v1(3.0f, 4.0f);
    float len = v1.length();
    assert(approxEqual(len, 5.0f));
    std::cout << "  ✓ Length of " << v1 << " = " << len << std::endl;

    // Length squared (should be 25)
    float lenSq = v1.lengthSquared();
    assert(approxEqual(lenSq, 25.0f));
    std::cout << "  ✓ Length squared = " << lenSq << std::endl;

    // Normalize (should become (0.6, 0.8))
    Vector2 v2 = v1.normalized();
    assert(approxEqual(v2.length(), 1.0f));
    assert(approxEqual(v2, Vector2(0.6f, 0.8f)));
    std::cout << "  ✓ Normalized " << v1 << " = " << v2 << " (length = " << v2.length() << ")" << std::endl;

    // In-place normalization
    Vector2 v3(3.0f, 4.0f);
    v3.normalize();
    assert(approxEqual(v3.length(), 1.0f));
    std::cout << "  ✓ In-place normalize: " << v3 << " (length = " << v3.length() << ")" << std::endl;

    // Zero vector normalization (should remain zero)
    Vector2 zero = Vector2::zero();
    zero.normalize();
    assert(zero.isZero());
    std::cout << "  ✓ Zero vector normalization: " << zero << std::endl;

    std::cout << "✓ Length and normalization passed!\n" << std::endl;
}

void testDistance() {
    std::cout << "Testing distance..." << std::endl;

    Vector2 p1(0.0f, 0.0f);
    Vector2 p2(3.0f, 4.0f);

    // Distance (should be 5)
    float dist = p1.distance(p2);
    assert(approxEqual(dist, 5.0f));
    std::cout << "  ✓ Distance from " << p1 << " to " << p2 << " = " << dist << std::endl;

    // Distance squared (should be 25)
    float distSq = p1.distanceSquared(p2);
    assert(approxEqual(distSq, 25.0f));
    std::cout << "  ✓ Distance squared = " << distSq << std::endl;

    std::cout << "✓ Distance passed!\n" << std::endl;
}

void testUtilityFunctions() {
    std::cout << "Testing utility functions..." << std::endl;

    // Zero vector
    Vector2 zero = Vector2::zero();
    assert(zero.isZero());
    std::cout << "  ✓ Zero vector: " << zero << " (isZero = true)" << std::endl;

    // Non-zero vector
    Vector2 nonZero(1.0f, 2.0f);
    assert(!nonZero.isZero());
    std::cout << "  ✓ Non-zero vector: " << nonZero << " (isZero = false)" << std::endl;

    // Perpendicular vector
    Vector2 v(3.0f, 4.0f);
    Vector2 perp = v.perpendicular();
    assert(approxEqual(perp, Vector2(-4.0f, 3.0f)));
    // Verify they are perpendicular (dot product should be 0)
    assert(approxEqual(v.dot(perp), 0.0f));
    std::cout << "  ✓ Perpendicular of " << v << " = " << perp
              << " (dot product = " << v.dot(perp) << ")" << std::endl;

    std::cout << "✓ Utility functions passed!\n" << std::endl;
}

void testStaticHelpers() {
    std::cout << "Testing static helpers..." << std::endl;

    Vector2 zero = Vector2::zero();
    assert(approxEqual(zero, Vector2(0.0f, 0.0f)));
    std::cout << "  ✓ zero(): " << zero << std::endl;

    Vector2 right = Vector2::right();
    assert(approxEqual(right, Vector2(1.0f, 0.0f)));
    std::cout << "  ✓ right(): " << right << std::endl;

    Vector2 up = Vector2::up();
    assert(approxEqual(up, Vector2(0.0f, 1.0f)));
    std::cout << "  ✓ up(): " << up << std::endl;

    std::cout << "✓ Static helpers passed!\n" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "      Vector2 Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;

    try {
        testConstructors();
        testArithmeticOperations();
        testCompoundOperators();
        testDotProduct();
        testCrossProduct();
        testLengthAndNormalization();
        testDistance();
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