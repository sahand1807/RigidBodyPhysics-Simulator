/**
 * @file test_collider.cpp
 * @brief Test suite for Collider and CircleCollider classes
 *
 * Comprehensive tests for collision shapes including:
 * - Construction and type identification
 * - Mass and inertia calculations
 * - Bounding box calculations
 * - Point containment
 * - Offset and world space transformations
 */

#include "physics/Collider.hpp"
#include "physics/CircleCollider.hpp"
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

void testCircleConstruction() {
    std::cout << "Testing circle construction..." << std::endl;

    // Basic construction
    CircleCollider circle1(1.0f);
    assert(approxEqual(circle1.getRadius(), 1.0f));
    assert(approxEqual(circle1.getOffset(), Vector2::zero()));
    assert(circle1.getType() == Collider::Type::Circle);
    std::cout << "  ✓ Basic construction: radius=" << circle1.getRadius() << std::endl;

    // Construction with offset
    CircleCollider circle2(2.0f, Vector2(3.0f, 4.0f));
    assert(approxEqual(circle2.getRadius(), 2.0f));
    assert(approxEqual(circle2.getOffset(), Vector2(3.0f, 4.0f)));
    std::cout << "  ✓ Construction with offset: radius=" << circle2.getRadius()
              << ", offset=" << circle2.getOffset() << std::endl;

    // Invalid radius (should default to 1.0)
    CircleCollider circle3(-5.0f);
    assert(approxEqual(circle3.getRadius(), 1.0f));
    std::cout << "  ✓ Invalid radius handled: " << circle3.getRadius() << std::endl;

    std::cout << "✓ Circle construction passed!\n" << std::endl;
}

void testMassCalculation() {
    std::cout << "Testing mass calculation..." << std::endl;

    CircleCollider circle(1.0f);  // radius = 1m

    // Mass formula: m = density × π × r²
    float density = 1.0f;
    float expectedMass = density * M_PI * 1.0f * 1.0f;  // π kg
    float mass = circle.calculateMass(density);

    assert(approxEqual(mass, expectedMass));
    std::cout << "  ✓ Mass for r=1m, ρ=1 kg/m²: " << mass
              << " kg (expected π ≈ " << M_PI << ")" << std::endl;

    // Different density
    CircleCollider circle2(2.0f);  // radius = 2m
    float woodDensity = 0.6f;
    float expectedMass2 = woodDensity * M_PI * 4.0f;  // 0.6 × π × 4
    float mass2 = circle2.calculateMass(woodDensity);

    assert(approxEqual(mass2, expectedMass2));
    std::cout << "  ✓ Mass for r=2m, ρ=0.6 kg/m²: " << mass2 << " kg" << std::endl;

    // Verify mass scales with r²
    CircleCollider small(1.0f);
    CircleCollider large(2.0f);
    float massSmall = small.calculateMass(1.0f);
    float massLarge = large.calculateMass(1.0f);

    // Large should be 4× heavier (r² relationship)
    assert(approxEqual(massLarge / massSmall, 4.0f));
    std::cout << "  ✓ Mass scales with r²: ratio=" << (massLarge / massSmall) << std::endl;

    std::cout << "✓ Mass calculation passed!\n" << std::endl;
}

void testInertiaCalculation() {
    std::cout << "Testing moment of inertia calculation..." << std::endl;

    // Formula: I = 0.5 × m × r²
    CircleCollider circle(1.0f);  // radius = 1m
    float mass = circle.calculateMass(1.0f);
    float inertia = circle.calculateInertia(mass);

    float expectedInertia = 0.5f * mass * 1.0f * 1.0f;
    assert(approxEqual(inertia, expectedInertia));
    std::cout << "  ✓ Inertia for m=" << mass << " kg, r=1m: "
              << inertia << " kg⋅m²" << std::endl;

    // Larger radius
    CircleCollider circle2(2.0f);  // radius = 2m
    float mass2 = circle2.calculateMass(1.0f);
    float inertia2 = circle2.calculateInertia(mass2);

    float expectedInertia2 = 0.5f * mass2 * 4.0f;  // r² = 4
    assert(approxEqual(inertia2, expectedInertia2));
    std::cout << "  ✓ Inertia for m=" << mass2 << " kg, r=2m: "
              << inertia2 << " kg⋅m²" << std::endl;

    // Verify inertia relationship
    // mass2 = 4 × mass1 (r² from area)
    // inertia2 = 0.5 × (4×mass1) × (2r)² = 0.5 × 4 × mass1 × 4 = 16 × (0.5 × mass1 × 1)
    // Inertia scales with r^4! (I = 0.5 × m × r², m scales with r²)
    // So inertia2 should be 16× inertia1
    float ratio = inertia2 / inertia;
    assert(approxEqual(ratio, 16.0f, 0.01f));
    std::cout << "  ✓ Inertia ratio (2m / 1m): " << ratio << "× (expected 16×)" << std::endl;

    std::cout << "✓ Inertia calculation passed!\n" << std::endl;
}

void testBoundsCalculation() {
    std::cout << "Testing bounding box calculation..." << std::endl;

    // Circle at origin
    CircleCollider circle1(1.0f);
    Vector2 min1, max1;
    circle1.getBounds(min1, max1);

    assert(approxEqual(min1, Vector2(-1.0f, -1.0f)));
    assert(approxEqual(max1, Vector2(1.0f, 1.0f)));
    std::cout << "  ✓ Bounds at origin: min=" << min1 << ", max=" << max1 << std::endl;

    // Circle with offset
    CircleCollider circle2(2.0f, Vector2(5.0f, 10.0f));
    Vector2 min2, max2;
    circle2.getBounds(min2, max2);

    assert(approxEqual(min2, Vector2(3.0f, 8.0f)));   // 5-2, 10-2
    assert(approxEqual(max2, Vector2(7.0f, 12.0f)));  // 5+2, 10+2
    std::cout << "  ✓ Bounds with offset: min=" << min2 << ", max=" << max2 << std::endl;

    // Verify AABB size
    Vector2 size = max2 - min2;
    assert(approxEqual(size, Vector2(4.0f, 4.0f)));  // 2 × radius
    std::cout << "  ✓ AABB size: " << size << " (2×radius = 4)" << std::endl;

    std::cout << "✓ Bounds calculation passed!\n" << std::endl;
}

void testPointContainment() {
    std::cout << "Testing point containment..." << std::endl;

    CircleCollider circle(1.0f, Vector2::zero());

    // Point at center (inside)
    assert(circle.containsPoint(Vector2(0.0f, 0.0f)));
    std::cout << "  ✓ Center point (0,0) is inside" << std::endl;

    // Point on circle (boundary)
    assert(circle.containsPoint(Vector2(1.0f, 0.0f)));
    std::cout << "  ✓ Boundary point (1,0) is inside" << std::endl;

    // Point just inside
    assert(circle.containsPoint(Vector2(0.5f, 0.5f)));
    std::cout << "  ✓ Point (0.5,0.5) is inside" << std::endl;

    // Point outside
    assert(!circle.containsPoint(Vector2(2.0f, 0.0f)));
    std::cout << "  ✓ Point (2,0) is outside" << std::endl;

    assert(!circle.containsPoint(Vector2(1.0f, 1.0f)));
    std::cout << "  ✓ Point (1,1) is outside (√2 > 1)" << std::endl;

    // Circle with offset
    CircleCollider circle2(2.0f, Vector2(10.0f, 20.0f));
    assert(circle2.containsPoint(Vector2(10.0f, 20.0f)));  // Center
    assert(circle2.containsPoint(Vector2(11.0f, 20.0f)));  // Inside
    assert(!circle2.containsPoint(Vector2(15.0f, 20.0f))); // Outside
    std::cout << "  ✓ Containment with offset works" << std::endl;

    std::cout << "✓ Point containment passed!\n" << std::endl;
}

void testOffsetFunctionality() {
    std::cout << "Testing offset functionality..." << std::endl;

    CircleCollider circle(1.0f);

    // Default offset
    assert(approxEqual(circle.getOffset(), Vector2::zero()));
    std::cout << "  ✓ Default offset: " << circle.getOffset() << std::endl;

    // Set offset
    circle.setOffset(Vector2(5.0f, 10.0f));
    assert(approxEqual(circle.getOffset(), Vector2(5.0f, 10.0f)));
    std::cout << "  ✓ Set offset: " << circle.getOffset() << std::endl;

    // Verify bounds updated with new offset
    Vector2 min, max;
    circle.getBounds(min, max);
    assert(approxEqual(min, Vector2(4.0f, 9.0f)));   // 5-1, 10-1
    assert(approxEqual(max, Vector2(6.0f, 11.0f)));  // 5+1, 10+1
    std::cout << "  ✓ Bounds updated: min=" << min << ", max=" << max << std::endl;

    std::cout << "✓ Offset functionality passed!\n" << std::endl;
}

void testWorldCenterCalculation() {
    std::cout << "Testing world center calculation..." << std::endl;

    // Circle with offset
    CircleCollider circle(1.0f, Vector2(2.0f, 0.0f));  // 2 units to the right

    // Body at origin, no rotation
    Transform bodyTransform1(Vector2::zero(), 0.0f);
    Vector2 worldCenter1 = circle.getWorldCenter(bodyTransform1);
    assert(approxEqual(worldCenter1, Vector2(2.0f, 0.0f)));
    std::cout << "  ✓ World center (body at origin): " << worldCenter1 << std::endl;

    // Body at (10, 20), no rotation
    Transform bodyTransform2(Vector2(10.0f, 20.0f), 0.0f);
    Vector2 worldCenter2 = circle.getWorldCenter(bodyTransform2);
    assert(approxEqual(worldCenter2, Vector2(12.0f, 20.0f)));
    std::cout << "  ✓ World center (body at (10,20)): " << worldCenter2 << std::endl;

    // Body at origin, 90° rotation
    Transform bodyTransform3(Vector2::zero(), M_PI / 2.0f);
    Vector2 worldCenter3 = circle.getWorldCenter(bodyTransform3);
    // (2, 0) rotated 90° becomes (0, 2)
    assert(approxEqual(worldCenter3, Vector2(0.0f, 2.0f)));
    std::cout << "  ✓ World center (90° rotation): " << worldCenter3 << std::endl;

    // Body at (5, 5), 90° rotation
    Transform bodyTransform4(Vector2(5.0f, 5.0f), M_PI / 2.0f);
    Vector2 worldCenter4 = circle.getWorldCenter(bodyTransform4);
    // (2, 0) rotated 90° = (0, 2), then + (5, 5) = (5, 7)
    assert(approxEqual(worldCenter4, Vector2(5.0f, 7.0f)));
    std::cout << "  ✓ World center (position + rotation): " << worldCenter4 << std::endl;

    std::cout << "✓ World center calculation passed!\n" << std::endl;
}

void testRadiusModification() {
    std::cout << "Testing radius modification..." << std::endl;

    CircleCollider circle(1.0f);

    // Initial radius
    assert(approxEqual(circle.getRadius(), 1.0f));

    // Set new radius
    circle.setRadius(3.0f);
    assert(approxEqual(circle.getRadius(), 3.0f));
    std::cout << "  ✓ Radius changed: " << circle.getRadius() << std::endl;

    // Verify mass updates
    float newMass = circle.calculateMass(1.0f);
    float expectedMass = M_PI * 9.0f;  // π × 3²
    assert(approxEqual(newMass, expectedMass));
    std::cout << "  ✓ Mass recalculated correctly: " << newMass << " kg" << std::endl;

    // Try invalid radius (should be ignored)
    circle.setRadius(-2.0f);
    assert(approxEqual(circle.getRadius(), 3.0f));  // Unchanged
    std::cout << "  ✓ Invalid radius rejected" << std::endl;

    circle.setRadius(0.0f);
    assert(approxEqual(circle.getRadius(), 3.0f));  // Unchanged
    std::cout << "  ✓ Zero radius rejected" << std::endl;

    std::cout << "✓ Radius modification passed!\n" << std::endl;
}

void testPolymorphism() {
    std::cout << "Testing polymorphism..." << std::endl;

    // Create circle through base class pointer
    Collider* collider = new CircleCollider(2.0f, Vector2(1.0f, 2.0f));

    // Verify type
    assert(collider->getType() == Collider::Type::Circle);
    std::cout << "  ✓ Type identification works" << std::endl;

    // Use base class interface
    float mass = collider->calculateMass(1.0f);
    float inertia = collider->calculateInertia(mass);
    Vector2 min, max;
    collider->getBounds(min, max);

    assert(mass > 0.0f);
    assert(inertia > 0.0f);
    assert(min.x < max.x && min.y < max.y);
    std::cout << "  ✓ Polymorphic calls work: mass=" << mass
              << ", inertia=" << inertia << std::endl;

    // Cast back to derived class
    CircleCollider* circle = static_cast<CircleCollider*>(collider);
    assert(approxEqual(circle->getRadius(), 2.0f));
    std::cout << "  ✓ Downcast works: radius=" << circle->getRadius() << std::endl;

    delete collider;
    std::cout << "  ✓ Polymorphic deletion works" << std::endl;

    std::cout << "✓ Polymorphism passed!\n" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   Collider & CircleCollider Test Suite" << std::endl;
    std::cout << "========================================\n" << std::endl;

    try {
        testCircleConstruction();
        testMassCalculation();
        testInertiaCalculation();
        testBoundsCalculation();
        testPointContainment();
        testOffsetFunctionality();
        testWorldCenterCalculation();
        testRadiusModification();
        testPolymorphism();

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