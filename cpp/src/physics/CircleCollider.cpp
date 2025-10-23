/**
 * @file CircleCollider.cpp
 * @brief Implementation of CircleCollider
 */

#include "physics/CircleCollider.hpp"
#include <cmath>
#include <algorithm>

namespace Physics {

// Mathematical constant
#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

// ========================================
// Constructor
// ========================================

CircleCollider::CircleCollider(float radius, const Vector2& offset)
    : Collider(offset)
    , radius(radius)
{
    // Validate radius
    if (radius <= 0.0f) {
        this->radius = 1.0f;  // Default to 1 meter if invalid
    }
}

// ========================================
// Collider Interface Implementation
// ========================================

Collider::Type CircleCollider::getType() const {
    return Type::Circle;
}

float CircleCollider::calculateMass(float density) const {
    // 2D circle area: A = π × r²
    // Mass: m = density × area
    float area = M_PI * radius * radius;
    return density * area;
}

float CircleCollider::calculateInertia(float mass) const {
    // 2D circle moment of inertia: I = 0.5 × m × r²
    //
    // Derivation:
    // For a disk rotating about its center:
    // I = ∫∫ r² dm
    // In 2D with uniform density:
    // I = (1/2) × m × r²

    return 0.5f * mass * radius * radius;
}

void CircleCollider::getBounds(Vector2& min, Vector2& max) const {
    // Circle's AABB is a square centered at offset
    // Size: 2r × 2r

    Vector2 radiusVec(radius, radius);

    min = offset - radiusVec;  // Bottom-left corner
    max = offset + radiusVec;  // Top-right corner
}

// ========================================
// Circle-Specific Methods
// ========================================

void CircleCollider::setRadius(float radius) {
    // Validate radius
    if (radius <= 0.0f) {
        return;  // Ignore invalid radius
    }

    this->radius = radius;
}

bool CircleCollider::containsPoint(const Vector2& point) const {
    // Check if point is within radius of center (offset)
    // Formula: |point - center| ≤ radius
    //
    // Optimization: Use distanceSquared to avoid sqrt
    // |point - center|² ≤ radius²

    float distSq = offset.distanceSquared(point);
    float radiusSq = radius * radius;

    return distSq <= radiusSq;
}

} // namespace Physics