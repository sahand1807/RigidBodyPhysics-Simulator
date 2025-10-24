/**
 * @file BoxCollider.cpp
 * @brief Implementation of BoxCollider class
 */

#include "physics/BoxCollider.hpp"
#include "math/Transform.hpp"
#include <cmath>
#include <algorithm>

namespace Physics {

// ========================================
// Constructor
// ========================================

BoxCollider::BoxCollider(float width, float height, const Vector2& offset)
    : Collider(offset), width(width), height(height) {
    // Validate dimensions
    if (width <= 0.0f || height <= 0.0f) {
        // In production code, might throw exception or log error
        // For now, clamp to minimum valid value
        this->width = std::max(width, 0.01f);
        this->height = std::max(height, 0.01f);
    }
}

// ========================================
// Collider Interface Implementation
// ========================================

Collider::Type BoxCollider::getType() const {
    return Type::Box;
}

float BoxCollider::calculateMass(float density) const {
    // Mass = density × area
    // Area of rectangle = width × height
    float area = width * height;
    return density * area;
}

float BoxCollider::calculateInertia(float mass) const {
    // Moment of inertia for a rectangle about its center:
    // I = (1/12) × m × (w² + h²)
    //
    // Derivation:
    // For a uniform rectangular plate rotating about its center,
    // we integrate r² dm over the entire area.
    //
    // This formula assumes the box rotates about its center of mass.
    return (1.0f / 12.0f) * mass * (width * width + height * height);
}

void BoxCollider::getBounds(Vector2& min, Vector2& max) const {
    // For now, return axis-aligned bounds assuming no rotation
    // In actual use, the Transform will be applied by the caller
    // to get world-space rotated bounds

    float halfWidth = width * 0.5f;
    float halfHeight = height * 0.5f;

    // Bounds in local space (centered at offset)
    min = offset + Vector2(-halfWidth, -halfHeight);
    max = offset + Vector2(halfWidth, halfHeight);
}

// ========================================
// Box-Specific Methods
// ========================================

void BoxCollider::setWidth(float width) {
    if (width > 0.0f) {
        this->width = width;
    }
}

void BoxCollider::setHeight(float height) {
    if (height > 0.0f) {
        this->height = height;
    }
}

void BoxCollider::setDimensions(float width, float height) {
    if (width > 0.0f && height > 0.0f) {
        this->width = width;
        this->height = height;
    }
}

bool BoxCollider::containsPoint(const Vector2& point) const {
    // Check if point is inside the box in local space
    // (assuming box is axis-aligned in local coordinates)

    Vector2 localPoint = point - offset;

    float halfWidth = width * 0.5f;
    float halfHeight = height * 0.5f;

    return (std::abs(localPoint.x) <= halfWidth &&
            std::abs(localPoint.y) <= halfHeight);
}

} // namespace Physics
