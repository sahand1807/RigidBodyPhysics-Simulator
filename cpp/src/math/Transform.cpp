/**
 * @file Transform.cpp
 * @brief Implementation of 2D Transform class
 */

#include "math/Transform.hpp"
#include <cmath>

namespace Physics {

// Mathematical constants
#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

// ========================================
// Constructors
// ========================================

Transform::Transform() : position(0.0f, 0.0f), rotation(0.0f) {}

Transform::Transform(const Vector2& position, float rotation)
    : position(position), rotation(rotation) {}

Transform::Transform(const Vector2& position)
    : position(position), rotation(0.0f) {}

// ========================================
// Point and Direction Transformations
// ========================================

Vector2 Transform::transformPoint(const Vector2& localPoint) const {
    // Rotate the point, then add position
    return position + rotate(localPoint, rotation);
}

Vector2 Transform::transformDirection(const Vector2& localDirection) const {
    // Only rotate, don't translate (directions have no position)
    return rotate(localDirection, rotation);
}

Vector2 Transform::inverseTransformPoint(const Vector2& worldPoint) const {
    // Subtract position first, then rotate backwards
    return rotate(worldPoint - position, -rotation);
}

Vector2 Transform::inverseTransformDirection(const Vector2& worldDirection) const {
    // Only rotate backwards
    return rotate(worldDirection, -rotation);
}

// ========================================
// Transform Combination
// ========================================

Transform Transform::combine(const Transform& other) const {
    // Combine transformations: apply 'this' first, then 'other'
    Transform result;

    // Combined rotation is sum of rotations
    result.rotation = this->rotation + other.rotation;

    // Combined position: other.position + rotate(this.position, other.rotation)
    result.position = other.position + rotate(this->position, other.rotation);

    return result;
}

Transform Transform::inverse() const {
    Transform result;

    // Inverse rotation
    result.rotation = -rotation;

    // Inverse position: rotate(-position, -rotation)
    result.position = rotate(-position, -rotation);

    return result;
}

// ========================================
// Directional Vectors
// ========================================

Vector2 Transform::getForward() const {
    // Forward is the direction we're facing
    // At rotation = 0, forward = (1, 0) pointing right
    return Vector2(std::cos(rotation), std::sin(rotation));
}

Vector2 Transform::getRight() const {
    // Right is 90° clockwise from forward
    // In 2D with +y up, right is perpendicular in clockwise direction
    // Mathematically: rotate forward by -90° (or -π/2)
    // cos(θ - π/2) = sin(θ)
    // sin(θ - π/2) = -cos(θ)
    return Vector2(std::sin(rotation), -std::cos(rotation));
}

Vector2 Transform::getUp() const {
    // Up is 90° counterclockwise from forward
    // This is opposite of right
    // cos(θ + π/2) = -sin(θ)
    // sin(θ + π/2) = cos(θ)
    return Vector2(-std::sin(rotation), std::cos(rotation));
}

// ========================================
// Utility Functions
// ========================================

void Transform::normalizeRotation() {
    // Normalize angle to range [-π, π)
    // Note: π and -π are equivalent (both are 180°), but we map π to -π for consistency

    // Use fmod to get remainder when dividing by 2π
    // This gives us a value in range (-2π, 2π)
    rotation = std::fmod(rotation, 2.0f * M_PI);

    // Convert to range [-π, π)
    // Handle boundary case: π should map to -π
    const float epsilon = 1e-6f;

    if (rotation > M_PI - epsilon) {
        rotation -= 2.0f * M_PI;
    } else if (rotation <= -M_PI - epsilon) {
        rotation += 2.0f * M_PI;
    }
}

void Transform::rotate(float angle) {
    rotation += angle;
}

void Transform::translate(const Vector2& displacement) {
    position += displacement;
}

void Transform::lookAt(const Vector2& target) {
    // Calculate direction to target
    Vector2 direction = target - position;

    // Set rotation to point toward target
    // atan2(y, x) gives angle of vector (x, y)
    rotation = std::atan2(direction.y, direction.x);
}

// ========================================
// Static Helper Functions
// ========================================

Transform Transform::identity() {
    return Transform(Vector2::zero(), 0.0f);
}

Transform Transform::translation(const Vector2& position) {
    return Transform(position, 0.0f);
}

Transform Transform::rotationTransform(float rotation) {
    return Transform(Vector2::zero(), rotation);
}

// ========================================
// Helper Functions (Private)
// ========================================

Vector2 Transform::rotate(const Vector2& v, float angle) {
    // 2D rotation matrix:
    // [ cos(θ)  -sin(θ) ] [ x ]   [ x*cos(θ) - y*sin(θ) ]
    // [ sin(θ)   cos(θ) ] [ y ] = [ x*sin(θ) + y*cos(θ) ]

    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);

    return Vector2(
        v.x * cosAngle - v.y * sinAngle,  // Rotated x
        v.x * sinAngle + v.y * cosAngle   // Rotated y
    );
}

} // namespace Physics
