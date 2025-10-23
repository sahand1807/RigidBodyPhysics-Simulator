/**
 * @file Vector2.cpp
 * @brief Implementation of 2D Vector mathematics
 */

#include "math/Vector2.hpp"
#include <cmath>

namespace Physics {

// ========================================
// Constructors
// ========================================

Vector2::Vector2() : x(0.0f), y(0.0f) {}

Vector2::Vector2(float x, float y) : x(x), y(y) {}

// ========================================
// Basic Vector Operations
// ========================================

Vector2 Vector2::operator+(const Vector2& other) const {
    return Vector2(x + other.x, y + other.y);
}

Vector2 Vector2::operator-(const Vector2& other) const {
    return Vector2(x - other.x, y - other.y);
}

Vector2 Vector2::operator*(float scalar) const {
    return Vector2(x * scalar, y * scalar);
}

Vector2 Vector2::operator/(float scalar) const {
    // Note: We don't check for division by zero here for performance.
    // We might want to add later: if (scalar == 0.0f) return Vector2::zero();
    return Vector2(x / scalar, y / scalar);
}

Vector2 Vector2::operator-() const {
    return Vector2(-x, -y);
}

// ========================================
// Compound Assignment Operators
// ========================================

Vector2& Vector2::operator+=(const Vector2& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Vector2& Vector2::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

Vector2& Vector2::operator/=(float scalar) {
    x /= scalar;
    y /= scalar;
    return *this;
}

// ========================================
// Vector Mathematics
// ========================================

float Vector2::dot(const Vector2& other) const {
    return x * other.x + y * other.y;
}

float Vector2::cross(const Vector2& other) const {
    // In 2D, cross product returns the z-component of the 3D cross product
    // This tells us the "signed area" of the parallelogram formed by the vectors
    return x * other.y - y * other.x;
}

float Vector2::length() const {
    return std::sqrt(x * x + y * y);
}

float Vector2::lengthSquared() const {
    return x * x + y * y;
}

Vector2& Vector2::normalize() {
    float len = length();

    // Avoid division by zero
    if (len > 0.0f) {
        x /= len;
        y /= len;
    }

    return *this;
}

Vector2 Vector2::normalized() const {
    float len = length();

    // Avoid division by zero
    if (len > 0.0f) {
        return Vector2(x / len, y / len);
    }

    return Vector2::zero();
}

float Vector2::distance(const Vector2& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
}

float Vector2::distanceSquared(const Vector2& other) const {
    float dx = x - other.x;
    float dy = y - other.y;
    return dx * dx + dy * dy;
}

// ========================================
// Utility Functions
// ========================================

bool Vector2::isZero() const {
    // Using a small epsilon for floating-point comparison
    const float epsilon = 1e-6f;
    return std::abs(x) < epsilon && std::abs(y) < epsilon;
}

Vector2 Vector2::perpendicular() const {
    // Rotate 90 degrees counterclockwise: (x, y) -> (-y, x)
    return Vector2(-y, x);
}

// ========================================
// Static Helper Functions
// ========================================

Vector2 Vector2::zero() {
    return Vector2(0.0f, 0.0f);
}

Vector2 Vector2::right() {
    return Vector2(1.0f, 0.0f);
}

Vector2 Vector2::up() {
    return Vector2(0.0f, 1.0f);
}

// ========================================
// Non-member Operators
// ========================================

Vector2 operator*(float scalar, const Vector2& vec) {
    return vec * scalar;
}

std::ostream& operator<<(std::ostream& os, const Vector2& vec) {
    os << "Vector2(" << vec.x << ", " << vec.y << ")";
    return os;
}

} // namespace Physics