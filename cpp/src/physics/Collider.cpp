/**
 * @file Collider.cpp
 * @brief Implementation of Collider base class
 */

#include "physics/Collider.hpp"
#include "math/Transform.hpp"

namespace Physics {

// ========================================
// Constructor
// ========================================

Collider::Collider(const Vector2& offset)
    : offset(offset)
{
}

// ========================================
// Non-Virtual Methods
// ========================================

void Collider::setOffset(const Vector2& offset) {
    this->offset = offset;
}

Vector2 Collider::getWorldCenter(const Transform& bodyTransform) const {
    // Transform local offset to world space using body's transform
    return bodyTransform.transformPoint(offset);
}

} // namespace Physics