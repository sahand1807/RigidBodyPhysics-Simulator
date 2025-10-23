/**
 * @file CollisionDetection.cpp
 * @brief Implementation of collision detection algorithms
 */

#include "physics/CollisionDetection.hpp"
#include "physics/Collider.hpp"
#include "physics/CircleCollider.hpp"
#include <cmath>

namespace Physics {

// ========================================
// Circle-Circle Collision
// ========================================

bool CollisionDetection::circleVsCircle(RigidBody* bodyA, RigidBody* bodyB, Manifold& manifold) {
    // Validate inputs
    if (bodyA == nullptr || bodyB == nullptr) {
        return false;
    }

    // Check if both bodies have colliders
    if (!bodyA->hasCollider() || !bodyB->hasCollider()) {
        return false;
    }

    // Get colliders
    Collider* colliderA = bodyA->getCollider();
    Collider* colliderB = bodyB->getCollider();

    // Check if both are circle colliders
    if (colliderA->getType() != Collider::Type::Circle ||
        colliderB->getType() != Collider::Type::Circle) {
        return false;  // Not circle-circle collision
    }

    // Cast to CircleCollider
    CircleCollider* circleA = static_cast<CircleCollider*>(colliderA);
    CircleCollider* circleB = static_cast<CircleCollider*>(colliderB);

    // Get world positions and radii
    Vector2 posA = bodyA->getPosition();
    Vector2 posB = bodyB->getPosition();
    float radiusA = circleA->getRadius();
    float radiusB = circleB->getRadius();

    // Calculate vector from B to A
    Vector2 delta = posA - posB;
    float distanceSquared = delta.lengthSquared();

    // Calculate sum of radii
    float radiusSum = radiusA + radiusB;
    float radiusSumSquared = radiusSum * radiusSum;

    // Check for collision (using squared distance to avoid sqrt)
    if (distanceSquared >= radiusSumSquared) {
        return false;  // No collision
    }

    // Collision detected! Calculate collision data
    float distance = std::sqrt(distanceSquared);

    // Handle special case: circles at exact same position
    if (distance < 1e-6f) {
        // Circles are basically at the same position
        // Choose arbitrary separation direction
        manifold.normal = Vector2(1.0f, 0.0f);
        manifold.penetration = radiusSum;
    } else {
        // Normal points from B to A
        manifold.normal = delta / distance;  // Normalize
        manifold.penetration = radiusSum - distance;
    }

    // Calculate contact point (midpoint of penetration)
    // Start at B's surface, move halfway into penetration
    manifold.contactPoint = posB + manifold.normal * (radiusB - manifold.penetration * 0.5f);

    // Store body references
    manifold.bodyA = bodyA;
    manifold.bodyB = bodyB;

    return true;
}

bool CollisionDetection::detectCollision(RigidBody* bodyA, RigidBody* bodyB, Manifold& manifold) {
    // Validate inputs
    if (bodyA == nullptr || bodyB == nullptr) {
        return false;
    }

    // Check if both have colliders
    if (!bodyA->hasCollider() || !bodyB->hasCollider()) {
        return false;
    }

    // Get collider types
    Collider::Type typeA = bodyA->getCollider()->getType();
    Collider::Type typeB = bodyB->getCollider()->getType();

    // Dispatch to appropriate collision function
    if (typeA == Collider::Type::Circle && typeB == Collider::Type::Circle) {
        return circleVsCircle(bodyA, bodyB, manifold);
    }

    // Future: Add more collision type combinations
    // if (typeA == Circle && typeB == Box) return circleVsBox(bodyA, bodyB, manifold);
    // if (typeA == Box && typeB == Circle) { ... swap and call circleVsBox ... }
    // if (typeA == Box && typeB == Box) return boxVsBox(bodyA, bodyB, manifold);

    // Unsupported collision type
    return false;
}

} // namespace Physics
