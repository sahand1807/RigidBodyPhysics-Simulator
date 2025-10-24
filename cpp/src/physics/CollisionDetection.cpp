/**
 * @file CollisionDetection.cpp
 * @brief Implementation of collision detection algorithms
 */

#include "physics/CollisionDetection.hpp"
#include "physics/Collider.hpp"
#include "physics/CircleCollider.hpp"
#include "physics/BoxCollider.hpp"
#include "math/Transform.hpp"
#include <cmath>
#include <algorithm>

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

// ========================================
// Circle-Box Collision
// ========================================

bool CollisionDetection::circleVsBox(RigidBody* bodyA, RigidBody* bodyB, Manifold& manifold) {
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

    // Determine which is circle and which is box
    CircleCollider* circle = nullptr;
    BoxCollider* box = nullptr;
    RigidBody* circleBody = nullptr;
    RigidBody* boxBody = nullptr;
    bool swapped = false;

    if (colliderA->getType() == Collider::Type::Circle &&
        colliderB->getType() == Collider::Type::Box) {
        circle = static_cast<CircleCollider*>(colliderA);
        box = static_cast<BoxCollider*>(colliderB);
        circleBody = bodyA;
        boxBody = bodyB;
        swapped = false;
    } else if (colliderA->getType() == Collider::Type::Box &&
               colliderB->getType() == Collider::Type::Circle) {
        circle = static_cast<CircleCollider*>(colliderB);
        box = static_cast<BoxCollider*>(colliderA);
        circleBody = bodyB;
        boxBody = bodyA;
        swapped = true;
    } else {
        return false;  // Not a circle-box collision
    }

    // Get world positions
    Vector2 circlePos = circleBody->getPosition();
    Vector2 boxPos = boxBody->getPosition();
    float boxRotation = boxBody->getRotation();

    // Transform circle center into box's local space
    Transform boxTransform(boxPos, boxRotation);
    Transform boxInverse = boxTransform.inverse();
    Vector2 circleLocalPos = boxInverse.transformPoint(circlePos);

    // Get box half-extents
    float halfWidth = box->getHalfWidth();
    float halfHeight = box->getHalfHeight();

    // Find closest point on box to circle center (in local space)
    Vector2 closest;
    closest.x = std::clamp(circleLocalPos.x, -halfWidth, halfWidth);
    closest.y = std::clamp(circleLocalPos.y, -halfHeight, halfHeight);

    // Calculate distance from closest point to circle center
    Vector2 localDelta = circleLocalPos - closest;
    float distanceSquared = localDelta.lengthSquared();
    float radius = circle->getRadius();
    float radiusSquared = radius * radius;

    // Check for collision
    if (distanceSquared >= radiusSquared) {
        return false;  // No collision
    }

    // Collision detected!
    float distance = std::sqrt(distanceSquared);

    // Calculate collision normal and penetration
    Vector2 localNormal;
    float penetration;

    if (distance < 1e-6f) {
        // Circle center is inside or on the box
        // Find the closest box edge and use that normal

        // Calculate distances to each edge
        float distLeft = halfWidth + circleLocalPos.x;
        float distRight = halfWidth - circleLocalPos.x;
        float distBottom = halfHeight + circleLocalPos.y;
        float distTop = halfHeight - circleLocalPos.y;

        // Find minimum distance (closest edge)
        float minDist = std::min({distLeft, distRight, distBottom, distTop});

        if (minDist == distLeft) {
            localNormal = Vector2(-1.0f, 0.0f);  // Left edge
            penetration = radius + distLeft;
        } else if (minDist == distRight) {
            localNormal = Vector2(1.0f, 0.0f);   // Right edge
            penetration = radius + distRight;
        } else if (minDist == distBottom) {
            localNormal = Vector2(0.0f, -1.0f);  // Bottom edge
            penetration = radius + distBottom;
        } else {
            localNormal = Vector2(0.0f, 1.0f);   // Top edge
            penetration = radius + distTop;
        }
    } else {
        // Circle is outside box
        localNormal = localDelta / distance;
        penetration = radius - distance;
    }

    // Transform normal back to world space
    Vector2 worldNormal = boxTransform.transformDirection(localNormal);

    // Calculate contact point in world space
    Vector2 worldClosest = boxTransform.transformPoint(closest);

    // If swapped (box was bodyA), flip the normal
    if (swapped) {
        worldNormal = -worldNormal;
    }

    // Fill manifold
    manifold.normal = worldNormal;
    manifold.penetration = penetration;
    manifold.contactPoint = worldClosest;
    manifold.bodyA = bodyA;  // Original order
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

    if ((typeA == Collider::Type::Circle && typeB == Collider::Type::Box) ||
        (typeA == Collider::Type::Box && typeB == Collider::Type::Circle)) {
        return circleVsBox(bodyA, bodyB, manifold);
    }

    // Future: Add more collision type combinations
    // if (typeA == Box && typeB == Box) return boxVsBox(bodyA, bodyB, manifold);

    // Unsupported collision type
    return false;
}

} // namespace Physics
