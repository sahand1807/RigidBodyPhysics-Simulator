/**
 * @file CollisionResponse.cpp
 * @brief Implementation of collision resolution
 */

#include "physics/CollisionResponse.hpp"
#include "physics/RigidBody.hpp"
#include <algorithm>

namespace Physics {

// ========================================
// Collision Resolution
// ========================================

void CollisionResponse::resolveCollision(Manifold& manifold) {
    // Validate manifold
    if (!manifold.isValid()) {
        return;
    }

    // Step 1: Separate bodies (positional correction)
    positionalCorrection(manifold);

    // Step 2: Resolve velocities (apply bounce impulse)
    resolveVelocity(manifold);
}

// ========================================
// Positional Correction
// ========================================

void CollisionResponse::positionalCorrection(Manifold& manifold) {
    RigidBody* bodyA = manifold.bodyA;
    RigidBody* bodyB = manifold.bodyB;

    // Calculate total inverse mass
    float totalInvMass = bodyA->getInverseMass() + bodyB->getInverseMass();

    // If both bodies are static, can't correct
    if (totalInvMass == 0.0f) {
        return;
    }

    // Calculate correction amount with slop
    // Only correct penetrations beyond the slop threshold
    float correctionDepth = std::max(manifold.penetration - PENETRATION_SLOP, 0.0f);
    float correctionMagnitude = correctionDepth * POSITIONAL_CORRECTION_PERCENT / totalInvMass;

    // Calculate correction vector
    Vector2 correction = manifold.normal * correctionMagnitude;

    // Apply correction proportional to inverse mass
    // Heavier objects move less
    bodyA->setPosition(bodyA->getPosition() + correction * bodyA->getInverseMass());
    bodyB->setPosition(bodyB->getPosition() - correction * bodyB->getInverseMass());
}

// ========================================
// Velocity Resolution
// ========================================

void CollisionResponse::resolveVelocity(Manifold& manifold) {
    RigidBody* bodyA = manifold.bodyA;
    RigidBody* bodyB = manifold.bodyB;

    // Calculate relative velocity
    Vector2 relativeVelocity = bodyA->getVelocity() - bodyB->getVelocity();

    // Calculate velocity along collision normal
    float velocityAlongNormal = relativeVelocity.dot(manifold.normal);

    // If bodies are separating, don't apply impulse
    // (velocityAlongNormal > 0 means moving apart)
    if (velocityAlongNormal > 0.0f) {
        return;
    }

    // Calculate restitution (bounciness)
    // Use minimum of both bodies' restitution for realistic behavior
    float restitution = std::min(bodyA->getRestitution(), bodyB->getRestitution());

    // Calculate impulse magnitude
    // Formula: j = -(1 + e) * vn / (1/mA + 1/mB)
    // Where:
    //   e = restitution
    //   vn = velocity along normal
    //   mA, mB = masses
    float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal;
    impulseMagnitude /= (bodyA->getInverseMass() + bodyB->getInverseMass());

    // Calculate impulse vector
    Vector2 impulse = manifold.normal * impulseMagnitude;

    // Apply impulse to both bodies
    // A gets positive impulse (pushed away from B)
    // B gets negative impulse (pushed away from A)
    bodyA->applyImpulse(impulse);
    bodyB->applyImpulse(-impulse);
}

} // namespace Physics
