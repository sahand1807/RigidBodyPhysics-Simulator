/**
 * @file DistanceConstraint.cpp
 * @brief Implementation of distance constraint
 */

#include "physics/DistanceConstraint.hpp"
#include <cmath>
#include <algorithm>

namespace Physics {

// ========================================
// Constructors
// ========================================

DistanceConstraint::DistanceConstraint(RigidBody* bodyA, const Vector2& localAnchorA,
                                       const Vector2& worldPoint, float length,
                                       float compliance)
    : bodyA(bodyA)
    , bodyB(nullptr)  // null means world constraint
    , localAnchorA(localAnchorA)
    , localAnchorB(worldPoint)  // Store world point directly
    , length(length)
    , compliance(compliance)
    , lagrangian(0.0f)
{
}

DistanceConstraint::DistanceConstraint(RigidBody* bodyA, const Vector2& localAnchorA,
                                       RigidBody* bodyB, const Vector2& localAnchorB,
                                       float length, float compliance)
    : bodyA(bodyA)
    , bodyB(bodyB)
    , localAnchorA(localAnchorA)
    , localAnchorB(localAnchorB)
    , length(length)
    , compliance(compliance)
    , lagrangian(0.0f)
{
}

// ========================================
// Constraint Interface Implementation
// ========================================

void DistanceConstraint::solve(float dt) {
    // Skip if disabled or invalid
    if (!enabled || !isValid()) {
        return;
    }

    // Get world positions of anchors
    Vector2 posA = getWorldAnchorA();
    Vector2 posB = getWorldAnchorB();

    // Calculate current distance and direction
    Vector2 delta = posA - posB;
    float currentDist = delta.length();

    // Handle degenerate case (anchors at same position)
    if (currentDist < 1e-6f) {
        return;  // Can't determine direction
    }

    // Calculate constraint error (positive = stretched, negative = compressed)
    float error = currentDist - length;

    // Calculate gradient (normalized direction vector)
    Vector2 gradient = delta / currentDist;

    // Get inverse masses
    float invMassA = bodyA->getInverseMass();
    float invMassB = bodyB ? bodyB->getInverseMass() : 0.0f;

    // Skip if both bodies are static
    float totalInvMass = invMassA + invMassB;
    if (totalInvMass < 1e-6f) {
        return;
    }

    // XPBD: Calculate compliance term
    // alpha = compliance / (dt^2)
    // This makes the constraint frame-rate independent
    float alpha = compliance / (dt * dt);

    // Calculate Lagrange multiplier delta
    // delta_lambda = -error / (totalInvMass + alpha)
    float deltaLagrangian = -error / (totalInvMass + alpha);

    // Accumulate for warm starting (future optimization)
    lagrangian += deltaLagrangian;

    // Calculate position correction
    Vector2 correction = gradient * deltaLagrangian;

    // Store old positions for velocity update
    Vector2 oldPosA = bodyA->getPosition();

    // Apply position correction to bodyA
    Vector2 newPosA = oldPosA + correction * invMassA;
    bodyA->setPosition(newPosA);

    // Update velocity to match position change (CRITICAL for energy conservation!)
    Vector2 deltaVelA = (newPosA - oldPosA) / dt;
    bodyA->setVelocity(bodyA->getVelocity() + deltaVelA);

    // Apply correction to bodyB (if it exists)
    if (bodyB) {
        Vector2 oldPosB = bodyB->getPosition();
        Vector2 newPosB = oldPosB - correction * invMassB;
        bodyB->setPosition(newPosB);

        // Update velocity to match position change
        Vector2 deltaVelB = (newPosB - oldPosB) / dt;
        bodyB->setVelocity(bodyB->getVelocity() + deltaVelB);
    }
}

bool DistanceConstraint::isValid() const {
    // Must have bodyA and positive length
    return bodyA != nullptr && length > 0.0f;
}

bool DistanceConstraint::involves(const RigidBody* body) const {
    return body == bodyA || body == bodyB;
}

// ========================================
// Getters & Setters
// ========================================

void DistanceConstraint::setLength(float length) {
    if (length > 0.0f) {
        this->length = length;
    }
}

void DistanceConstraint::setCompliance(float compliance) {
    if (compliance >= 0.0f) {
        this->compliance = compliance;
    }
}

float DistanceConstraint::getCurrentDistance() const {
    Vector2 posA = getWorldAnchorA();
    Vector2 posB = getWorldAnchorB();
    return (posA - posB).length();
}

// ========================================
// Private Helper Methods
// ========================================

Vector2 DistanceConstraint::getWorldAnchorA() const {
    // Transform local anchor to world space
    // For now, anchors are relative to body center
    // In future, could use full transform with rotation
    return bodyA->getPosition() + localAnchorA;
}

Vector2 DistanceConstraint::getWorldAnchorB() const {
    if (bodyB) {
        // Body-to-body: transform local anchor
        return bodyB->getPosition() + localAnchorB;
    } else {
        // Body-to-world: localAnchorB stores the world point directly
        return localAnchorB;
    }
}

} // namespace Physics
