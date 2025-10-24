/**
 * @file DistanceConstraint.hpp
 * @brief Distance constraint - keeps two points at fixed distance
 *
 * A DistanceConstraint maintains a fixed distance between two points,
 * simulating ropes, strings, springs, or rigid rods.
 *
 * Supports two modes:
 * 1. Body-to-World: One body attached to a fixed world point (hanging string)
 * 2. Body-to-Body: Two bodies connected together (rope segment)
 *
 * Uses XPBD (Extended Position-Based Dynamics) solver for stability.
 */

#ifndef DISTANCECONSTRAINT_HPP
#define DISTANCECONSTRAINT_HPP

#include "physics/Constraint.hpp"
#include "physics/RigidBody.hpp"
#include "math/Vector2.hpp"

namespace Physics {

/**
 * @class DistanceConstraint
 * @brief Maintains fixed distance between two points
 *
 * Physics Applications:
 * - Ropes and strings (low compliance)
 * - Springs (high compliance)
 * - Rigid rods (zero compliance)
 * - Pendulums (body attached to world point)
 * - Chain links (body-to-body)
 *
 * XPBD Algorithm:
 * - Compliance parameter controls stiffness
 * - 0.0 = perfectly rigid (rope/rod)
 * - 0.001 = slightly stretchy (realistic rope)
 * - 0.1+ = spring-like behavior
 *
 * Example Usage:
 * @code
 * // Create a pendulum (ball hanging from ceiling)
 * DistanceConstraint* string = new DistanceConstraint(
 *     ball,                    // The swinging ball
 *     Vector2(0, 0),          // Attach at ball center
 *     Vector2(0, 10),         // Ceiling point
 *     5.0f,                   // String length: 5 meters
 *     0.0001f                 // Very rigid (low compliance)
 * );
 * world->addConstraint(string);
 * @endcode
 */
class DistanceConstraint : public Constraint {
public:
    // ========================================
    // Constructors
    // ========================================

    /**
     * @brief Create body-to-world distance constraint (hanging string)
     *
     * @param bodyA The body to constrain
     * @param localAnchorA Attachment point in bodyA's local space
     * @param worldPoint Fixed point in world space (e.g., ceiling)
     * @param length Target distance to maintain
     * @param compliance Material stiffness (0 = rigid, higher = stretchy)
     */
    DistanceConstraint(RigidBody* bodyA, const Vector2& localAnchorA,
                      const Vector2& worldPoint, float length,
                      float compliance = 0.0001f);

    /**
     * @brief Create body-to-body distance constraint (rope segment)
     *
     * @param bodyA First body
     * @param localAnchorA Attachment point in bodyA's local space
     * @param bodyB Second body
     * @param localAnchorB Attachment point in bodyB's local space
     * @param length Target distance to maintain
     * @param compliance Material stiffness (0 = rigid, higher = stretchy)
     */
    DistanceConstraint(RigidBody* bodyA, const Vector2& localAnchorA,
                      RigidBody* bodyB, const Vector2& localAnchorB,
                      float length, float compliance = 0.0001f);

    /**
     * @brief Destructor
     */
    ~DistanceConstraint() override = default;

    // ========================================
    // Constraint Interface Implementation
    // ========================================

    /**
     * @brief Solve constraint using XPBD algorithm
     *
     * Calculates the distance error and applies position corrections
     * to bodyA (and bodyB if present) to maintain target distance.
     *
     * @param dt Time step (seconds)
     */
    void solve(float dt) override;

    /**
     * @brief Check if constraint is valid
     *
     * @return True if bodyA exists and length is positive
     */
    bool isValid() const override;

    /**
     * @brief Check if constraint involves given body
     *
     * @param body Body to check
     * @return True if body is bodyA or bodyB
     */
    bool involves(const RigidBody* body) const override;

    // ========================================
    // Getters & Setters
    // ========================================

    /**
     * @brief Get first body
     * @return Pointer to bodyA
     */
    RigidBody* getBodyA() const { return bodyA; }

    /**
     * @brief Get second body (may be nullptr for world constraints)
     * @return Pointer to bodyB or nullptr
     */
    RigidBody* getBodyB() const { return bodyB; }

    /**
     * @brief Get target distance
     * @return Distance in meters
     */
    float getLength() const { return length; }

    /**
     * @brief Set target distance
     * @param length New distance in meters (must be positive)
     */
    void setLength(float length);

    /**
     * @brief Get compliance (stiffness parameter)
     * @return Compliance value (0 = rigid, higher = stretchy)
     */
    float getCompliance() const { return compliance; }

    /**
     * @brief Set compliance
     * @param compliance New compliance (must be >= 0)
     */
    void setCompliance(float compliance);

    /**
     * @brief Get current distance between anchor points
     * @return Current distance in meters
     */
    float getCurrentDistance() const;

private:
    // ========================================
    // Private Members
    // ========================================

    RigidBody* bodyA;           ///< First body (never null)
    RigidBody* bodyB;           ///< Second body (null for world constraints)

    Vector2 localAnchorA;       ///< Attachment point on bodyA (local space)
    Vector2 localAnchorB;       ///< Attachment point on bodyB (local space) or world point

    float length;               ///< Target distance to maintain
    float compliance;           ///< Material stiffness (0 = rigid, higher = stretchy)

    float lagrangian;           ///< Accumulated constraint force (for XPBD)

    // ========================================
    // Private Helper Methods
    // ========================================

    /**
     * @brief Get world position of anchor A
     * @return Anchor A in world coordinates
     */
    Vector2 getWorldAnchorA() const;

    /**
     * @brief Get world position of anchor B
     * @return Anchor B in world coordinates (world point if bodyB is null)
     */
    Vector2 getWorldAnchorB() const;
};

} // namespace Physics

#endif // DISTANCECONSTRAINT_HPP
