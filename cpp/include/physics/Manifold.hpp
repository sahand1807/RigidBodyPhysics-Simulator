/**
 * @file Manifold.hpp
 * @brief Collision contact information
 *
 * A Manifold stores information about a collision between two bodies:
 * - Which bodies are colliding
 * - Contact normal (direction to separate)
 * - Penetration depth (how much they overlap)
 * - Contact point (where they touched)
 *
 * This information is used by the collision resolver to apply
 * appropriate impulses to separate and bounce the bodies.
 */

#ifndef MANIFOLD_HPP
#define MANIFOLD_HPP

#include "math/Vector2.hpp"

namespace Physics {

// Forward declaration
class RigidBody;

/**
 * @struct Manifold
 * @brief Collision contact information between two bodies
 *
 * A Manifold contains all information needed to resolve a collision:
 *
 * **Bodies**: The two colliding objects (A and B)
 * **Normal**: Direction from B to A (unit vector)
 *   - Points from B toward A
 *   - Used to separate bodies
 *   - Used to calculate bounce direction
 *
 * **Penetration**: How much bodies overlap (scalar)
 *   - Always positive for actual collisions
 *   - Zero or negative = no collision
 *   - Used to push bodies apart
 *
 * **Contact Point**: Where collision occurred (world space)
 *   - Used for rotational impulses (future)
 *   - Used for visualization
 *
 * Example usage:
 * @code
 * Manifold collision;
 * if (detectCollision(bodyA, bodyB, collision)) {
 *     // collision.normal points from B to A
 *     // collision.penetration is overlap amount
 *     resolveCollision(collision);
 * }
 * @endcode
 *
 * Coordinate convention:
 * - normal points from bodyB toward bodyA
 * - To separate: move A along +normal, B along -normal
 */
struct Manifold {
    // ========================================
    // Member Variables
    // ========================================

    RigidBody* bodyA;        ///< First colliding body
    RigidBody* bodyB;        ///< Second colliding body
    Vector2 normal;          ///< Collision normal (from B to A, unit length)
    float penetration;       ///< Overlap depth (positive = collision)
    Vector2 contactPoint;    ///< Contact location in world space

    // ========================================
    // Constructors
    // ========================================

    /**
     * @brief Default constructor
     *
     * Creates empty manifold with null bodies and zero values.
     */
    Manifold()
        : bodyA(nullptr)
        , bodyB(nullptr)
        , normal(Vector2::zero())
        , penetration(0.0f)
        , contactPoint(Vector2::zero())
    {
    }

    /**
     * @brief Construct manifold with collision data
     * @param bodyA First body
     * @param bodyB Second body
     * @param normal Collision normal (from B to A)
     * @param penetration Overlap depth
     * @param contactPoint Contact location
     */
    Manifold(RigidBody* bodyA, RigidBody* bodyB, const Vector2& normal,
             float penetration, const Vector2& contactPoint)
        : bodyA(bodyA)
        , bodyB(bodyB)
        , normal(normal)
        , penetration(penetration)
        , contactPoint(contactPoint)
    {
    }

    // ========================================
    // Utility Methods
    // ========================================

    /**
     * @brief Check if manifold represents a valid collision
     * @return True if both bodies exist and penetration is positive
     */
    bool isValid() const {
        return bodyA != nullptr &&
               bodyB != nullptr &&
               penetration > 0.0f;
    }

    /**
     * @brief Clear manifold data
     */
    void clear() {
        bodyA = nullptr;
        bodyB = nullptr;
        normal = Vector2::zero();
        penetration = 0.0f;
        contactPoint = Vector2::zero();
    }
};

} // namespace Physics

#endif // MANIFOLD_HPP
