/**
 * @file CollisionDetection.hpp
 * @brief Collision detection algorithms
 *
 * Provides functions to detect collisions between different shape types:
 * - Circle-Circle collision
 * - Circle-Box collision (future)
 * - Box-Box collision (future)
 *
 * Each detection function fills a Manifold with collision data if a
 * collision is detected.
 */

#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP

#include "physics/Manifold.hpp"
#include "physics/RigidBody.hpp"
#include "physics/CircleCollider.hpp"

namespace Physics {

/**
 * @class CollisionDetection
 * @brief Static collision detection algorithms
 *
 * Provides collision detection for various shape combinations.
 * All methods are static - no instance needed.
 *
 * General approach:
 * 1. Early-out checks (AABB, distance)
 * 2. Detailed collision test
 * 3. Calculate collision data (normal, penetration, contact point)
 * 4. Fill manifold
 *
 * Example usage:
 * @code
 * Manifold manifold;
 * if (CollisionDetection::circleVsCircle(body1, body2, manifold)) {
 *     // Collision detected!
 *     // manifold contains collision data
 *     resolveCollision(manifold);
 * }
 * @endcode
 */
class CollisionDetection {
public:
    // ========================================
    // Circle-Circle Collision
    // ========================================

    /**
     * @brief Detect collision between two circles
     * @param bodyA First body (must have CircleCollider)
     * @param bodyB Second body (must have CircleCollider)
     * @param manifold Output manifold (filled if collision detected)
     * @return True if collision detected, false otherwise
     *
     * Algorithm:
     * 1. Get positions and radii of both circles
     * 2. Calculate distance between centers
     * 3. Check if distance < r1 + r2 (overlap test)
     * 4. Calculate normal: direction from B to A
     * 5. Calculate penetration: (r1 + r2) - distance
     * 6. Calculate contact point: midpoint of overlap
     *
     * Mathematical details:
     * - Distance: d = |posA - posB|
     * - Collision: d < r1 + r2
     * - Normal: n = (posA - posB) / d  (unit vector from B to A)
     * - Penetration: p = (r1 + r2) - d
     * - Contact: c = posB + n × (r2 - p/2)
     *
     * Edge cases:
     * - Null bodies: Returns false
     * - Null colliders: Returns false
     * - Non-circle colliders: Returns false (for now)
     * - Exact same position: Returns false (undefined normal)
     *
     * Example:
     * @code
     * // Circle at (0, 0) with radius 1
     * RigidBody* ball1 = ...;
     * // Circle at (1.5, 0) with radius 1
     * RigidBody* ball2 = ...;
     *
     * Manifold m;
     * if (CollisionDetection::circleVsCircle(ball1, ball2, m)) {
     *     // Collision! (distance 1.5 < sum of radii 2.0)
     *     // m.normal ≈ (1, 0)  (from ball2 to ball1)
     *     // m.penetration ≈ 0.5
     *     // m.contactPoint ≈ (0.75, 0)
     * }
     * @endcode
     */
    static bool circleVsCircle(RigidBody* bodyA, RigidBody* bodyB, Manifold& manifold);

    // Future: More collision types
    // static bool circleVsBox(RigidBody* circle, RigidBody* box, Manifold& manifold);
    // static bool boxVsBox(RigidBody* boxA, RigidBody* boxB, Manifold& manifold);

    // ========================================
    // General Collision Detection
    // ========================================

    /**
     * @brief Detect collision between any two bodies
     * @param bodyA First body
     * @param bodyB Second body
     * @param manifold Output manifold
     * @return True if collision detected
     *
     * Automatically dispatches to correct collision function based on
     * collider types. Currently only supports circle-circle.
     *
     * Future: Will handle all combinations:
     * - Circle vs Circle ✓
     * - Circle vs Box (future)
     * - Box vs Box (future)
     */
    static bool detectCollision(RigidBody* bodyA, RigidBody* bodyB, Manifold& manifold);

private:
    // No instances needed - all static methods
    CollisionDetection() = delete;
};

} // namespace Physics

#endif // COLLISIONDETECTION_HPP
