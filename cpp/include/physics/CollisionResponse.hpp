/**
 * @file CollisionResponse.hpp
 * @brief Collision resolution using impulse-based physics
 *
 * Resolves collisions by applying impulses to separate bodies and
 * simulate bouncing. Uses conservation of momentum and energy.
 */

#ifndef COLLISIONRESPONSE_HPP
#define COLLISIONRESPONSE_HPP

#include "physics/Manifold.hpp"

namespace Physics {

/**
 * @class CollisionResponse
 * @brief Static collision resolution methods
 *
 * Resolves collisions using impulse-based physics:
 * 1. Positional correction: Separate penetrating bodies
 * 2. Velocity resolution: Apply impulses to bounce bodies apart
 *
 * The impulse calculation considers:
 * - Relative velocity at contact point
 * - Masses of both bodies (heavier = less affected)
 * - Restitution (bounciness)
 * - Friction (tangential resistance)
 *
 * All methods are static - no instance needed.
 */
class CollisionResponse {
public:
    // ========================================
    // Collision Resolution
    // ========================================

    /**
     * @brief Resolve a collision (positional + velocity)
     * @param manifold Collision information
     *
     * Complete collision resolution in two steps:
     * 1. Positional correction: Move bodies apart
     * 2. Velocity resolution: Apply bounce impulse
     *
     * Example:
     * @code
     * Manifold m;
     * if (CollisionDetection::detectCollision(bodyA, bodyB, m)) {
     *     CollisionResponse::resolveCollision(m);
     * }
     * @endcode
     */
    static void resolveCollision(Manifold& manifold);

    /**
     * @brief Correct positions to separate penetrating bodies
     * @param manifold Collision information
     *
     * Moves bodies apart along the collision normal to remove overlap.
     * Movement is proportional to inverse mass (heavier moves less).
     *
     * Algorithm:
     * - Calculate total inverse mass: sum = 1/mA + 1/mB
     * - Move A: +normal * penetration * (1/mA) / sum
     * - Move B: -normal * penetration * (1/mB) / sum
     *
     * Static bodies (invMass = 0) don't move.
     *
     * Note: This is a simplified version. More advanced engines use
     * iterative constraint solving for better stability.
     */
    static void positionalCorrection(Manifold& manifold);

    /**
     * @brief Apply impulse to resolve velocities
     * @param manifold Collision information
     *
     * Calculates and applies impulse to make bodies bounce apart.
     * Uses restitution and mass to determine bounce strength.
     *
     * Algorithm:
     * 1. Calculate relative velocity: vrel = vA - vB
     * 2. Calculate velocity along normal: vn = vrel · n
     * 3. If separating (vn >= 0), no impulse needed
     * 4. Calculate impulse magnitude: j = -(1 + e) * vn / (1/mA + 1/mB)
     * 5. Apply impulse: vA += j*n/mA, vB -= j*n/mB
     *
     * Where:
     * - e = restitution (average of both bodies)
     * - n = collision normal
     * - vn = velocity along normal
     * - j = impulse magnitude
     *
     * Physical meaning:
     * - j > 0: Bodies colliding, apply bounce
     * - j = 0: Bodies sliding along surface
     * - j < 0: Bodies separating (no impulse)
     *
     * Restitution effects:
     * - e = 0: Perfectly inelastic (no bounce)
     * - e = 0.5: Moderate bounce
     * - e = 1: Perfectly elastic (full bounce)
     */
    static void resolveVelocity(Manifold& manifold);

private:
    // No instances needed - all static methods
    CollisionResponse() = delete;

    // ========================================
    // Helper Constants
    // ========================================

    /// Percentage of penetration to correct (0-1)
    /// Lower = softer, more jitter; Higher = harder, less penetration
    static constexpr float POSITIONAL_CORRECTION_PERCENT = 0.8f;

    /// Minimum penetration before applying correction (meters)
    /// Prevents jitter from tiny overlaps due to floating point errors
    static constexpr float PENETRATION_SLOP = 0.01f;
};

} // namespace Physics

#endif // COLLISIONRESPONSE_HPP
