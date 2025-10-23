/**
 * @file CircleCollider.hpp
 * @brief Circular collision shape
 *
 * Implements Collider interface for circular objects.
 * Circles are the simplest collision shape with:
 * - Fastest collision detection (distance check only)
 * - Symmetric in all directions
 * - No rotation needed for collision
 */

#ifndef CIRCLECOLLIDER_HPP
#define CIRCLECOLLIDER_HPP

#include "physics/Collider.hpp"
#include "math/Vector2.hpp"

namespace Physics {

/**
 * @class CircleCollider
 * @brief Circular collision shape
 *
 * A circle is defined by:
 * - Center position (inherited offset from Collider)
 * - Radius
 *
 * Advantages of circles:
 * - Simple collision detection (just compare distances)
 * - No rotation needed (circles look the same from any angle)
 * - Fast to compute
 * - Naturally roll and bounce
 *
 * Physical properties (2D):
 * - Area: A = π × r²
 * - Mass: m = density × area = density × π × r²
 * - Moment of inertia: I = 0.5 × m × r²
 *
 * Use cases:
 * - Balls, wheels, coins
 * - Character capsules (circle + circle)
 * - Projectiles
 * - Simple approximations of complex shapes
 *
 * Example usage:
 * @code
 * // Create a 1-meter radius ball
 * CircleCollider* ball = new CircleCollider(1.0f);
 *
 * // Calculate mass for wood density
 * float woodDensity = 0.6f;  // kg/m²
 * float mass = ball->calculateMass(woodDensity);
 * float inertia = ball->calculateInertia(mass);
 *
 * // Attach to body
 * RigidBody body;
 * body.setMass(mass);
 * body.setMomentOfInertia(inertia);
 * body.attachCollider(ball);
 * @endcode
 */
class CircleCollider : public Collider {
public:
    // ========================================
    // Constructors
    // ========================================

    /**
     * @brief Construct circle with radius and optional offset
     * @param radius Circle radius in meters (must be > 0)
     * @param offset Local offset from RigidBody's center (default: origin)
     *
     * Example:
     * @code
     * // Circle at body's center
     * CircleCollider c1(1.0f);
     *
     * // Circle offset 2 units to the right
     * CircleCollider c2(0.5f, Vector2(2.0f, 0.0f));
     * @endcode
     */
    explicit CircleCollider(float radius, const Vector2& offset = Vector2::zero());

    /**
     * @brief Virtual destructor
     */
    virtual ~CircleCollider() = default;

    // ========================================
    // Collider Interface Implementation
    // ========================================

    /**
     * @brief Get shape type
     * @return Type::Circle
     */
    Type getType() const override;

    /**
     * @brief Calculate mass from circle area
     * @param density Material density in kg/m²
     * @return Mass in kilograms
     *
     * Formula: m = density × area = density × π × r²
     *
     * Examples (for radius = 1m):
     * - Wood (0.6 kg/m²): m = 0.6 × π × 1² ≈ 1.88 kg
     * - Metal (7.8 kg/m²): m = 7.8 × π × 1² ≈ 24.5 kg
     * - Rubber (1.1 kg/m²): m = 1.1 × π × 1² ≈ 3.46 kg
     */
    float calculateMass(float density) const override;

    /**
     * @brief Calculate moment of inertia
     * @param mass Mass in kilograms
     * @return Moment of inertia in kg⋅m²
     *
     * Formula: I = 0.5 × m × r²
     *
     *
     * Examples:
     * - Small ball (m=1kg, r=0.5m): I = 0.5 × 1 × 0.25 = 0.125 kg⋅m²
     * - Large ball (m=1kg, r=1.0m): I = 0.5 × 1 × 1.0 = 0.5 kg⋅m²
     *   → 4× harder to spin!
     */
    float calculateInertia(float mass) const override;

    /**
     * @brief Get axis-aligned bounding box
     * @param min Output: Minimum corner (center - radius)
     * @param max Output: Maximum corner (center + radius)
     *
     * For a circle, AABB is a square:
     * - Width = 2 × radius
     * - Height = 2 × radius
     *
     * Example:
     * @code
     * CircleCollider circle(1.0f, Vector2(5, 10));
     * Vector2 min, max;
     * circle.getBounds(min, max);
     * // min = (4, 9), max = (6, 11)
     * @endcode
     */
    void getBounds(Vector2& min, Vector2& max) const override;

    // ========================================
    // Circle-Specific Methods
    // ========================================

    /**
     * @brief Get circle radius
     * @return Radius in meters
     */
    float getRadius() const { return radius; }

    /**
     * @brief Set circle radius
     * @param radius New radius (must be > 0)
     *
     * Note: Changing radius doesn't automatically update RigidBody's
     * mass and inertia. You must recalculate and set them manually.
     *
     * Example:
     * @code
     * circle->setRadius(2.0f);
     * body.setMass(circle->calculateMass(density));
     * body.setMomentOfInertia(circle->calculateInertia(body.getMass()));
     * @endcode
     */
    void setRadius(float radius);

    /**
     * @brief Check if a point is inside the circle
     * @param point Point in local space (relative to offset)
     * @return True if point is inside or on the circle
     *
     * Formula: |point - center| ≤ radius
     *
     * Example:
     * @code
     * CircleCollider circle(1.0f);
     * bool inside = circle.containsPoint(Vector2(0.5f, 0.5f));
     * // inside = true (point is within radius 1)
     * @endcode
     */
    bool containsPoint(const Vector2& point) const;

private:
    // ========================================
    // Private Members
    // ========================================

    float radius;  ///< Circle radius in meters
};

} // namespace Physics

#endif // CIRCLECOLLIDER_HPP