/**
 * @file BoxCollider.hpp
 * @brief Rectangular collision shape
 *
 * Implements Collider interface for rectangular/box objects.
 * Boxes are oriented bounding boxes (OBB) that can rotate with the body.
 */

#ifndef BOXCOLLIDER_HPP
#define BOXCOLLIDER_HPP

#include "physics/Collider.hpp"
#include "math/Vector2.hpp"

namespace Physics {

/**
 * @class BoxCollider
 * @brief Rectangular collision shape (Oriented Bounding Box)
 *
 * A box is defined by:
 * - Center position (inherited offset from Collider)
 * - Width and height (dimensions)
 * - Rotation (from RigidBody's transform)
 *
 * Advantages of boxes:
 * - More realistic for many objects (crates, walls, platforms)
 * - Efficient for axis-aligned cases (AABB)
 * - Good for stacking and stable contact
 * - Natural for ground planes and walls
 *
 * Physical properties (2D):
 * - Area: A = width × height
 * - Mass: m = density × area = density × w × h
 * - Moment of inertia: I = (1/12) × m × (w² + h²)
 *
 * Use cases:
 * - Crates, boxes, platforms
 * - Ground planes, walls, obstacles
 * - Buildings, vehicles
 * - Dominoes, blocks
 *
 * Example usage:
 * @code
 * // Create a 2m × 1m box
 * BoxCollider* crate = new BoxCollider(2.0f, 1.0f);
 *
 * // Calculate mass for wood density
 * float woodDensity = 0.6f;  // kg/m²
 * float mass = crate->calculateMass(woodDensity);
 * float inertia = crate->calculateInertia(mass);
 *
 * // Attach to body
 * RigidBody body;
 * body.setMass(mass);
 * body.setMomentOfInertia(inertia);
 * body.setCollider(crate);
 * @endcode
 */
class BoxCollider : public Collider {
public:
    // ========================================
    // Constructors
    // ========================================

    /**
     * @brief Construct box with dimensions and optional offset
     * @param width Box width in meters (must be > 0)
     * @param height Box height in meters (must be > 0)
     * @param offset Local offset from RigidBody's center (default: origin)
     *
     * Example:
     * @code
     * // Square box at body's center
     * BoxCollider b1(1.0f, 1.0f);
     *
     * // Rectangular box offset upward
     * BoxCollider b2(2.0f, 0.5f, Vector2(0.0f, 1.0f));
     * @endcode
     */
    explicit BoxCollider(float width, float height, const Vector2& offset = Vector2::zero());

    /**
     * @brief Virtual destructor
     */
    virtual ~BoxCollider() = default;

    // ========================================
    // Collider Interface Implementation
    // ========================================

    /**
     * @brief Get shape type
     * @return Type::Box
     */
    Type getType() const override;

    /**
     * @brief Calculate mass from box area
     * @param density Material density in kg/m²
     * @return Mass in kilograms
     *
     * Formula: m = density × area = density × width × height
     *
     * Examples (for 2m × 1m box):
     * - Wood (0.6 kg/m²): m = 0.6 × 2 × 1 = 1.2 kg
     * - Metal (7.8 kg/m²): m = 7.8 × 2 × 1 = 15.6 kg
     * - Rubber (1.1 kg/m²): m = 1.1 × 2 × 1 = 2.2 kg
     */
    float calculateMass(float density) const override;

    /**
     * @brief Calculate moment of inertia
     * @param mass Mass in kilograms
     * @return Moment of inertia in kg⋅m²
     *
     * Formula: I = (1/12) × m × (w² + h²)
     *
     * Physical meaning:
     * - Larger dimensions → harder to rotate
     * - Square distributes mass evenly
     * - Long thin box easier to spin around long axis
     *
     * Examples:
     * - Square (m=1kg, w=h=1m): I = (1/12) × 1 × (1 + 1) = 0.167 kg⋅m²
     * - Rectangle (m=1kg, w=2m, h=0.5m): I = (1/12) × 1 × (4 + 0.25) = 0.354 kg⋅m²
     */
    float calculateInertia(float mass) const override;

    /**
     * @brief Get axis-aligned bounding box
     * @param min Output: Minimum corner
     * @param max Output: Maximum corner
     *
     * For an oriented box, AABB is calculated by finding the
     * extremes of the four rotated corners.
     *
     * Note: AABB is larger than the actual box when rotated.
     * - At 0°: AABB = exact box
     * - At 45°: AABB is √2 times larger
     *
     * Example:
     * @code
     * BoxCollider box(2.0f, 1.0f, Vector2(5, 10));
     * Vector2 min, max;
     * box.getBounds(min, max);
     * // For non-rotated box:
     * // min = (4, 9.5), max = (6, 10.5)
     * @endcode
     */
    void getBounds(Vector2& min, Vector2& max) const override;

    // ========================================
    // Box-Specific Methods
    // ========================================

    /**
     * @brief Get box width
     * @return Width in meters
     */
    float getWidth() const { return width; }

    /**
     * @brief Get box height
     * @return Height in meters
     */
    float getHeight() const { return height; }

    /**
     * @brief Set box width
     * @param width New width (must be > 0)
     *
     * Note: Changing dimensions doesn't automatically update RigidBody's
     * mass and inertia. You must recalculate and set them manually.
     */
    void setWidth(float width);

    /**
     * @brief Set box height
     * @param height New height (must be > 0)
     */
    void setHeight(float height);

    /**
     * @brief Set both dimensions at once
     * @param width New width (must be > 0)
     * @param height New height (must be > 0)
     */
    void setDimensions(float width, float height);

    /**
     * @brief Get half-width (half-extent in x)
     * @return width / 2
     *
     * Half-extents are commonly used in collision detection algorithms.
     */
    float getHalfWidth() const { return width * 0.5f; }

    /**
     * @brief Get half-height (half-extent in y)
     * @return height / 2
     */
    float getHalfHeight() const { return height * 0.5f; }

    /**
     * @brief Check if a point is inside the box (in local space)
     * @param point Point in local space (relative to offset, non-rotated)
     * @return True if point is inside or on the box
     *
     * For local space check (axis-aligned):
     * Formula: |point.x| ≤ width/2 && |point.y| ≤ height/2
     *
     * Example:
     * @code
     * BoxCollider box(2.0f, 1.0f);
     * bool inside = box.containsPoint(Vector2(0.5f, 0.3f));
     * // inside = true (point is within half-extents)
     * @endcode
     */
    bool containsPoint(const Vector2& point) const;

private:
    // ========================================
    // Private Members
    // ========================================

    float width;   ///< Box width in meters (x-axis)
    float height;  ///< Box height in meters (y-axis)
};

} // namespace Physics

#endif // BOXCOLLIDER_HPP
