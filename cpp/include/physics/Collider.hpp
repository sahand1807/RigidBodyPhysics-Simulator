/**
 * @file Collider.hpp
 * @brief Abstract base class for collision shapes
 *
 * Defines the interface that all collision shapes must implement.
 * Collision shapes define the physical bounds of a RigidBody for
 * collision detection and mass property calculations.
 */

#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "math/Vector2.hpp"

namespace Physics {

/**
 * @class Collider
 * @brief Abstract base class for all collision shapes
 *
 * A Collider defines:
 * 1. The shape of a RigidBody (circle, box, polygon, etc.)
 * 2. How to calculate mass and moment of inertia from the shape
 * 3. Bounding box for broad-phase collision detection
 *
 * Design Pattern: Abstract Base Class
 * - Derived classes implement pure virtual methods
 * - Allows polymorphism: RigidBody can have any collider type
 * - Extensible: Easy to add new shapes
 *
 * Separation of Concerns:
 * - RigidBody: Handles motion (velocity, forces, integration)
 * - Collider: Handles shape (geometry, mass properties)
 * - Together: Complete physical object
 *
 * Example usage:
 * @code
 * // Create a circle collider
 * CircleCollider* circle = new CircleCollider(1.0f);
 *
 * // Calculate mass properties
 * float mass = circle->calculateMass(1.0f);  // density = 1 kg/m²
 * float inertia = circle->calculateInertia(mass);
 *
 * // Attach to body
 * RigidBody body;
 * body.setMass(mass);
 * body.setMomentOfInertia(inertia);
 * body.attachCollider(circle);
 * @endcode
 */
class Collider {
public:
    // ========================================
    // Shape Type Enumeration
    // ========================================

    /**
     * @enum Type
     * @brief Identifies the type of collision shape
     *
     * Used for:
     * - Runtime type checking
     * - Collision detection dispatch (circle-circle, circle-box, etc.)
     * - Debugging and visualization
     */
    enum class Type {
        Circle,  ///< Circular shape
        Box      ///< Rectangular (axis-aligned or oriented) shape
    };

    // ========================================
    // Constructor and Destructor
    // ========================================

    /**
     * @brief Default constructor
     * @param offset Local offset from RigidBody's center (default: origin)
     */
    Collider(const Vector2& offset = Vector2::zero());

    /**
     * @brief Virtual destructor (required for polymorphism)
     *
     * Ensures derived class destructors are called when deleting through
     * base class pointer: delete (Collider*) circleCollider;
     */
    virtual ~Collider() = default;

    // ========================================
    // Pure Virtual Methods (Must Override)
    // ========================================

    /**
     * @brief Get the type of this collider
     * @return Shape type enum value
     *
     * Pure virtual: Derived classes must implement.
     *
     * Example:
     * @code
     * if (collider->getType() == Collider::Type::Circle) {
     *     CircleCollider* circle = static_cast<CircleCollider*>(collider);
     *     // Use circle-specific methods
     * }
     * @endcode
     */
    virtual Type getType() const = 0;

    /**
     * @brief Calculate mass from shape and density
     * @param density Material density in kg/m² (2D area density)
     * @return Mass in kilograms
     *
     * Pure virtual: Each shape has different mass formula.
     *
     * Formulas:
     * - Circle: m = density × π × r²
     * - Box: m = density × width × height
     *
     * Typical densities (2D approximations):
     * - Wood: 0.6 kg/m²
     * - Metal: 7.8 kg/m²
     * - Rubber: 1.1 kg/m²
     */
    virtual float calculateMass(float density) const = 0;

    /**
     * @brief Calculate moment of inertia from mass
     * @param mass Mass in kilograms
     * @return Moment of inertia in kg⋅m²
     *
     * Pure virtual: Each shape has different inertia formula.
     *
     * Moment of inertia (rotational mass) depends on mass distribution.
     * Mass far from center → harder to rotate → larger I
     *
     * Formulas:
     * - Circle: I = 0.5 × m × r²
     * - Box: I = (1/12) × m × (w² + h²)
     */
    virtual float calculateInertia(float mass) const = 0;

    /**
     * @brief Get axis-aligned bounding box (AABB)
     * @param min Output: Minimum corner (bottom-left)
     * @param max Output: Maximum corner (top-right)
     *
     * Pure virtual: Each shape computes bounds differently.
     *
     * AABB is used for:
     * - Broad-phase collision detection (quick rejection)
     * - Spatial partitioning (quadtree, grid)
     * - Frustum culling (rendering optimization)
     *
     * Example:
     * @code
     * Vector2 min, max;
     * collider->getBounds(min, max);
     *
     * // Check if point is potentially inside
     * if (point.x >= min.x && point.x <= max.x &&
     *     point.y >= min.y && point.y <= max.y) {
     *     // Detailed collision check needed
     * }
     * @endcode
     */
    virtual void getBounds(Vector2& min, Vector2& max) const = 0;

    // ========================================
    // Non-Virtual Methods (Common to All)
    // ========================================

    /**
     * @brief Get local offset from RigidBody's center
     * @return Offset vector
     *
     * Offset allows collider to be positioned away from body's center.
     * Useful for:
     * - Complex shapes made of multiple colliders
     * - Asymmetric objects
     * - Composite bodies
     *
     * Example:
     * @code
     * // Car body with offset wheels
     * RigidBody car;
     * CircleCollider* frontWheel = new CircleCollider(0.5f);
     * frontWheel->setOffset(Vector2(1.5f, -0.5f));  // Front-right
     * @endcode
     */
    const Vector2& getOffset() const { return offset; }

    /**
     * @brief Set local offset from RigidBody's center
     * @param offset New offset vector
     */
    void setOffset(const Vector2& offset);

    /**
     * @brief Get world-space center of this collider
     * @param bodyTransform The RigidBody's transform
     * @return World-space center position
     *
     * Transforms local offset to world space using body's transform.
     *
     * Formula: worldCenter = bodyTransform.transformPoint(offset)
     *
     * Example:
     * @code
     * RigidBody body;
     * body.setPosition(Vector2(10, 20));
     *
     * CircleCollider circle(1.0f);
     * circle.setOffset(Vector2(2, 0));  // 2 units to the right
     *
     * Vector2 worldCenter = circle.getWorldCenter(body.getTransform());
     * // Result: (12, 20) = body position + rotated offset
     * @endcode
     */
    Vector2 getWorldCenter(const class Transform& bodyTransform) const;

protected:
    // ========================================
    // Protected Members (Accessible to Derived Classes)
    // ========================================

    Vector2 offset;  ///< Local offset from RigidBody's center of mass
};

} // namespace Physics

#endif // COLLIDER_HPP