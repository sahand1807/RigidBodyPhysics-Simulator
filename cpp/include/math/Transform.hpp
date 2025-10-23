/**
 * @file Transform.hpp
 * @brief 2D Transform for position and rotation
 *
 * This file defines a Transform class representing an object's position
 * and orientation in 2D space. Essential for rigid body physics.
 */

#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "math/Vector2.hpp"
#include <cmath>

namespace Physics {

/**
 * @class Transform
 * @brief Represents position and rotation in 2D space
 *
 * A Transform defines where an object is (position) and how it's oriented (rotation).
 * It can transform points and directions between local and world coordinate systems.
 *
 * Every rigid body has a transform that describes its placement in the world.
 *
 * Coordinate System:
 * - Position: (x, y) in world space
 * - Rotation: angle in radians, counterclockwise from right (standard math convention)
 *   - 0 radians = pointing right (1, 0)
 *   - π/2 radians = pointing up (0, 1)
 *   - π radians = pointing left (-1, 0)
 *   - 3π/2 radians = pointing down (0, -1)
 *
 * Example usage:
 * @code
 * Transform bodyTransform(Vector2(5.0f, 3.0f), 0.785f);  // Position (5,3), 45° rotation
 * Vector2 localPoint(1.0f, 0.0f);  // Point in body's local space
 * Vector2 worldPoint = bodyTransform.transformPoint(localPoint);  // Convert to world
 * @endcode
 */
class Transform {
public:
    // ========================================
    // Members
    // ========================================

    Vector2 position;  ///< Position in world space
    float rotation;    ///< Rotation angle in radians (counterclockwise from right)

    // ========================================
    // Constructors
    // ========================================

    /**
     * @brief Default constructor - creates identity transform
     *
     * Identity transform: position at origin (0, 0), no rotation (0 radians)
     */
    Transform();

    /**
     * @brief Construct transform with position and rotation
     * @param position The position in world space
     * @param rotation The rotation angle in radians
     */
    Transform(const Vector2& position, float rotation);

    /**
     * @brief Construct transform with only position (no rotation)
     * @param position The position in world space
     */
    explicit Transform(const Vector2& position);

    // ========================================
    // Point and Direction Transformations
    // ========================================

    /**
     * @brief Transform a point from local space to world space
     * @param localPoint Point in local coordinates
     * @return Point in world coordinates
     *
     * Formula: worldPoint = position + rotate(localPoint, rotation)
     *
     * This applies both rotation and translation.
     *
     * Example:
     * @code
     * Transform t(Vector2(10, 20), 0);  // At (10,20), no rotation
     * Vector2 local(1, 0);
     * Vector2 world = t.transformPoint(local);  // Result: (11, 20)
     * @endcode
     *
     * Used for: Converting collision shape vertices to world space
     */
    Vector2 transformPoint(const Vector2& localPoint) const;

    /**
     * @brief Transform a direction from local space to world space
     * @param localDirection Direction vector in local coordinates
     * @return Direction vector in world coordinates
     *
     * Formula: worldDirection = rotate(localDirection, rotation)
     *
     * This applies only rotation, NOT translation (directions have no position).
     *
     * Example:
     * @code
     * Transform t(Vector2(10, 20), PI/2);  // At (10,20), rotated 90°
     * Vector2 localDir(1, 0);  // Pointing "forward" in local space
     * Vector2 worldDir = t.transformDirection(localDir);  // Result: (0, 1) pointing up
     * @endcode
     *
     * Used for: Converting velocity directions, surface normals
     */
    Vector2 transformDirection(const Vector2& localDirection) const;

    /**
     * @brief Transform a point from world space to local space
     * @param worldPoint Point in world coordinates
     * @return Point in local coordinates
     *
     * Formula: localPoint = rotate(worldPoint - position, -rotation)
     *
     * Inverse of transformPoint(). Useful for collision detection in local space.
     *
     * Example:
     * @code
     * Transform t(Vector2(10, 20), 0);
     * Vector2 world(11, 20);
     * Vector2 local = t.inverseTransformPoint(world);  // Result: (1, 0)
     * @endcode
     *
     * Used for: Converting world points to check against local collision shapes
     */
    Vector2 inverseTransformPoint(const Vector2& worldPoint) const;

    /**
     * @brief Transform a direction from world space to local space
     * @param worldDirection Direction vector in world coordinates
     * @return Direction vector in local coordinates
     *
     * Formula: localDirection = rotate(worldDirection, -rotation)
     *
     * Inverse of transformDirection().
     */
    Vector2 inverseTransformDirection(const Vector2& worldDirection) const;

    // ========================================
    // Transform Combination
    // ========================================

    /**
     * @brief Combine two transforms (this followed by other)
     * @param other The second transform to apply
     * @return Combined transform
     *
     * Combines transformations: first apply 'this', then apply 'other'.
     *
     * Formula:
     * - Combined position = other.position + rotate(this.position, other.rotation)
     * - Combined rotation = this.rotation + other.rotation
     *
     * Used for: Parent-child transform hierarchies
     *
     * Example:
     * @code
     * Transform parent(Vector2(10, 0), PI/2);  // At (10,0), rotated 90°
     * Transform childLocal(Vector2(5, 0), 0);   // 5 units to the right of parent
     * Transform childWorld = childLocal.combine(parent);
     * // Result: child is at (10, 5) in world space
     * @endcode
     */
    Transform combine(const Transform& other) const;

    /**
     * @brief Get the inverse of this transform
     * @return Inverse transform
     *
     * The inverse transform undoes this transform.
     * If: worldPoint = transform.transformPoint(localPoint)
     * Then: localPoint = inverse.transformPoint(worldPoint)
     *
     * Formula:
     * - Inverse position = rotate(-position, -rotation)
     * - Inverse rotation = -rotation
     */
    Transform inverse() const;

    // ========================================
    // Directional Vectors
    // ========================================

    /**
     * @brief Get the forward direction vector (local +x axis in world space)
     * @return Unit vector pointing in the direction of rotation
     *
     * Forward is the direction the transform is "facing".
     * At rotation = 0, forward = (1, 0) (pointing right)
     * At rotation = π/2, forward = (0, 1) (pointing up)
     *
     * Formula: (cos(rotation), sin(rotation))
     *
     * Used for: Applying forces in facing direction, movement
     */
    Vector2 getForward() const;

    /**
     * @brief Get the right direction vector (local +y axis in world space)
     * @return Unit vector pointing 90° clockwise from forward
     *
     * Right is perpendicular to forward (90° clockwise).
     * At rotation = 0, right = (0, -1) (pointing down)
     * At rotation = π/2, right = (1, 0) (pointing right)
     *
     * Formula: (sin(rotation), -cos(rotation))
     *
     * Note: In our coordinate system, +y is up, so "right" points in -y direction
     * when rotation is 0.
     *
     * Used for: Sideways movement, perpendicular forces
     */
    Vector2 getRight() const;

    /**
     * @brief Get the up direction vector (perpendicular to forward, in +y direction)
     * @return Unit vector pointing 90° counterclockwise from forward
     *
     * Up is perpendicular to forward (90° counterclockwise).
     * This is the opposite of getRight().
     *
     * Formula: (-sin(rotation), cos(rotation))
     */
    Vector2 getUp() const;

    // ========================================
    // Utility Functions
    // ========================================

    /**
     * @brief Normalize the rotation angle to [-π, π]
     *
     * Ensures rotation stays in range [-π, π] radians ([-180°, 180°]).
     * This prevents numerical issues from rotation accumulating over time.
     *
     * Example:
     * - rotation = 3π → normalized to π
     * - rotation = -3π → normalized to -π
     * - rotation = 2π → normalized to 0
     */
    void normalizeRotation();

    /**
     * @brief Rotate the transform by an additional angle
     * @param angle Angle in radians to add to current rotation
     */
    void rotate(float angle);

    /**
     * @brief Translate the transform by a displacement
     * @param displacement Vector to add to current position
     */
    void translate(const Vector2& displacement);

    /**
     * @brief Set rotation to look towards a target point
     * @param target The point to look at
     *
     * Adjusts rotation so that the forward direction points toward target.
     */
    void lookAt(const Vector2& target);

    // ========================================
    // Static Helper Functions
    // ========================================

    /**
     * @brief Create an identity transform (origin, no rotation)
     * @return Identity transform
     */
    static Transform identity();

    /**
     * @brief Create a transform with only translation (no rotation)
     * @param position The position
     * @return Transform at position with no rotation
     */
    static Transform translation(const Vector2& position);

    /**
     * @brief Create a transform with only rotation (at origin)
     * @param rotation The rotation angle in radians
     * @return Transform at origin with rotation
     */
    static Transform rotationTransform(float rotation);

    // ========================================
    // Helper Functions (Internal Use)
    // ========================================

private:
    /**
     * @brief Rotate a vector by an angle
     * @param v The vector to rotate
     * @param angle The rotation angle in radians
     * @return Rotated vector
     *
     * Rotation matrix:
     * [ cos(θ)  -sin(θ) ] [ x ]   [ x*cos(θ) - y*sin(θ) ]
     * [ sin(θ)   cos(θ) ] [ y ] = [ x*sin(θ) + y*cos(θ) ]
     */
    static Vector2 rotate(const Vector2& v, float angle);
};

} // namespace Physics

#endif // TRANSFORM_HPP
