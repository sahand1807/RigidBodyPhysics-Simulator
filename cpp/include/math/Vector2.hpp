/**
 * @file Vector2.hpp
 * @brief 2D Vector mathematics for physics simulation
 *
 * This file defines a 2D vector class used throughout the physics engine
 * for representing positions, velocities, forces, and directions.
 */

#ifndef VECTOR2_HPP
#define VECTOR2_HPP

#include <cmath>
#include <ostream>

namespace Physics {

/**
 * @class Vector2
 * @brief Represents a 2D vector with x and y components
 *
 * A Vector2 can represent:
 * - Positions: (x, y) coordinates in 2D space
 * - Velocities: (vx, vy) speed in x and y directions
 * - Forces: (fx, fy) force components
 * - Directions: normalized vectors pointing somewhere
 *
 * Example usage:
 * @code
 * Vector2 position(5.0f, 10.0f);
 * Vector2 velocity(2.0f, -1.0f);
 * position = position + velocity * dt;
 * @endcode
 */
class Vector2 {
public:
    // Components
    float x;  ///< X component
    float y;  ///< Y component

    // ========================================
    // Constructors
    // ========================================

    /**
     * @brief Default constructor - creates zero vector (0, 0)
     */
    Vector2();

    /**
     * @brief Construct vector with specific x and y values
     * @param x The x component
     * @param y The y component
     */
    Vector2(float x, float y);

    // ========================================
    // Basic Vector Operations
    // ========================================

    /**
     * @brief Add two vectors component-wise
     * @param other The vector to add
     * @return Sum of the two vectors
     *
     * Example: (1, 2) + (3, 4) = (4, 6)
     * Used for: position += velocity * dt
     */
    Vector2 operator+(const Vector2& other) const;

    /**
     * @brief Subtract two vectors component-wise
     * @param other The vector to subtract
     * @return Difference of the two vectors
     *
     * Example: (5, 3) - (2, 1) = (3, 2)
     * Used for: finding direction from point A to point B
     */
    Vector2 operator-(const Vector2& other) const;

    /**
     * @brief Multiply vector by a scalar
     * @param scalar The scalar value to multiply by
     * @return Scaled vector
     *
     * Example: (2, 3) * 2 = (4, 6)
     * Used for: scaling velocities, forces
     */
    Vector2 operator*(float scalar) const;

    /**
     * @brief Divide vector by a scalar
     * @param scalar The scalar value to divide by
     * @return Scaled vector
     *
     * Example: (4, 6) / 2 = (2, 3)
     * Used for: averaging, normalization
     */
    Vector2 operator/(float scalar) const;

    /**
     * @brief Negate vector (flip direction)
     * @return Vector pointing in opposite direction
     *
     * Example: -(2, 3) = (-2, -3)
     */
    Vector2 operator-() const;

    // ========================================
    // Compound Assignment Operators
    // ========================================

    /**
     * @brief Add another vector to this one
     * @param other The vector to add
     * @return Reference to this vector
     */
    Vector2& operator+=(const Vector2& other);

    /**
     * @brief Subtract another vector from this one
     * @param other The vector to subtract
     * @return Reference to this vector
     */
    Vector2& operator-=(const Vector2& other);

    /**
     * @brief Multiply this vector by a scalar
     * @param scalar The scalar to multiply by
     * @return Reference to this vector
     */
    Vector2& operator*=(float scalar);

    /**
     * @brief Divide this vector by a scalar
     * @param scalar The scalar to divide by
     * @return Reference to this vector
     */
    Vector2& operator/=(float scalar);

    // ========================================
    // Vector Mathematics
    // ========================================

    /**
     * @brief Calculate dot product with another vector
     * @param other The other vector
     * @return Dot product (scalar value)
     *
     * Dot product = x1*x2 + y1*y2
     *
     * Properties:
     * - If dot product = 0, vectors are perpendicular
     * - If dot product > 0, angle < 90 degrees
     * - If dot product < 0, angle > 90 degrees
     *
     * Used in: collision detection, projection calculations
     */
    float dot(const Vector2& other) const;

    /**
     * @brief Calculate 2D cross product (returns scalar, not vector)
     * @param other The other vector
     * @return Cross product magnitude (z-component in 3D)
     *
     * Cross product = x1*y2 - y1*x2
     *
     * Properties:
     * - If cross product > 0, other is counterclockwise from this
     * - If cross product < 0, other is clockwise from this
     * - If cross product = 0, vectors are parallel
     *
     * Used in: determining rotation direction, winding order
     */
    float cross(const Vector2& other) const;

    /**
     * @brief Calculate the length (magnitude) of the vector
     * @return Length of the vector
     *
     * Length = sqrt(x² + y²)
     *
     * Used for: speeds, distances, normalizing vectors
     */
    float length() const;

    /**
     * @brief Calculate squared length (faster than length())
     * @return Squared length
     *
     * Squared length = x² + y²
     *
     * Useful for comparisons where actual length isn't needed,
     * avoids expensive sqrt() operation
     */
    float lengthSquared() const;

    /**
     * @brief Normalize this vector (make length = 1)
     * @return Reference to this vector
     *
     * Converts vector to unit vector pointing in same direction.
     * If length is 0, vector remains (0, 0)
     *
     * Used for: direction vectors, surface normals
     */
    Vector2& normalize();

    /**
     * @brief Get normalized copy of this vector
     * @return Normalized vector
     *
     * Returns a unit vector without modifying the original.
     */
    Vector2 normalized() const;

    /**
     * @brief Calculate distance to another point
     * @param other The other point
     * @return Distance between the two points
     *
     * Distance = length(this - other)
     */
    float distance(const Vector2& other) const;

    /**
     * @brief Calculate squared distance (faster than distance())
     * @param other The other point
     * @return Squared distance
     *
     * Useful for distance comparisons without sqrt()
     */
    float distanceSquared(const Vector2& other) const;

    // ========================================
    // Utility Functions
    // ========================================

    /**
     * @brief Check if this is a zero vector
     * @return True if both components are zero
     */
    bool isZero() const;

    /**
     * @brief Get perpendicular vector (rotated 90° counterclockwise)
     * @return Perpendicular vector
     *
     * Example: (x, y) -> (-y, x)
     *
     * Used for: surface normals, tangent vectors
     */
    Vector2 perpendicular() const;

    // ========================================
    // Static Helper Functions
    // ========================================

    /**
     * @brief Create zero vector (0, 0)
     * @return Zero vector
     */
    static Vector2 zero();

    /**
     * @brief Create unit vector pointing right (1, 0)
     * @return Right vector
     */
    static Vector2 right();

    /**
     * @brief Create unit vector pointing up (0, 1)
     * @return Up vector
     */
    static Vector2 up();
};

// ========================================
// Non-member Operators
// ========================================

/**
 * @brief Multiply scalar by vector (allows: 2.0f * vector)
 * @param scalar The scalar value
 * @param vec The vector
 * @return Scaled vector
 */
Vector2 operator*(float scalar, const Vector2& vec);

/**
 * @brief Output vector to stream for debugging
 * @param os Output stream
 * @param vec The vector to output
 * @return Reference to output stream
 *
 * Format: Vector2(x, y)
 */
std::ostream& operator<<(std::ostream& os, const Vector2& vec);

} // namespace Physics

#endif // VECTOR2_HPP