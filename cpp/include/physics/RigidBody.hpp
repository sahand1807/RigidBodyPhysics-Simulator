/**
 * @file RigidBody.hpp
 * @brief 2D Rigid Body for physics simulation
 *
 * A RigidBody represents a physical object that can move and rotate
 * but does not deform. It has mass, velocity, and responds to forces.
 */

#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include "math/Vector2.hpp"
#include "math/Transform.hpp"

namespace Physics {

/**
 * @class RigidBody
 * @brief Represents a rigid body in 2D physics simulation
 *
 * A rigid body is an idealized solid object that:
 * - Has mass and moment of inertia
 * - Can translate (move linearly) and rotate (move angularly)
 * - Responds to forces and torques
 * - Does not deform under stress
 *
 * Physical Properties:
 * - Transform: position and rotation in world space
 * - Mass: resistance to linear acceleration (F = ma)
 * - Moment of Inertia: resistance to angular acceleration (τ = Iα)
 * - Linear velocity: rate of position change
 * - Angular velocity: rate of rotation change
 *
 * Material Properties:
 * - Restitution: bounciness (0 = no bounce, 1 = perfect bounce)
 * - Friction: resistance to sliding (0 = ice, 1 = rubber)
 *
 * Example usage:
 * @code
 * // Create a dynamic circle
 * RigidBody ball;
 * ball.setMass(1.0f);
 * ball.setMomentOfInertia(0.5f);
 * ball.setPosition(Vector2(0.0f, 10.0f));
 * ball.applyForce(Vector2(0.0f, -9.8f));  // Gravity
 * ball.integrate(0.016f);  // Update for 1/60 second
 * @endcode
 */
class RigidBody {
public:
    // ========================================
    // Constructors and Destructor
    // ========================================

    /**
     * @brief Default constructor - creates a dynamic body at origin
     *
     * Initial state:
     * - Position: (0, 0)
     * - Rotation: 0
     * - Mass: 1.0 kg
     * - Moment of inertia: 1.0
     * - All velocities: zero
     * - Restitution: 0.5 (moderate bounce)
     * - Friction: 0.5 (moderate friction)
     */
    RigidBody();

    /**
     * @brief Construct body with specific mass
     * @param mass The mass in kilograms (must be > 0)
     */
    explicit RigidBody(float mass);

    // ========================================
    // Physical Properties (Getters/Setters)
    // ========================================

    /**
     * @brief Get the body's transform (position and rotation)
     * @return Current transform
     */
    const Transform& getTransform() const { return transform; }

    /**
     * @brief Set the body's transform
     * @param transform New transform
     */
    void setTransform(const Transform& transform);

    /**
     * @brief Get position in world space
     * @return Position vector
     */
    Vector2 getPosition() const { return transform.position; }

    /**
     * @brief Set position in world space
     * @param position New position
     */
    void setPosition(const Vector2& position);

    /**
     * @brief Get rotation angle in radians
     * @return Rotation angle
     */
    float getRotation() const { return transform.rotation; }

    /**
     * @brief Set rotation angle
     * @param rotation New rotation in radians
     */
    void setRotation(float rotation);

    /**
     * @brief Get mass in kilograms
     * @return Mass (returns 0 for static bodies)
     */
    float getMass() const { return mass; }

    /**
     * @brief Set mass
     * @param mass New mass in kg (must be > 0 for dynamic bodies)
     *
     * Setting mass automatically updates inverse mass.
     * Use setStatic() to create immovable objects.
     */
    void setMass(float mass);

    /**
     * @brief Get inverse mass (1/mass)
     * @return Inverse mass (0 for static bodies)
     *
     * Inverse mass is used in calculations for efficiency.
     * Static bodies have invMass = 0, making F/m = F*0 = 0 (no acceleration).
     */
    float getInverseMass() const { return invMass; }

    /**
     * @brief Get moment of inertia (rotational mass)
     * @return Moment of inertia
     *
     * Moment of inertia determines resistance to angular acceleration.
     * For common shapes:
     * - Circle: I = 0.5 * m * r²
     * - Box: I = (1/12) * m * (w² + h²)
     */
    float getMomentOfInertia() const { return momentOfInertia; }

    /**
     * @brief Set moment of inertia
     * @param inertia New moment of inertia (must be > 0 for dynamic bodies)
     */
    void setMomentOfInertia(float inertia);

    /**
     * @brief Get inverse moment of inertia
     * @return Inverse moment of inertia (0 for static bodies)
     */
    float getInverseMomentOfInertia() const { return invMomentOfInertia; }

    /**
     * @brief Get linear velocity
     * @return Velocity vector in units/second
     */
    const Vector2& getVelocity() const { return velocity; }

    /**
     * @brief Set linear velocity
     * @param velocity New velocity vector
     */
    void setVelocity(const Vector2& velocity);

    /**
     * @brief Get angular velocity (rotation rate)
     * @return Angular velocity in radians/second
     *
     * Positive = counterclockwise, negative = clockwise
     */
    float getAngularVelocity() const { return angularVelocity; }

    /**
     * @brief Set angular velocity
     * @param angularVelocity New angular velocity in rad/s
     */
    void setAngularVelocity(float angularVelocity);

    // ========================================
    // Material Properties
    // ========================================

    /**
     * @brief Get coefficient of restitution (bounciness)
     * @return Restitution in range [0, 1]
     *
     * - 0.0 = perfectly inelastic (no bounce, like clay)
     * - 0.5 = moderate bounce (like wood)
     * - 1.0 = perfectly elastic (perfect bounce, like super ball)
     *
     * In collision, combined restitution = max(e1, e2) or average
     */
    float getRestitution() const { return restitution; }

    /**
     * @brief Set coefficient of restitution
     * @param restitution Bounciness [0, 1]
     */
    void setRestitution(float restitution);

    /**
     * @brief Get coefficient of friction
     * @return Friction coefficient [0, 1+]
     *
     * - 0.0 = frictionless (ice)
     * - 0.3 = low friction (wet surface)
     * - 0.5 = moderate friction (wood on wood)
     * - 1.0+ = high friction (rubber)
     *
     * In collision, combined friction = sqrt(μ1 * μ2)
     */
    float getFriction() const { return friction; }

    /**
     * @brief Set coefficient of friction
     * @param friction Friction coefficient [0, 1+]
     */
    void setFriction(float friction);

    // ========================================
    // Static vs Dynamic
    // ========================================

    /**
     * @brief Check if body is static (immovable)
     * @return True if static
     *
     * Static bodies:
     * - Have infinite mass (invMass = 0)
     * - Don't respond to forces
     * - Don't move or rotate
     * - Useful for walls, floors, platforms
     */
    bool isStatic() const { return invMass == 0.0f; }

    /**
     * @brief Make body static (infinite mass)
     *
     * Sets invMass and invMomentOfInertia to 0, making body immovable.
     * Useful for terrain, walls, platforms.
     */
    void setStatic();

    /**
     * @brief Make body dynamic (normal mass)
     * @param mass New mass (must be > 0)
     * @param inertia New moment of inertia (must be > 0)
     */
    void setDynamic(float mass, float inertia);

    // ========================================
    // Force and Torque Application
    // ========================================

    /**
     * @brief Apply a force to the center of mass
     * @param force Force vector in Newtons
     *
     * Force is accumulated and applied during integration.
     * F = ma → a = F/m
     *
     * Example: Apply gravity: applyForce(Vector2(0, -9.8) * mass)
     */
    void applyForce(const Vector2& force);

    /**
     * @brief Apply a force at a world space point
     * @param force Force vector in Newtons
     * @param point Point of application in world space
     *
     * This applies both linear force and torque (rotation).
     * Torque = r × F (cross product of radius and force)
     *
     */
    void applyForceAtPoint(const Vector2& force, const Vector2& point);

    /**
     * @brief Apply a torque (rotational force)
     * @param torque Torque in Newton-meters
     *
     * Torque causes angular acceleration: α = τ/I
     * Positive torque = counterclockwise rotation
     * Negative torque = clockwise rotation
     */
    void applyTorque(float torque);

    /**
     * @brief Apply an impulse (instantaneous velocity change)
     * @param impulse Impulse vector in Newton-seconds (kg⋅m/s)
     *
     * Impulse directly changes velocity without waiting for integration.
     * Δv = J/m
     *
     * Used for: collision response, explosions, instant forces
     *
     * Example: Make object jump: applyImpulse(Vector2(0, 5) * mass)
     */
    void applyImpulse(const Vector2& impulse);

    /**
     * @brief Apply an impulse at a world space point
     * @param impulse Impulse vector in Newton-seconds
     * @param point Point of application in world space
     *
     * Applies both linear and angular impulse.
     * Used in collision resolution.
     */
    void applyImpulseAtPoint(const Vector2& impulse, const Vector2& point);

    /**
     * @brief Apply an angular impulse (instant rotation change)
     * @param impulse Angular impulse in kg⋅m²/s
     *
     * Directly changes angular velocity: Δω = impulse/I
     */
    void applyAngularImpulse(float impulse);

    /**
     * @brief Clear all accumulated forces and torques
     *
     * Should be called after integration, before next frame.
     * Forces are temporary and must be reapplied each frame.
     */
    void clearForces();

    // ========================================
    // Integration (Physics Update)
    // ========================================

    /**
     * @brief Update position and velocity using semi-implicit Euler
     * @param dt Time step in seconds (typically 1/60 = 0.0166...)
     *
     * Semi-implicit Euler integration:
     * 1. Calculate acceleration: a = F/m
     * 2. Update velocity: v(t+dt) = v(t) + a * dt
     * 3. Update position: x(t+dt) = x(t) + v(t+dt) * dt  (uses NEW velocity!)
     *
     * Angular motion:
     * 1. α = τ/I
     * 2. ω(t+dt) = ω(t) + α * dt
     * 3. θ(t+dt) = θ(t) + ω(t+dt) * dt
     *
     * This method has better energy conservation than explicit Euler.
     *
     * Static bodies are not integrated.
     */
    void integrate(float dt);

    // ========================================
    // Utility Functions
    // ========================================

    /**
     * @brief Get kinetic energy of the body
     * @return Total kinetic energy in Joules
     *
     * KE = 0.5 * m * v² + 0.5 * I * ω²
     *      (translational + rotational)
     */
    float getKineticEnergy() const;

    /**
     * @brief Get linear momentum
     * @return Momentum vector (kg⋅m/s)
     *
     * p = m * v
     */
    Vector2 getMomentum() const;

    /**
     * @brief Get angular momentum
     * @return Angular momentum (kg⋅m²/s)
     *
     * L = I * ω
     */
    float getAngularMomentum() const;

    /**
     * @brief Transform a point from local space to world space
     * @param localPoint Point in body's local coordinates
     * @return Point in world coordinates
     *
     * Convenience function that uses the body's transform.
     */
    Vector2 localToWorld(const Vector2& localPoint) const;

    /**
     * @brief Transform a point from world space to local space
     * @param worldPoint Point in world coordinates
     * @return Point in body's local coordinates
     *
     * Useful for collision detection in local space.
     */
    Vector2 worldToLocal(const Vector2& worldPoint) const;

    /**
     * @brief Get velocity at a world space point on the body
     * @param worldPoint Point in world coordinates
     * @return Velocity at that point
     *
     * For rotating bodies, different points have different velocities.
     * v_point = v_center + ω × r
     * where r is the vector from center to point
     *
     * In 2D: v_point = v_center + ω * perpendicular(r)
     */
    Vector2 getVelocityAtPoint(const Vector2& worldPoint) const;

private:
    // ========================================
    // Physical State
    // ========================================

    Transform transform;  ///< Position and rotation in world space

    // Linear motion
    float mass;                ///< Mass in kilograms
    float invMass;             ///< Inverse mass (1/mass), 0 for static
    Vector2 velocity;          ///< Linear velocity in m/s
    Vector2 forceAccumulator;  ///< Sum of all forces this frame

    // Angular motion
    float momentOfInertia;     ///< Moment of inertia (rotational mass)
    float invMomentOfInertia;  ///< Inverse moment of inertia, 0 for static
    float angularVelocity;     ///< Angular velocity in rad/s
    float torqueAccumulator;   ///< Sum of all torques this frame

    // Material properties
    float restitution;  ///< Coefficient of restitution [0, 1]
    float friction;     ///< Coefficient of friction [0, 1+]
};

} // namespace Physics

#endif // RIGIDBODY_HPP