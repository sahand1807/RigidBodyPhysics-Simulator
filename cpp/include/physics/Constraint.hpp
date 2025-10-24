/**
 * @file Constraint.hpp
 * @brief Base class for physics constraints
 *
 * Constraints are used to limit the motion of rigid bodies,
 * enforcing geometric or kinematic relationships.
 *
 * Examples:
 * - DistanceConstraint: Keeps two points at fixed distance (ropes, strings)
 * - HingeConstraint: Allows rotation around a fixed axis (doors, wheels)
 * - PointConstraint: Fixes a point in space (pin, nail)
 *
 * Constraints are solved iteratively using XPBD (Extended Position-Based Dynamics)
 * for stability and ease of implementation.
 */

#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

namespace Physics {

// Forward declaration
class RigidBody;

/**
 * @class Constraint
 * @brief Abstract base class for all physics constraints
 *
 * A constraint restricts the motion of one or more rigid bodies
 * by enforcing geometric or kinematic relationships.
 *
 * Constraints are solved after velocity integration but before
 * collision detection, and again after collision resolution
 * for stability.
 *
 * Derived classes must implement:
 * - solve(): Apply constraint correction to body positions/velocities
 * - isValid(): Check if constraint is properly configured
 */
class Constraint {
public:
    // ========================================
    // Constructor & Destructor
    // ========================================

    /**
     * @brief Virtual destructor for proper cleanup of derived classes
     */
    virtual ~Constraint() = default;

    // ========================================
    // Core Constraint Interface
    // ========================================

    /**
     * @brief Solve the constraint for one iteration
     *
     * This method should calculate the constraint error and apply
     * corrections to the involved bodies to reduce that error.
     *
     * Called multiple times per frame (5-10 iterations) to converge
     * to a solution.
     *
     * @param dt Time step (seconds)
     */
    virtual void solve(float dt) = 0;

    /**
     * @brief Check if constraint is valid and ready to solve
     *
     * @return True if constraint has valid bodies and parameters
     */
    virtual bool isValid() const = 0;

    // ========================================
    // Body Query Methods
    // ========================================

    /**
     * @brief Check if this constraint involves the given body
     *
     * Useful for:
     * - Removing constraints when a body is destroyed
     * - Finding all constraints affecting a body
     * - Debugging constraint networks
     *
     * @param body The body to check
     * @return True if this constraint affects the given body
     */
    virtual bool involves(const RigidBody* body) const = 0;

    // ========================================
    // Enable/Disable
    // ========================================

    /**
     * @brief Enable or disable this constraint
     *
     * Disabled constraints are not solved, allowing temporary
     * suspension of constraint behavior.
     *
     * @param enabled True to enable, false to disable
     */
    void setEnabled(bool enabled) { this->enabled = enabled; }

    /**
     * @brief Check if constraint is enabled
     * @return True if enabled
     */
    bool isEnabled() const { return enabled; }

protected:
    // ========================================
    // Protected Members
    // ========================================

    bool enabled = true;  ///< Whether constraint is active

    /**
     * @brief Default constructor (protected - only derived classes can create)
     */
    Constraint() = default;
};

} // namespace Physics

#endif // CONSTRAINT_HPP
