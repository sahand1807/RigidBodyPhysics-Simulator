/**
 * @file PhysicsWorld.hpp
 * @brief Physics simulation world manager
 *
 * PhysicsWorld manages the entire physics simulation, including:
 * - Collection of all RigidBody objects
 * - Global forces (gravity)
 * - Simulation stepping (integration loop)
 * - Force clearing
 *
 * This is the main interface for running physics simulations.
 */

#ifndef PHYSICSWORLD_HPP
#define PHYSICSWORLD_HPP

#include "math/Vector2.hpp"
#include "physics/RigidBody.hpp"
#include <vector>
#include <memory>

namespace Physics {

/**
 * @class PhysicsWorld
 * @brief Manages physics simulation of multiple rigid bodies
 *
 * PhysicsWorld is the central manager for physics simulation. It:
 * - Stores all RigidBody objects in the simulation
 * - Applies global forces like gravity
 * - Advances the simulation forward in time via step()
 * - Manages the integration loop
 *
 * Typical usage:
 * @code
 * PhysicsWorld world;
 * world.setGravity(Vector2(0, -9.81f));  // Earth gravity
 *
 * // Create and add bodies
 * RigidBody* ball = new RigidBody();
 * ball->setMass(1.0f);
 * ball->setPosition(Vector2(0, 10));
 * world.addBody(ball);
 *
 * // Simulation loop
 * float dt = 1.0f / 60.0f;  // 60 FPS
 * for (int i = 0; i < 100; i++) {
 *     world.step(dt);
 *     std::cout << "Position: " << ball->getPosition() << std::endl;
 * }
 * @endcode
 *
 * Integration Loop (in step()):
 * 1. Apply global forces (gravity) to all bodies
 * 2. Integrate each body (velocity → position update)
 * 3. Clear force accumulators for next frame
 *
 * Memory Management:
 * - PhysicsWorld does NOT own the RigidBody pointers
 * - User is responsible for deleting bodies
 * - Use removeBody() before deleting a body
 * - clear() only removes references, doesn't delete
 */
class PhysicsWorld {
public:
    // ========================================
    // Constructors and Destructors
    // ========================================

    /**
     * @brief Construct empty physics world
     *
     * Default settings:
     * - Gravity: (0, -9.81) m/s² (Earth gravity downward)
     * - No bodies
     */
    PhysicsWorld();

    /**
     * @brief Destructor
     *
     * Note: Does NOT delete RigidBody objects!
     * User must manage body lifetime separately.
     */
    ~PhysicsWorld() = default;

    // ========================================
    // Body Management
    // ========================================

    /**
     * @brief Add a rigid body to the simulation
     * @param body Pointer to RigidBody to add
     *
     * The body will be affected by:
     * - Global gravity (if not static)
     * - Integration during step()
     *
     * Note: PhysicsWorld does NOT take ownership.
     * Do not delete the body without calling removeBody() first.
     *
     * Example:
     * @code
     * RigidBody* ball = new RigidBody();
     * world.addBody(ball);
     * // ... simulate ...
     * world.removeBody(ball);
     * delete ball;
     * @endcode
     */
    void addBody(RigidBody* body);

    /**
     * @brief Remove a rigid body from the simulation
     * @param body Pointer to RigidBody to remove
     * @return True if body was found and removed, false otherwise
     *
     * After removal, the body is no longer affected by the simulation.
     * User is still responsible for deleting the body.
     *
     * Example:
     * @code
     * world.removeBody(ball);
     * delete ball;  // Now safe to delete
     * @endcode
     */
    bool removeBody(RigidBody* body);

    /**
     * @brief Remove all bodies from the simulation
     *
     * Note: Does NOT delete the bodies!
     * User must delete bodies separately if needed.
     */
    void clear();

    /**
     * @brief Get all bodies in the simulation
     * @return Const reference to vector of body pointers
     *
     * Use this to iterate over all bodies:
     * @code
     * for (RigidBody* body : world.getBodies()) {
     *     std::cout << body->getPosition() << std::endl;
     * }
     * @endcode
     */
    const std::vector<RigidBody*>& getBodies() const { return bodies; }

    /**
     * @brief Get number of bodies in simulation
     * @return Number of bodies
     */
    size_t getBodyCount() const { return bodies.size(); }

    // ========================================
    // Simulation Control
    // ========================================

    /**
     * @brief Advance the simulation by a time step
     * @param dt Time step in seconds
     *
     * This performs the complete integration loop:
     * 1. Apply gravity to all non-static bodies
     * 2. Integrate each body (update velocity and position)
     * 3. Clear force accumulators
     *
     * Typical dt values:
     * - 60 FPS: dt = 1/60 ≈ 0.0167 seconds
     * - 30 FPS: dt = 1/30 ≈ 0.0333 seconds
     * - Fixed step: dt = 0.01 or 0.02 seconds
     *
     * Example:
     * @code
     * // 60 FPS game loop
     * const float dt = 1.0f / 60.0f;
     * while (running) {
     *     world.step(dt);
     *     render();
     * }
     * @endcode
     *
     * Physics stability:
     * - Smaller dt → more accurate but slower
     * - Larger dt → less accurate, may become unstable
     * - Recommended: dt < 0.02 seconds (50+ FPS)
     */
    void step(float dt);

    // ========================================
    // World Settings
    // ========================================

    /**
     * @brief Set global gravity acceleration
     * @param gravity Gravity vector in m/s²
     *
     * Common values:
     * - Earth: Vector2(0, -9.81)  ← downward
     * - Moon: Vector2(0, -1.62)
     * - Mars: Vector2(0, -3.71)
     * - Space: Vector2(0, 0)
     * - Custom: Vector2(x, y) for any direction
     *
     * Gravity is applied as: F = m × g
     *
     * Note: Static bodies (invMass = 0) are not affected by gravity.
     *
     * Example:
     * @code
     * world.setGravity(Vector2(0, -9.81f));  // Earth
     * world.setGravity(Vector2(-5, 0));      // Pull to the left
     * world.setGravity(Vector2::zero());     // Zero gravity
     * @endcode
     */
    void setGravity(const Vector2& gravity);

    /**
     * @brief Get current gravity
     * @return Gravity vector in m/s²
     */
    const Vector2& getGravity() const { return gravity; }

private:
    // ========================================
    // Private Members
    // ========================================

    std::vector<RigidBody*> bodies;  ///< All bodies in the simulation
    Vector2 gravity;                  ///< Global gravity acceleration (m/s²)

    // ========================================
    // Private Helper Methods
    // ========================================

    /**
     * @brief Apply gravity to all non-static bodies
     * @param dt Time step in seconds
     *
     * For each dynamic body:
     * - Calculate gravity force: F = m × g
     * - Apply force to body
     *
     * Static bodies (invMass = 0) are skipped.
     */
    void applyGravity(float dt);
};

} // namespace Physics

#endif // PHYSICSWORLD_HPP