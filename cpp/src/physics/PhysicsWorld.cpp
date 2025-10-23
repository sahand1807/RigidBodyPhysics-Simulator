/**
 * @file PhysicsWorld.cpp
 * @brief Implementation of PhysicsWorld
 */

#include "physics/PhysicsWorld.hpp"
#include "physics/CollisionDetection.hpp"
#include "physics/CollisionResponse.hpp"
#include <algorithm>

namespace Physics {

// ========================================
// Constructors
// ========================================

PhysicsWorld::PhysicsWorld()
    : gravity(0.0f, -9.81f)  // Earth gravity (downward)
{
    // Empty world, ready for bodies to be added
}

// ========================================
// Body Management
// ========================================

void PhysicsWorld::addBody(RigidBody* body) {
    if (body == nullptr) {
        return;  // Ignore null pointers
    }

    // Check if body is already in the world
    auto it = std::find(bodies.begin(), bodies.end(), body);
    if (it != bodies.end()) {
        return;  // Already added, don't duplicate
    }

    bodies.push_back(body);
}

bool PhysicsWorld::removeBody(RigidBody* body) {
    if (body == nullptr) {
        return false;
    }

    // Find the body in the vector
    auto it = std::find(bodies.begin(), bodies.end(), body);
    if (it == bodies.end()) {
        return false;  // Body not found
    }

    // Remove it
    bodies.erase(it);
    return true;
}

void PhysicsWorld::clear() {
    bodies.clear();
}

// ========================================
// Simulation Control
// ========================================

void PhysicsWorld::step(float dt) {
    // 1. Apply global forces (gravity)
    applyGravity(dt);

    // 2. Integrate all bodies (velocity → position update)
    for (RigidBody* body : bodies) {
        body->integrate(dt);
    }

    // 3. Detect and resolve collisions
    detectAndResolveCollisions();

    // 4. Clear force accumulators for next frame
    for (RigidBody* body : bodies) {
        body->clearForces();
    }
}

// ========================================
// World Settings
// ========================================

void PhysicsWorld::setGravity(const Vector2& gravity) {
    this->gravity = gravity;
}

// ========================================
// Private Helper Methods
// ========================================

void PhysicsWorld::applyGravity(float dt) {
    for (RigidBody* body : bodies) {
        // Skip static bodies (they don't move)
        if (body->isStatic()) {
            continue;
        }

        // Apply gravity force: F = m × g
        // Note: We apply force, not impulse, so it accumulates with other forces
        Vector2 gravityForce = gravity * body->getMass();
        body->applyForce(gravityForce);
    }
}

void PhysicsWorld::detectAndResolveCollisions() {
    // Check all pairs of bodies for collisions
    // O(n²) complexity - could be optimized with spatial partitioning
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            RigidBody* bodyA = bodies[i];
            RigidBody* bodyB = bodies[j];

            // Skip if either body has no collider
            if (!bodyA->hasCollider() || !bodyB->hasCollider()) {
                continue;
            }

            // Skip if both bodies are static (they can't move anyway)
            if (bodyA->isStatic() && bodyB->isStatic()) {
                continue;
            }

            // Detect collision
            Manifold manifold;
            if (CollisionDetection::detectCollision(bodyA, bodyB, manifold)) {
                // Collision detected! Resolve it
                CollisionResponse::resolveCollision(manifold);
            }
        }
    }
}

} // namespace Physics