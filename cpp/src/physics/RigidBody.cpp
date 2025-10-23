/**
 * @file RigidBody.cpp
 * @brief Implementation of 2D Rigid Body physics
 */

#include "physics/RigidBody.hpp"
#include <cmath>
#include <algorithm>

namespace Physics {

// ========================================
// Constructors
// ========================================

RigidBody::RigidBody()
    : transform()
    , mass(1.0f)
    , invMass(1.0f)
    , velocity(0.0f, 0.0f)
    , forceAccumulator(0.0f, 0.0f)
    , momentOfInertia(1.0f)
    , invMomentOfInertia(1.0f)
    , angularVelocity(0.0f)
    , torqueAccumulator(0.0f)
    , restitution(0.5f)
    , friction(0.5f)
{
}

RigidBody::RigidBody(float mass)
    : RigidBody()  // Delegate to default constructor
{
    setMass(mass);
}

// ========================================
// Physical Properties (Setters)
// ========================================

void RigidBody::setTransform(const Transform& transform) {
    this->transform = transform;
}

void RigidBody::setPosition(const Vector2& position) {
    transform.position = position;
}

void RigidBody::setRotation(float rotation) {
    transform.rotation = rotation;
}

void RigidBody::setMass(float mass) {
    // Validate mass
    if (mass <= 0.0f) {
        // Invalid mass, make static
        this->mass = 0.0f;
        this->invMass = 0.0f;
        return;
    }

    this->mass = mass;
    this->invMass = 1.0f / mass;
}

void RigidBody::setMomentOfInertia(float inertia) {
    // Validate moment of inertia
    if (inertia <= 0.0f) {
        // Invalid inertia, make rotationally static
        this->momentOfInertia = 0.0f;
        this->invMomentOfInertia = 0.0f;
        return;
    }

    this->momentOfInertia = inertia;
    this->invMomentOfInertia = 1.0f / inertia;
}

void RigidBody::setVelocity(const Vector2& velocity) {
    this->velocity = velocity;
}

void RigidBody::setAngularVelocity(float angularVelocity) {
    this->angularVelocity = angularVelocity;
}

void RigidBody::setRestitution(float restitution) {
    // Clamp to valid range [0, 1]
    this->restitution = std::max(0.0f, std::min(1.0f, restitution));
}

void RigidBody::setFriction(float friction) {
    // Friction can be > 1, but not negative
    this->friction = std::max(0.0f, friction);
}

// ========================================
// Static vs Dynamic
// ========================================

void RigidBody::setStatic() {
    mass = 0.0f;
    invMass = 0.0f;
    momentOfInertia = 0.0f;
    invMomentOfInertia = 0.0f;

    // Clear velocities
    velocity = Vector2::zero();
    angularVelocity = 0.0f;

    // Clear forces
    forceAccumulator = Vector2::zero();
    torqueAccumulator = 0.0f;
}

void RigidBody::setDynamic(float mass, float inertia) {
    setMass(mass);
    setMomentOfInertia(inertia);
}

// ========================================
// Force and Torque Application
// ========================================

void RigidBody::applyForce(const Vector2& force) {
    // Static bodies don't accumulate forces
    if (isStatic()) return;

    forceAccumulator += force;
}

void RigidBody::applyForceAtPoint(const Vector2& force, const Vector2& point) {
    // Static bodies don't accumulate forces
    if (isStatic()) return;

    // Apply linear force
    forceAccumulator += force;

    // Calculate torque: τ = r × F
    // In 2D, cross product gives scalar: r.x * F.y - r.y * F.x
    Vector2 r = point - transform.position;  // Vector from center to point
    float torque = r.cross(force);

    // Apply torque
    torqueAccumulator += torque;
}

void RigidBody::applyTorque(float torque) {
    // Static bodies don't accumulate torques
    if (isStatic()) return;

    torqueAccumulator += torque;
}

void RigidBody::applyImpulse(const Vector2& impulse) {
    // Static bodies don't respond to impulses
    if (isStatic()) return;

    // Impulse directly changes velocity: Δv = J/m
    velocity += impulse * invMass;
}

void RigidBody::applyImpulseAtPoint(const Vector2& impulse, const Vector2& point) {
    // Static bodies don't respond to impulses
    if (isStatic()) return;

    // Apply linear impulse
    velocity += impulse * invMass;

    // Calculate angular impulse: ΔL = r × J
    Vector2 r = point - transform.position;
    float angularImpulse = r.cross(impulse);

    // Apply angular impulse: Δω = ΔL/I
    angularVelocity += angularImpulse * invMomentOfInertia;
}

void RigidBody::applyAngularImpulse(float impulse) {
    // Static bodies don't respond to impulses
    if (isStatic()) return;

    // Δω = impulse/I
    angularVelocity += impulse * invMomentOfInertia;
}

void RigidBody::clearForces() {
    forceAccumulator = Vector2::zero();
    torqueAccumulator = 0.0f;
}

// ========================================
// Integration (Physics Update)
// ========================================

void RigidBody::integrate(float dt) {
    // Static bodies don't move
    if (isStatic()) return;

    // Semi-implicit Euler integration for linear motion
    // Step 1: Calculate acceleration from accumulated forces
    // a = F / m
    Vector2 acceleration = forceAccumulator * invMass;

    // Step 2: Update velocity (v = v + a*dt)
    velocity += acceleration * dt;

    // Step 3: Update position using NEW velocity (x = x + v*dt)
    transform.position += velocity * dt;

    // Semi-implicit Euler integration for angular motion
    // Step 1: Calculate angular acceleration from accumulated torques
    // α = τ / I
    float angularAcceleration = torqueAccumulator * invMomentOfInertia;

    // Step 2: Update angular velocity (ω = ω + α*dt)
    angularVelocity += angularAcceleration * dt;

    // Step 3: Update rotation using NEW angular velocity (θ = θ + ω*dt)
    transform.rotation += angularVelocity * dt;

    // Normalize rotation to keep it in reasonable range
    // (Prevents floating point errors from accumulating)
    transform.normalizeRotation();
}

// ========================================
// Utility Functions
// ========================================

float RigidBody::getKineticEnergy() const {
    // KE = 0.5 * m * v² + 0.5 * I * ω²
    // (translational kinetic energy + rotational kinetic energy)

    float translationalKE = 0.5f * mass * velocity.lengthSquared();
    float rotationalKE = 0.5f * momentOfInertia * angularVelocity * angularVelocity;

    return translationalKE + rotationalKE;
}

Vector2 RigidBody::getMomentum() const {
    // p = m * v
    return velocity * mass;
}

float RigidBody::getAngularMomentum() const {
    // L = I * ω
    return momentOfInertia * angularVelocity;
}

Vector2 RigidBody::localToWorld(const Vector2& localPoint) const {
    return transform.transformPoint(localPoint);
}

Vector2 RigidBody::worldToLocal(const Vector2& worldPoint) const {
    return transform.inverseTransformPoint(worldPoint);
}

Vector2 RigidBody::getVelocityAtPoint(const Vector2& worldPoint) const {
    // For a rotating body, different points have different velocities
    // v_point = v_center + ω × r
    //
    // In 2D, cross product with scalar: ω × r = ω * perpendicular(r)
    // If r = (x, y), then perpendicular(r) = (-y, x)
    // So: ω × r = ω * (-y, x) = (-ω*y, ω*x)

    // Calculate vector from center of mass to point
    Vector2 r = worldPoint - transform.position;

    // Calculate velocity contribution from rotation
    // ω × r in 2D
    Vector2 rotationalVelocity(-angularVelocity * r.y, angularVelocity * r.x);

    // Total velocity is linear velocity + rotational contribution
    return velocity + rotationalVelocity;
}

} // namespace Physics