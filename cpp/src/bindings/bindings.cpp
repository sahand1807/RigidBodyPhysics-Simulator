/**
 * @file bindings.cpp
 * @brief Python bindings for the physics engine using pybind11
 *
 * This file exposes all C++ classes and functions to Python,
 * allowing the physics engine to be used from Python scripts.
 */

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "math/Vector2.hpp"
#include "math/Transform.hpp"
#include "physics/RigidBody.hpp"
#include "physics/Collider.hpp"
#include "physics/CircleCollider.hpp"
#include "physics/PhysicsWorld.hpp"

#include <cmath>

namespace py = pybind11;
using namespace Physics;

// Helper function for approximate equality
inline bool approxEqual(float a, float b, float epsilon = 1e-5f) {
    return std::abs(a - b) < epsilon;
}

PYBIND11_MODULE(physics_engine_core, m) {
    m.doc() = "2D Rigid Body Physics Engine - C++ core with Python bindings";

    // ========================================
    // Vector2 Class
    // ========================================
    py::class_<Vector2>(m, "Vector2")
        .def(py::init<>(), "Create zero vector")
        .def(py::init<float, float>(), "Create vector with x and y components",
             py::arg("x"), py::arg("y"))

        // Properties
        .def_readwrite("x", &Vector2::x, "X component")
        .def_readwrite("y", &Vector2::y, "Y component")

        // Arithmetic operators
        .def(py::self + py::self, "Add two vectors")
        .def(py::self - py::self, "Subtract two vectors")
        .def(py::self * float(), "Multiply vector by scalar")
        .def(float() * py::self, "Multiply scalar by vector")
        .def(py::self / float(), "Divide vector by scalar")
        .def(-py::self, "Negate vector")

        // Comparison operators (implement via lambda since not in C++)
        .def("__eq__", [](const Vector2& a, const Vector2& b) {
            return approxEqual(a.x, b.x) && approxEqual(a.y, b.y);
        }, py::is_operator())
        .def("__ne__", [](const Vector2& a, const Vector2& b) {
            return !approxEqual(a.x, b.x) || !approxEqual(a.y, b.y);
        }, py::is_operator())

        // Compound assignment
        .def("__iadd__", [](Vector2& self, const Vector2& other) {
            self += other;
            return self;
        }, py::is_operator())
        .def("__isub__", [](Vector2& self, const Vector2& other) {
            self -= other;
            return self;
        }, py::is_operator())
        .def("__imul__", [](Vector2& self, float scalar) {
            self *= scalar;
            return self;
        }, py::is_operator())
        .def("__itruediv__", [](Vector2& self, float scalar) {
            self /= scalar;
            return self;
        }, py::is_operator())

        // Methods
        .def("length", &Vector2::length, "Get magnitude of vector")
        .def("length_squared", &Vector2::lengthSquared, "Get squared magnitude (faster)")
        .def("normalize", &Vector2::normalize, "Normalize vector in-place, returns self")
        .def("normalized", &Vector2::normalized, "Return normalized copy")
        .def("dot", &Vector2::dot, "Dot product with another vector", py::arg("other"))
        .def("cross", &Vector2::cross, "2D cross product (returns scalar)", py::arg("other"))
        .def("distance", &Vector2::distance, "Distance to another vector", py::arg("other"))
        .def("distance_squared", &Vector2::distanceSquared, "Squared distance (faster)", py::arg("other"))

        // String representation
        .def("__repr__", [](const Vector2& v) {
            return "Vector2(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
        })

        // Static members
        .def_static("zero", &Vector2::zero, "Return zero vector (0, 0)")
        .def_static("up", &Vector2::up, "Return up vector (0, 1)")
        .def_static("right", &Vector2::right, "Return right vector (1, 0)");

    // ========================================
    // Transform Class
    // ========================================
    py::class_<Transform>(m, "Transform")
        .def(py::init<>(), "Create identity transform")
        .def(py::init<const Vector2&, float>(), "Create transform with position and rotation",
             py::arg("position"), py::arg("rotation") = 0.0f)

        // Properties
        .def_readwrite("position", &Transform::position, "Position in world space")
        .def_readwrite("rotation", &Transform::rotation, "Rotation angle in radians")

        // Methods
        .def("transform_point", &Transform::transformPoint,
             "Transform a point from local to world space", py::arg("local_point"))
        .def("transform_direction", &Transform::transformDirection,
             "Transform a direction vector (ignores position)", py::arg("local_direction"))
        .def("inverse_transform_point", &Transform::inverseTransformPoint,
             "Transform a point from world to local space", py::arg("world_point"))
        .def("inverse_transform_direction", &Transform::inverseTransformDirection,
             "Transform a direction from world to local space", py::arg("world_direction"))
        .def("inverse", &Transform::inverse, "Get inverse transform")
        .def("normalize_rotation", &Transform::normalizeRotation, "Normalize rotation to [-π, π)")

        // Directional vectors
        .def("get_forward", &Transform::getForward, "Get forward direction vector (rotated right)")
        .def("get_right", &Transform::getRight, "Get right direction vector")
        .def("get_up", &Transform::getUp, "Get up direction vector (rotated up)")

        // String representation
        .def("__repr__", [](const Transform& t) {
            return "Transform(pos=(" + std::to_string(t.position.x) + ", " +
                   std::to_string(t.position.y) + "), rot=" +
                   std::to_string(t.rotation) + ")";
        });

    // ========================================
    // Collider Base Class
    // ========================================
    py::class_<Collider>(m, "Collider")
        .def("get_type", &Collider::getType, "Get collider type")
        .def("calculate_mass", &Collider::calculateMass, "Calculate mass from density",
             py::arg("density"))
        .def("calculate_inertia", &Collider::calculateInertia, "Calculate moment of inertia",
             py::arg("mass"))
        .def("get_bounds", [](const Collider& c) {
            Vector2 min, max;
            c.getBounds(min, max);
            return py::make_tuple(min, max);
        }, "Get bounding box as (min, max) tuple")
        .def("get_offset", &Collider::getOffset, "Get local offset from body center")
        .def("set_offset", &Collider::setOffset, "Set local offset", py::arg("offset"))
        .def("get_world_center", &Collider::getWorldCenter, "Get world space center",
             py::arg("body_transform"));

    // Collider Type enum
    py::enum_<Collider::Type>(m, "ColliderType")
        .value("Circle", Collider::Type::Circle)
        .value("Box", Collider::Type::Box)
        .export_values();

    // ========================================
    // CircleCollider Class
    // ========================================
    py::class_<CircleCollider, Collider>(m, "CircleCollider")
        .def(py::init<float>(), "Create circle collider with radius",
             py::arg("radius"))
        .def(py::init<float, const Vector2&>(), "Create circle collider with radius and offset",
             py::arg("radius"), py::arg("offset"))

        .def("get_radius", &CircleCollider::getRadius, "Get circle radius")
        .def("set_radius", &CircleCollider::setRadius, "Set circle radius", py::arg("radius"))
        .def("contains_point", &CircleCollider::containsPoint, "Check if point is inside circle",
             py::arg("point"))

        .def("__repr__", [](const CircleCollider& c) {
            return "CircleCollider(radius=" + std::to_string(c.getRadius()) + ")";
        });

    // ========================================
    // RigidBody Class
    // ========================================
    py::class_<RigidBody>(m, "RigidBody")
        .def(py::init<>(), "Create rigid body with default properties")
        .def(py::init<float>(), "Create rigid body with specific mass", py::arg("mass"))

        // Transform properties
        .def("get_transform", &RigidBody::getTransform, "Get body transform",
             py::return_value_policy::reference_internal)
        .def("set_transform", &RigidBody::setTransform, "Set body transform", py::arg("transform"))
        .def("get_position", &RigidBody::getPosition, "Get position")
        .def("set_position", &RigidBody::setPosition, "Set position", py::arg("position"))
        .def("get_rotation", &RigidBody::getRotation, "Get rotation in radians")
        .def("set_rotation", &RigidBody::setRotation, "Set rotation", py::arg("rotation"))

        // Physical properties
        .def("get_mass", &RigidBody::getMass, "Get mass in kg")
        .def("set_mass", &RigidBody::setMass, "Set mass", py::arg("mass"))
        .def("get_inverse_mass", &RigidBody::getInverseMass, "Get inverse mass (1/m)")
        .def("get_moment_of_inertia", &RigidBody::getMomentOfInertia, "Get moment of inertia")
        .def("set_moment_of_inertia", &RigidBody::setMomentOfInertia, "Set moment of inertia",
             py::arg("inertia"))

        // Velocities
        .def("get_velocity", &RigidBody::getVelocity, "Get linear velocity")
        .def("set_velocity", &RigidBody::setVelocity, "Set linear velocity", py::arg("velocity"))
        .def("get_angular_velocity", &RigidBody::getAngularVelocity, "Get angular velocity")
        .def("set_angular_velocity", &RigidBody::setAngularVelocity, "Set angular velocity",
             py::arg("angular_velocity"))

        // Material properties
        .def("get_restitution", &RigidBody::getRestitution, "Get restitution (bounciness)")
        .def("set_restitution", &RigidBody::setRestitution, "Set restitution", py::arg("restitution"))
        .def("get_friction", &RigidBody::getFriction, "Get friction coefficient")
        .def("set_friction", &RigidBody::setFriction, "Set friction", py::arg("friction"))

        // Static/dynamic
        .def("is_static", &RigidBody::isStatic, "Check if body is static (infinite mass)")
        .def("set_static", &RigidBody::setStatic, "Make body static (infinite mass)")
        .def("set_dynamic", &RigidBody::setDynamic, "Make body dynamic",
             py::arg("mass"), py::arg("inertia"))

        // Forces and impulses
        .def("apply_force", &RigidBody::applyForce, "Apply force at center of mass", py::arg("force"))
        .def("apply_force_at_point", &RigidBody::applyForceAtPoint, "Apply force at point",
             py::arg("force"), py::arg("point"))
        .def("apply_torque", &RigidBody::applyTorque, "Apply torque", py::arg("torque"))
        .def("apply_impulse", &RigidBody::applyImpulse, "Apply impulse at center", py::arg("impulse"))
        .def("apply_impulse_at_point", &RigidBody::applyImpulseAtPoint, "Apply impulse at point",
             py::arg("impulse"), py::arg("point"))
        .def("apply_angular_impulse", &RigidBody::applyAngularImpulse, "Apply angular impulse",
             py::arg("impulse"))
        .def("clear_forces", &RigidBody::clearForces, "Clear accumulated forces")

        // Integration
        .def("integrate", &RigidBody::integrate, "Update position and velocity", py::arg("dt"))

        // Utility
        .def("get_kinetic_energy", &RigidBody::getKineticEnergy, "Get total kinetic energy")
        .def("get_momentum", &RigidBody::getMomentum, "Get linear momentum")
        .def("get_angular_momentum", &RigidBody::getAngularMomentum, "Get angular momentum")
        .def("local_to_world", &RigidBody::localToWorld, "Transform point to world space",
             py::arg("local_point"))
        .def("world_to_local", &RigidBody::worldToLocal, "Transform point to local space",
             py::arg("world_point"))
        .def("get_velocity_at_point", &RigidBody::getVelocityAtPoint, "Get velocity at point",
             py::arg("world_point"))

        // Collider
        .def("set_collider", &RigidBody::setCollider, "Attach collider to body",
             py::arg("collider"), py::keep_alive<1, 2>())  // Keep collider alive while body exists
        .def("get_collider", &RigidBody::getCollider, "Get attached collider",
             py::return_value_policy::reference_internal)
        .def("has_collider", &RigidBody::hasCollider, "Check if body has collider")

        .def("__repr__", [](const RigidBody& b) {
            return "RigidBody(mass=" + std::to_string(b.getMass()) +
                   ", pos=(" + std::to_string(b.getPosition().x) + ", " +
                   std::to_string(b.getPosition().y) + "))";
        });

    // ========================================
    // PhysicsWorld Class
    // ========================================
    py::class_<PhysicsWorld>(m, "PhysicsWorld")
        .def(py::init<>(), "Create physics world with default settings")

        // Body management
        .def("add_body", &PhysicsWorld::addBody, "Add rigid body to simulation",
             py::arg("body"), py::keep_alive<1, 2>())  // Keep body alive while world exists
        .def("remove_body", &PhysicsWorld::removeBody, "Remove body from simulation",
             py::arg("body"))
        .def("clear", &PhysicsWorld::clear, "Remove all bodies")
        .def("get_bodies", &PhysicsWorld::getBodies, "Get all bodies",
             py::return_value_policy::reference_internal)
        .def("get_body_count", &PhysicsWorld::getBodyCount, "Get number of bodies")

        // Simulation
        .def("step", &PhysicsWorld::step, "Advance simulation by time step", py::arg("dt"))

        // Settings
        .def("set_gravity", &PhysicsWorld::setGravity, "Set gravity vector", py::arg("gravity"))
        .def("get_gravity", &PhysicsWorld::getGravity, "Get gravity vector")

        .def("__repr__", [](const PhysicsWorld& w) {
            return "PhysicsWorld(bodies=" + std::to_string(w.getBodyCount()) + ")";
        });
}
