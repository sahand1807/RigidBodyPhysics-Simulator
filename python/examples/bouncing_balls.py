#!/usr/bin/env python3
"""
Bouncing Balls Simulation

Simple demo with balls bouncing on a ground.
Good for testing basic physics and rendering.
"""

from physics_viz import Simulation, Vector2, RigidBody, CircleCollider
import random


class BouncingBalls(Simulation):
    """Simple bouncing balls demonstration"""

    def __init__(self):
        super().__init__(title="Bouncing Balls")

    def setup(self):
        """Create ground and some bouncing balls"""
        # Normal gravity
        self.world.set_gravity(Vector2(0, -9.81))

        # Create ground (large static circle)
        ground = RigidBody()
        ground_collider = CircleCollider(50.0)
        ground.set_collider(ground_collider)
        ground.set_static()
        ground.set_position(Vector2(0, -50))
        ground.set_restitution(0.8)  # Bouncy ground
        self.world.add_body(ground)

        # Create several balls
        for i in range(10):
            ball = RigidBody()

            # Varying sizes
            radius = random.uniform(0.5, 2.0)
            collider = CircleCollider(radius)
            ball.set_collider(collider)

            # Calculate mass from density
            density = 1.0
            mass = collider.calculate_mass(density)
            inertia = collider.calculate_inertia(mass)
            ball.set_mass(mass)
            ball.set_moment_of_inertia(inertia)

            # Random position
            x = random.uniform(-15, 15)
            y = random.uniform(5, 20)
            ball.set_position(Vector2(x, y))

            # Random initial velocity
            vx = random.uniform(-5, 5)
            vy = random.uniform(-2, 2)
            ball.set_velocity(Vector2(vx, vy))

            # High restitution for bouncy balls
            ball.set_restitution(random.uniform(0.7, 0.95))
            ball.set_friction(0.2)

            self.world.add_body(ball)

        # Setup camera
        self.renderer.camera.focus_on(Vector2(0, 5))
        self.renderer.camera.pixels_per_meter = 20.0


def main():
    sim = BouncingBalls()
    sim.run()


if __name__ == "__main__":
    main()
