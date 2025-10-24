#!/usr/bin/env python3
"""
Momentum Transfer Simulation

Demonstrates conservation of momentum through elastic collisions.
A projectile ball hits a chain of stationary balls, transferring
momentum through the chain.
"""

from physics_viz import Simulation, Vector2, RigidBody, CircleCollider
import math


class MomentumTransfer(Simulation):
    """Momentum transfer through collision chain demonstration"""

    def __init__(self):
        super().__init__(title="Momentum Transfer - Elastic Collisions")
        self.balls = []
        self.projectile = None

    def setup(self):
        """Create stationary ball chain and projectile"""
        # No gravity - balls float in space
        self.world.set_gravity(Vector2(0, 0))

        # Configuration
        num_stationary_balls = 5
        ball_radius = 1.0
        separation = ball_radius * 2.0  # Just touching
        height = 10.0  # Vertical position

        # Create stationary balls in a row
        for i in range(num_stationary_balls):
            ball = RigidBody()
            collider = CircleCollider(ball_radius)
            ball.set_collider(collider)

            # Calculate mass and inertia
            density = 1.0
            mass = collider.calculate_mass(density)
            inertia = collider.calculate_inertia(mass)
            ball.set_mass(mass)
            ball.set_moment_of_inertia(inertia)

            # Position in a row (centered at x=0)
            x = i * separation
            ball.set_position(Vector2(x, height))

            # Start stationary
            ball.set_velocity(Vector2(0, 0))

            # High restitution for energy conservation
            ball.set_restitution(0.98)  # Nearly perfect elastic
            ball.set_friction(0.0)  # No friction

            self.world.add_body(ball)
            self.balls.append(ball)

        # Create projectile ball (incoming from left)
        self.projectile = RigidBody()
        projectile_collider = CircleCollider(ball_radius)
        self.projectile.set_collider(projectile_collider)

        # Same mass as other balls
        mass = projectile_collider.calculate_mass(density)
        inertia = projectile_collider.calculate_inertia(mass)
        self.projectile.set_mass(mass)
        self.projectile.set_moment_of_inertia(inertia)

        # Position far to the left
        projectile_start_x = -15.0
        self.projectile.set_position(Vector2(projectile_start_x, height))

        # Give it velocity toward the chain
        projectile_velocity = 10.0
        self.projectile.set_velocity(Vector2(projectile_velocity, 0))

        self.projectile.set_restitution(0.98)
        self.projectile.set_friction(0.0)

        self.world.add_body(self.projectile)

        # Center camera on the scene
        view_center_x = (projectile_start_x + num_stationary_balls * separation) / 2.0
        self.renderer.camera.focus_on(Vector2(view_center_x, height))
        self.renderer.camera.pixels_per_meter = 30.0

    def update(self, dt):
        """Update simulation"""
        # Could add momentum/energy statistics here
        pass


def main():
    sim = MomentumTransfer()
    print("Watch the projectile ball (left) hit the stationary chain!")
    print("Momentum transfers through the chain to the last ball.")
    sim.run()


if __name__ == "__main__":
    main()
