#!/usr/bin/env python3
"""
Pyramid Stacking Simulation

Creates a pyramid of balls and tests collision stability.
Demonstrates stacking and resting contact.
"""

from physics_viz import Simulation, Vector2, RigidBody, CircleCollider
import math


class Pyramid(Simulation):
    """Pyramid stacking demonstration"""

    def __init__(self):
        super().__init__(title="Pyramid Stack")
        self.pyramid_height = 8

    def setup(self):
        """Create ground and pyramid of balls"""
        # Normal gravity
        self.world.set_gravity(Vector2(0, -9.81))

        # Create ground
        ground = RigidBody()
        ground_collider = CircleCollider(100.0)
        ground.set_collider(ground_collider)
        ground.set_static()
        ground.set_position(Vector2(0, -100))
        ground.set_restitution(0.3)  # Low bounce
        self.world.add_body(ground)

        # Build pyramid
        ball_radius = 0.8
        spacing = ball_radius * 2.05  # Slight gap for stability

        y_offset = ball_radius  # Start just above ground

        for row in range(self.pyramid_height, 0, -1):
            # Calculate row position
            row_y = y_offset + (self.pyramid_height - row) * spacing * 0.866  # sin(60°)

            # Calculate starting x for centering
            row_width = (row - 1) * spacing
            start_x = -row_width / 2.0

            # Create balls in this row
            for col in range(row):
                ball = RigidBody()
                collider = CircleCollider(ball_radius)
                ball.set_collider(collider)

                # Calculate mass
                density = 1.0
                mass = collider.calculate_mass(density)
                inertia = collider.calculate_inertia(mass)
                ball.set_mass(mass)
                ball.set_moment_of_inertia(inertia)

                # Position
                x = start_x + col * spacing
                ball.set_position(Vector2(x, row_y))

                # Low restitution for stable stacking
                ball.set_restitution(0.2)
                ball.set_friction(0.5)

                self.world.add_body(ball)

        # Setup camera
        pyramid_center_y = y_offset + self.pyramid_height * spacing * 0.433
        self.renderer.camera.focus_on(Vector2(0, pyramid_center_y))
        self.renderer.camera.pixels_per_meter = 25.0

    def on_key_press(self, key):
        """Handle key presses"""
        import pygame

        if key == pygame.K_SPACE:
            # On spacebar, add a projectile
            self.launch_projectile()

    def launch_projectile(self):
        """Launch a ball at the pyramid"""
        ball = RigidBody()
        collider = CircleCollider(0.5)
        ball.set_collider(collider)

        density = 2.0  # Heavier projectile
        mass = collider.calculate_mass(density)
        inertia = collider.calculate_inertia(mass)
        ball.set_mass(mass)
        ball.set_moment_of_inertia(inertia)

        # Position to the left
        ball.set_position(Vector2(-15, 3))

        # High velocity toward pyramid
        ball.set_velocity(Vector2(20, 2))

        ball.set_restitution(0.7)
        ball.set_friction(0.3)

        self.world.add_body(ball)


def main():
    pyramid = Pyramid()
    print("Press SPACE to launch a projectile at the pyramid!")
    pyramid.run()


if __name__ == "__main__":
    main()
