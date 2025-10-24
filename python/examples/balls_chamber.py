#!/usr/bin/env python3
"""
Balls Chamber Simulation

Random balls shooting around inside a closed chamber.
High energy collisions with walls and each other.
"""

from physics_viz import Simulation, Vector2, RigidBody, CircleCollider, BoxCollider
import random


class BallsChamber(Simulation):
    """Energetic balls bouncing around inside a closed chamber"""

    def __init__(self):
        super().__init__(title="Balls Chamber - High Energy Collisions")

    def setup(self):
        """Create chamber walls and random high-velocity balls"""
        # Normal gravity
        self.world.set_gravity(Vector2(0, -9.81))

        # Chamber dimensions
        chamber_width = 40.0
        chamber_height = 30.0
        wall_thickness = 2.0

        # Create chamber walls (4 static boxes forming a rectangle)

        # Bottom wall
        bottom = RigidBody()
        bottom.set_collider(BoxCollider(chamber_width, wall_thickness))
        bottom.set_static()
        bottom.set_position(Vector2(0, -chamber_height/2))
        bottom.set_restitution(0.9)
        self.world.add_body(bottom)

        # Top wall
        top = RigidBody()
        top.set_collider(BoxCollider(chamber_width, wall_thickness))
        top.set_static()
        top.set_position(Vector2(0, chamber_height/2))
        top.set_restitution(0.9)
        self.world.add_body(top)

        # Left wall
        left = RigidBody()
        left.set_collider(BoxCollider(wall_thickness, chamber_height))
        left.set_static()
        left.set_position(Vector2(-chamber_width/2, 0))
        left.set_restitution(0.9)
        self.world.add_body(left)

        # Right wall
        right = RigidBody()
        right.set_collider(BoxCollider(wall_thickness, chamber_height))
        right.set_static()
        right.set_position(Vector2(chamber_width/2, 0))
        right.set_restitution(0.9)
        self.world.add_body(right)

        # Create random balls with high velocities
        num_balls = 20

        for i in range(num_balls):
            ball = RigidBody()

            # Random sizes
            radius = random.uniform(0.4, 1.0)
            collider = CircleCollider(radius)
            ball.set_collider(collider)

            # Calculate mass from density
            density = 1.0
            mass = collider.calculate_mass(density)
            inertia = collider.calculate_inertia(mass)
            ball.set_mass(mass)
            ball.set_moment_of_inertia(inertia)

            # Random position inside chamber (avoid spawning too close to walls)
            margin = 4.0
            x = random.uniform(-chamber_width/2 + margin, chamber_width/2 - margin)
            y = random.uniform(-chamber_height/2 + margin, chamber_height/2 - margin)
            ball.set_position(Vector2(x, y))

            # Random high velocity shooting in random directions
            vx = random.uniform(-12, 12)
            vy = random.uniform(-12, 12)
            ball.set_velocity(Vector2(vx, vy))

            # Very high restitution for energetic bouncing
            ball.set_restitution(0.9)
            # No friction to keep energy high

            self.world.add_body(ball)

        # Setup camera to view the entire chamber
        self.renderer.camera.focus_on(Vector2(0, 0))
        self.renderer.camera.pixels_per_meter = 15.0


def main():
    sim = BallsChamber()
    sim.run()


if __name__ == "__main__":
    main()
