#!/usr/bin/env python3
"""
Ball Pit Simulation

Drops many balls into a container and watches them bounce and settle.
Demonstrates collision handling with many objects.
"""

from physics_viz import Simulation, Vector2, RigidBody, CircleCollider, BoxCollider
import random


class BallPit(Simulation):
    """Ball pit demonstration with container"""

    def __init__(self):
        super().__init__(title="Ball Pit")
        self.spawn_timer = 0.0
        self.spawn_interval = 0.1  # Spawn every 0.1 seconds
        self.max_balls = 100

    def setup(self):
        """Create container and start spawning balls"""
        # Normal gravity
        self.world.set_gravity(Vector2(0, -9.81))

        # Create container walls (box colliders)

        # Bottom
        bottom = RigidBody()
        bottom_box = BoxCollider(40.0, 2.0)  # 40m wide, 2m thick
        bottom.set_collider(bottom_box)
        bottom.set_static()
        bottom.set_position(Vector2(0, -10))
        bottom.set_restitution(0.6)
        self.world.add_body(bottom)

        # Left wall
        left_wall = RigidBody()
        left_box = BoxCollider(2.0, 25.0)  # 2m thick, 25m tall
        left_wall.set_collider(left_box)
        left_wall.set_static()
        left_wall.set_position(Vector2(-21, 2))
        left_wall.set_restitution(0.6)
        self.world.add_body(left_wall)

        # Right wall
        right_wall = RigidBody()
        right_box = BoxCollider(2.0, 25.0)  # 2m thick, 25m tall
        right_wall.set_collider(right_box)
        right_wall.set_static()
        right_wall.set_position(Vector2(21, 2))
        right_wall.set_restitution(0.6)
        self.world.add_body(right_wall)

        # Setup camera
        self.renderer.camera.focus_on(Vector2(0, 5))
        self.renderer.camera.pixels_per_meter = 15.0

    def spawn_ball(self):
        """Spawn a random ball"""
        if self.world.get_body_count() >= self.max_balls + 3:  # +3 for walls
            return

        ball = RigidBody()

        # Random radius
        radius = random.uniform(0.3, 1.2)
        collider = CircleCollider(radius)
        ball.set_collider(collider)

        # Calculate mass
        density = 1.0
        mass = collider.calculate_mass(density)
        inertia = collider.calculate_inertia(mass)
        ball.set_mass(mass)
        ball.set_moment_of_inertia(inertia)

        # Random position at top
        x = random.uniform(-8, 8)
        y = random.uniform(15, 20)
        ball.set_position(Vector2(x, y))

        # Random initial velocity
        vx = random.uniform(-2, 2)
        ball.set_velocity(Vector2(vx, 0))

        # Random restitution (bounciness)
        ball.set_restitution(random.uniform(0.3, 0.8))
        ball.set_friction(0.3)

        self.world.add_body(ball)

    def update(self, dt):
        """Update simulation - spawn balls periodically"""
        if self.paused:
            return

        self.spawn_timer += dt
        if self.spawn_timer >= self.spawn_interval:
            self.spawn_ball()
            self.spawn_timer = 0.0


def main():
    ball_pit = BallPit()
    ball_pit.run()


if __name__ == "__main__":
    main()
