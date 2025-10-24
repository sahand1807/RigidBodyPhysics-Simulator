#!/usr/bin/env python3
"""
Simple script to record all demo GIFs.

Just run: python3 scripts/record_all_demos.py

Each demo will run for 10 seconds and automatically save as GIF.
"""

import sys
from pathlib import Path

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))

import pygame
from physics_viz.gif_recorder import GifRecorder


def record_sandbox():
    """Record sandbox demo."""
    from python.examples.sandbox import Sandbox

    recorder = GifRecorder("demos/sandbox.gif", duration=10, fps=15)

    demo = Sandbox()

    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update simulation
        demo.step_physics()

        # Render
        demo.render()
        pygame.display.flip()

        # Capture frame
        if not recorder.capture(demo.renderer.screen):
            running = False  # Stop after duration

        clock.tick(60)

    recorder.save()
    pygame.quit()


def record_momentum_transfer():
    """Record momentum transfer demo."""
    from python.examples.momentum_transfer import MomentumTransfer

    recorder = GifRecorder("demos/momentum_transfer.gif", duration=10, fps=15)

    demo = MomentumTransfer()

    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        demo.step_physics()
        demo.render()
        pygame.display.flip()

        if not recorder.capture(demo.renderer.screen):
            running = False

        clock.tick(60)

    recorder.save()
    pygame.quit()


def record_balls_chamber():
    """Record balls chamber demo."""
    from python.examples.balls_chamber import BallsChamber

    recorder = GifRecorder("demos/balls_chamber.gif", duration=8, fps=15)

    demo = BallsChamber()

    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        demo.step_physics()
        demo.render()
        pygame.display.flip()

        if not recorder.capture(demo.renderer.screen):
            running = False

        clock.tick(60)

    recorder.save()
    pygame.quit()


def record_ball_pit():
    """Record ball pit demo."""
    from python.examples.ball_pit import BallPit

    recorder = GifRecorder("demos/ball_pit.gif", duration=10, fps=15)

    demo = BallPit()

    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        demo.step_physics()
        demo.render()
        pygame.display.flip()

        if not recorder.capture(demo.renderer.screen):
            running = False

        clock.tick(60)

    recorder.save()
    pygame.quit()


def record_newtons_cradle():
    """Record Newton's cradle demo."""
    from python.examples.newtons_cradle import NewtonsCradle

    recorder = GifRecorder("demos/newtons_cradle.gif", duration=12, fps=15)

    demo = NewtonsCradle()

    # Auto-configure and start
    demo.num_balls_slider.value = 5
    demo.initial_angle_slider.value = 30
    demo.start_simulation()

    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if not demo.paused:
            demo.step_physics()

        demo.render()
        pygame.display.flip()

        if not recorder.capture(demo.renderer.screen):
            running = False

        clock.tick(60)

    recorder.save()
    pygame.quit()


def record_physics_builder():
    """
    Record physics builder demo.

    Note: This one requires manual interaction, so we'll create a preset scene.
    """
    from python.examples.physics_builder import PhysicsBuilder
    from physics_viz import Vector2, CircleCollider, BoxCollider

    recorder = GifRecorder("demos/physics_builder.gif", duration=10, fps=15)

    demo = PhysicsBuilder()

    # Create an interesting preset scene
    # Add some boxes at the bottom
    for i in range(5):
        body = demo.world.create_rigid_body()
        body.set_collider(BoxCollider(1.5, 0.8))
        body.set_position(Vector2(-4 + i * 2, 2))
        mass = body.get_collider().calculate_mass(1.0)
        body.set_mass(mass)
        body.set_restitution(0.9)
        demo.world.add_body(body)

    # Add some balls that will fall
    for i in range(8):
        body = demo.world.create_rigid_body()
        body.set_collider(CircleCollider(0.5))
        body.set_position(Vector2(-6 + i * 1.5, 10 + i * 0.5))
        mass = body.get_collider().calculate_mass(1.0)
        body.set_mass(mass)
        body.set_restitution(0.9)
        demo.world.add_body(body)

    # Start simulation
    demo.start_simulation()

    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if not demo.paused:
            demo.step_physics()

        demo.render()
        pygame.display.flip()

        if not recorder.capture(demo.renderer.screen):
            running = False

        clock.tick(60)

    recorder.save()
    pygame.quit()


def main():
    """Record all demos."""

    demos = [
        ("Newton's Cradle", record_newtons_cradle),
        ("Physics Builder", record_physics_builder),
        ("Momentum Transfer", record_momentum_transfer),
        ("Balls Chamber", record_balls_chamber),
        ("Ball Pit", record_ball_pit),
        ("Sandbox", record_sandbox),
    ]

    print("=" * 70)
    print("  Automatic Demo GIF Recorder")
    print("=" * 70)
    print(f"\nRecording {len(demos)} demos...\n")

    for i, (name, func) in enumerate(demos, 1):
        print(f"\n[{i}/{len(demos)}] {name}")
        print("-" * 70)

        try:
            func()
            print(f"✅ {name} recorded successfully!\n")
        except Exception as e:
            print(f"❌ Error recording {name}: {e}\n")
            import traceback
            traceback.print_exc()

        # Small delay between demos
        import time
        time.sleep(0.5)

    print("\n" + "=" * 70)
    print("  All demos recorded!")
    print("=" * 70)
    print("\nCheck the demos/ directory for the generated GIFs.")
    print("Don't forget to: git add demos/*.gif && git commit && git push")


if __name__ == '__main__':
    main()
