"""
Simulation runner for physics visualization

Manages the main simulation loop, input handling, and rendering.
"""

import pygame
from .physics_engine_core import PhysicsWorld, Vector2
from .renderer import Renderer


class Simulation:
    """Base class for physics simulations

    Handles the main loop, input, timing, and rendering.
    Subclass this to create specific demo scenarios.
    """

    def __init__(self, width=1280, height=720, title="Physics Simulation"):
        """Initialize simulation

        Args:
            width: Window width in pixels
            height: Window height in pixels
            title: Window title
        """
        # Create renderer
        self.renderer = Renderer(width, height, title)

        # Create physics world
        self.world = PhysicsWorld()
        self.world.set_gravity(Vector2(0, -9.81))  # Earth gravity

        # Simulation state
        self.running = False
        self.paused = False
        self.show_help = False

        # Timing
        self.time_scale = 1.0  # Speed multiplier
        self.fixed_dt = 1.0 / 60.0  # Fixed timestep (60 Hz)
        self.accumulator = 0.0  # For fixed timestep

        # Input state
        self.mouse_pos = (0, 0)
        self.mouse_down = False
        self.mouse_drag_start = None

    def setup(self):
        """Setup simulation (override in subclasses)

        Create bodies, set initial conditions, etc.
        """
        pass

    def update(self, dt):
        """Update simulation logic (override in subclasses)

        Args:
            dt: Delta time in seconds
        """
        pass

    def on_key_press(self, key):
        """Handle key press events (override in subclasses)

        Args:
            key: pygame key constant
        """
        pass

    def on_mouse_click(self, pos, button):
        """Handle mouse click events (override in subclasses)

        Args:
            pos: Mouse position (x, y) in screen space
            button: Mouse button (1=left, 2=middle, 3=right)
        """
        pass

    def handle_input(self):
        """Process input events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                self.handle_key_press(event.key)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.mouse_down = True
                self.mouse_pos = event.pos

                if event.button == 1:  # Left click
                    self.mouse_drag_start = event.pos
                    self.on_mouse_click(event.pos, 1)

                elif event.button == 3:  # Right click
                    self.on_mouse_click(event.pos, 3)

                elif event.button == 4:  # Scroll up
                    self.renderer.camera.zoom(1.1, event.pos)

                elif event.button == 5:  # Scroll down
                    self.renderer.camera.zoom(0.9, event.pos)

            elif event.type == pygame.MOUSEBUTTONUP:
                self.mouse_down = False
                self.mouse_drag_start = None

            elif event.type == pygame.MOUSEMOTION:
                old_pos = self.mouse_pos
                self.mouse_pos = event.pos

                # Pan camera with left drag
                if self.mouse_down and self.mouse_drag_start:
                    dx = event.pos[0] - old_pos[0]
                    dy = event.pos[1] - old_pos[1]
                    self.renderer.camera.pan(-dx, -dy)

    def handle_key_press(self, key):
        """Handle keyboard input

        Args:
            key: pygame key constant
        """
        if key == pygame.K_ESCAPE:
            self.running = False

        elif key == pygame.K_SPACE:
            self.paused = not self.paused

        elif key == pygame.K_r:
            self.reset()

        elif key == pygame.K_g:
            self.renderer.show_grid = not self.renderer.show_grid

        elif key == pygame.K_v:
            self.renderer.show_velocity = not self.renderer.show_velocity

        elif key == pygame.K_h:
            self.show_help = not self.show_help

        elif key == pygame.K_EQUALS or key == pygame.K_PLUS:
            self.time_scale *= 1.2

        elif key == pygame.K_MINUS:
            self.time_scale *= 0.8

        # Pass to subclass handler
        self.on_key_press(key)

    def reset(self):
        """Reset simulation to initial state"""
        self.world.clear()
        self.setup()
        self.paused = False

    def step_physics(self, dt):
        """Step physics simulation with fixed timestep

        Uses accumulator for stable integration.

        Args:
            dt: Delta time in seconds
        """
        if self.paused:
            return

        # Apply time scale
        dt *= self.time_scale

        # Accumulator for fixed timestep
        self.accumulator += dt

        # Step physics in fixed increments
        while self.accumulator >= self.fixed_dt:
            self.world.step(self.fixed_dt)
            self.accumulator -= self.fixed_dt

    def render(self):
        """Render frame"""
        self.renderer.clear()
        self.renderer.draw_world(self.world)

        if self.show_help:
            self.renderer.draw_help()

        if self.paused:
            self.renderer.draw_text("PAUSED", (self.renderer.width // 2 - 50, 10),
                                   font='large', color='highlight')

        self.renderer.flip()

    def run(self):
        """Run simulation loop"""
        self.running = True

        # Setup simulation
        self.setup()

        # Main loop
        while self.running:
            # Handle input
            self.handle_input()

            # Get delta time
            dt = self.renderer.tick(60)

            # Update custom logic
            self.update(dt)

            # Step physics
            self.step_physics(dt)

            # Render
            self.render()

        # Cleanup
        self.renderer.quit()


class InteractiveSandbox(Simulation):
    """Interactive physics sandbox

    Click to spawn objects, drag to apply forces.
    """

    def __init__(self):
        super().__init__(title="Interactive Physics Sandbox")
        self.spawn_mode = 'circle'  # Type of object to spawn

    def setup(self):
        """Setup sandbox with ground"""
        from .physics_engine_core import RigidBody, BoxCollider

        # Create ground (box platform)
        ground = RigidBody()
        ground_box = BoxCollider(150.0, 3.0)  # Very wide platform, 3m thick
        ground.set_collider(ground_box)
        ground.set_static()
        ground.set_position(Vector2(0, -10))
        self.world.add_body(ground)

    def on_mouse_click(self, pos, button):
        """Spawn object on click

        Args:
            pos: Mouse position in screen space
            button: Mouse button
        """
        from .physics_engine_core import RigidBody, CircleCollider
        import random

        if button == 1:  # Left click spawns object
            # Convert to world space
            world_pos = self.renderer.camera.screen_to_world(pos)

            # Create ball
            ball = RigidBody()
            radius = random.uniform(0.5, 2.0)
            collider = CircleCollider(radius)
            ball.set_collider(collider)

            mass = collider.calculate_mass(1.0)  # Density = 1
            inertia = collider.calculate_inertia(mass)
            ball.set_mass(mass)
            ball.set_moment_of_inertia(inertia)

            ball.set_position(world_pos)
            ball.set_restitution(random.uniform(0.3, 0.9))
            ball.set_friction(0.5)

            self.world.add_body(ball)

    def on_key_press(self, key):
        """Handle additional keys

        Args:
            key: pygame key constant
        """
        if key == pygame.K_c:
            self.world.clear()
            self.setup()
