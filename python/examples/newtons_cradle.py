#!/usr/bin/env python3
"""
Interactive Newton's Cradle Demo with GUI Controls

Features:
- Sliders to adjust number of balls (3-10)
- Sliders to adjust rod length (2-8m)
- Sliders to adjust initial angle (0-60 degrees)
- Play button to start simulation
- Real-time energy display

Controls:
- Use sliders to configure the setup
- Click PLAY button to start simulation
- SPACE: Pause simulation
- R: Reset to initial state
- ESC: Quit
"""

from physics_viz import (
    Simulation, RigidBody, CircleCollider, Vector2, DistanceConstraint
)
import pygame
import math


class Slider:
    """Custom slider UI element"""

    def __init__(self, x, y, width, min_val, max_val, initial_val, label, step=1):
        self.rect = pygame.Rect(x, y, width, 20)
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial_val
        self.label = label
        self.step = step
        self.dragging = False

        # Colors
        self.track_color = (80, 80, 80)
        self.handle_color = (200, 200, 200)
        self.handle_hover_color = (255, 255, 100)
        self.label_color = (220, 220, 220)

    def get_handle_pos(self):
        """Calculate handle x position based on value"""
        ratio = (self.value - self.min_val) / (self.max_val - self.min_val)
        return int(self.rect.x + ratio * self.rect.width)

    def get_handle_rect(self):
        """Get handle rectangle"""
        handle_x = self.get_handle_pos()
        return pygame.Rect(handle_x - 8, self.rect.y - 5, 16, 30)

    def handle_event(self, event):
        """Handle mouse events"""
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                handle_rect = self.get_handle_rect()
                if handle_rect.collidepoint(event.pos):
                    self.dragging = True
                    return True

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self.dragging = False

        elif event.type == pygame.MOUSEMOTION:
            if self.dragging:
                # Calculate new value from mouse position
                x = event.pos[0]
                ratio = (x - self.rect.x) / self.rect.width
                ratio = max(0.0, min(1.0, ratio))
                new_value = self.min_val + ratio * (self.max_val - self.min_val)

                # Snap to step
                new_value = round(new_value / self.step) * self.step

                if new_value != self.value:
                    self.value = new_value
                    return True

        return False

    def draw(self, screen, font):
        """Draw the slider"""
        # Draw track
        pygame.draw.rect(screen, self.track_color, self.rect, border_radius=3)

        # Draw filled portion
        handle_x = self.get_handle_pos()
        filled_rect = pygame.Rect(self.rect.x, self.rect.y,
                                   handle_x - self.rect.x, self.rect.height)
        pygame.draw.rect(screen, (100, 150, 200), filled_rect, border_radius=3)

        # Draw handle
        handle_rect = self.get_handle_rect()
        mouse_pos = pygame.mouse.get_pos()
        handle_color = self.handle_hover_color if handle_rect.collidepoint(mouse_pos) else self.handle_color
        pygame.draw.rect(screen, handle_color, handle_rect, border_radius=4)

        # Draw label
        if isinstance(self.value, float):
            text = f"{self.label}: {self.value:.1f}"
        else:
            text = f"{self.label}: {int(self.value)}"

        label_surface = font.render(text, True, self.label_color)
        screen.blit(label_surface, (self.rect.x, self.rect.y - 25))


class Button:
    """Custom button UI element"""

    def __init__(self, x, y, width, height, label, color_scheme='green'):
        self.rect = pygame.Rect(x, y, width, height)
        self.label = label
        self.enabled = True
        self.color_scheme = color_scheme

        # Color schemes
        color_schemes = {
            'green': ((60, 120, 60), (80, 150, 80)),
            'orange': ((180, 100, 20), (220, 130, 30)),
            'blue': ((40, 80, 140), (60, 100, 160))
        }

        # Colors
        scheme = color_schemes.get(color_scheme, color_schemes['green'])
        self.bg_color = scheme[0]
        self.bg_hover_color = scheme[1]
        self.bg_disabled_color = (80, 80, 80)
        self.text_color = (255, 255, 255)
        self.text_disabled_color = (150, 150, 150)

    def handle_event(self, event):
        """Handle mouse events"""
        if not self.enabled:
            return False

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                if self.rect.collidepoint(event.pos):
                    return True

        return False

    def draw(self, screen, font):
        """Draw the button"""
        mouse_pos = pygame.mouse.get_pos()

        if self.enabled:
            if self.rect.collidepoint(mouse_pos):
                bg_color = self.bg_hover_color
            else:
                bg_color = self.bg_color
            text_color = self.text_color
        else:
            bg_color = self.bg_disabled_color
            text_color = self.text_disabled_color

        # Draw background
        pygame.draw.rect(screen, bg_color, self.rect, border_radius=5)
        pygame.draw.rect(screen, (100, 100, 100), self.rect, 2, border_radius=5)

        # Draw label (centered)
        label_surface = font.render(self.label, True, text_color)
        label_rect = label_surface.get_rect(center=self.rect.center)
        screen.blit(label_surface, label_rect)


class NewtonsCradle(Simulation):
    """Interactive Newton's Cradle with GUI controls"""

    def __init__(self):
        super().__init__(title="Interactive Newton's Cradle - Distance Constraints")
        self.balls = []
        self.constraints = []
        self.anchors = []

        # Configuration (user adjustable)
        self.num_balls = 5
        self.ball_radius = 0.5
        self.rod_length = 5.0
        self.initial_angle = 30.0  # degrees
        self.anchor_y = 10.0

        # Simulation state
        self.is_playing = False  # Start paused for setup
        self.paused = True

        # Mouse/camera state
        self.mouse_dragging = False
        self.last_mouse_pos = None

        # Disable default rendering features
        self.renderer.show_velocity = False
        self.renderer.show_stats = False

        # Create UI elements (right side panel)
        self.ui_panel_width = 280
        ui_x = self.renderer.width - self.ui_panel_width + 20
        slider_width = 240

        # Sliders
        self.slider_balls = Slider(ui_x, 80, slider_width, 3, 15, 5, "Number of Balls", step=1)
        self.slider_length = Slider(ui_x, 140, slider_width, 2.0, 10.0, 5.0, "Rod Length (m)", step=0.5)
        self.slider_angle = Slider(ui_x, 200, slider_width, 0, 60, 30, "Initial Angle (°)", step=5)

        # Buttons (vertically stacked)
        button_width = 240
        button_height = 40
        self.play_button = Button(ui_x, 270, button_width, button_height, "PLAY", color_scheme='green')
        self.pause_button = Button(ui_x, 320, button_width, button_height, "PAUSE", color_scheme='orange')
        self.reset_button = Button(ui_x, 370, button_width, button_height, "RESET", color_scheme='blue')

        # Initially disable pause button
        self.pause_button.enabled = False

        # Font for UI
        self.ui_font = pygame.font.Font(None, 24)
        self.ui_font_small = pygame.font.Font(None, 20)

    def setup(self):
        """Create Newton's cradle with current configuration"""
        # Clear previous setup
        self.balls = []
        self.constraints = []
        self.anchors = []

        # Clear physics world
        self.world.clear()
        self.world.clear_constraints()

        # Set gravity
        self.world.set_gravity(Vector2(0, -9.81))

        # Calculate spacing (balls exactly touching)
        ball_spacing = self.ball_radius * 2.0

        # Calculate starting positions (centered)
        total_width = ball_spacing * (self.num_balls - 1)
        start_x = -total_width / 2

        # Create each ball and its rod constraint
        for i in range(self.num_balls):
            # Create ball
            ball = RigidBody()
            collider = CircleCollider(self.ball_radius)
            ball.set_collider(collider)

            # Calculate mass and inertia
            density = 1.0
            mass = collider.calculate_mass(density)
            inertia = collider.calculate_inertia(mass)
            ball.set_mass(mass)
            ball.set_moment_of_inertia(inertia)

            # Position at rest
            x = start_x + i * ball_spacing
            y = self.anchor_y - self.rod_length

            # Apply initial angle to first ball (only if angle > 0)
            if i == 0 and self.initial_angle > 0:
                angle_rad = math.radians(self.initial_angle)
                # Calculate displaced position
                x = start_x - self.rod_length * math.sin(angle_rad)
                y = self.anchor_y - self.rod_length * math.cos(angle_rad)

            ball.set_position(Vector2(x, y))

            # Material properties
            ball.set_restitution(0.99)
            ball.set_friction(0.0)

            # Add to world
            self.world.add_body(ball)
            self.balls.append(ball)

            # Create rigid rod constraint
            anchor_x = start_x + i * ball_spacing
            anchor_point = Vector2(anchor_x, self.anchor_y)
            rod = DistanceConstraint(
                ball,
                Vector2(0, 0),
                anchor_point,
                self.rod_length,
                0.0001  # Small compliance for energy preservation
            )
            self.world.add_constraint(rod)
            self.constraints.append(rod)
            self.anchors.append(anchor_point)

        # Set camera
        self.renderer.camera.focus_on(Vector2(0, self.anchor_y - self.rod_length / 2))
        self.renderer.camera.pixels_per_meter = 50.0

        print(f"\n=== Newton's Cradle Configuration ===")
        print(f"Balls: {self.num_balls}")
        print(f"Rod length: {self.rod_length}m")
        print(f"Initial angle: {self.initial_angle}°")

    def on_key_press(self, key):
        """Handle additional key presses"""
        if key == pygame.K_RETURN:  # ENTER to play
            if not self.is_playing:
                self.start_simulation()
            else:
                self.paused = not self.paused
                print("▶ RESUMED" if not self.paused else "⏸ PAUSED")

    def start_simulation(self):
        """Start the simulation"""
        self.is_playing = True
        self.paused = False
        self.play_button.enabled = False
        self.pause_button.enabled = True
        self.setup()  # Rebuild with current settings
        print("▶ PLAYING - Simulation started!")

    def pause_simulation(self):
        """Pause/unpause the simulation"""
        self.paused = not self.paused
        if self.paused:
            self.pause_button.label = "RESUME"
            print("⏸ PAUSED")
        else:
            self.pause_button.label = "PAUSE"
            print("▶ RESUMED")

    def reset_simulation(self):
        """Reset to setup mode"""
        self.is_playing = False
        self.paused = True
        self.play_button.enabled = True
        self.pause_button.enabled = False
        self.pause_button.label = "PAUSE"
        self.setup()
        print("🔄 RESET - Back to setup mode")

    def handle_input(self):
        """Override input handling for UI elements"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                self.handle_key_press(event.key)

            # Handle UI events
            # Sliders (only in setup mode)
            if not self.is_playing:
                if self.slider_balls.handle_event(event):
                    self.num_balls = int(self.slider_balls.value)
                    self.setup()

                if self.slider_length.handle_event(event):
                    self.rod_length = self.slider_length.value
                    self.setup()

                if self.slider_angle.handle_event(event):
                    self.initial_angle = self.slider_angle.value
                    self.setup()

            # Buttons (available in different modes)
            if self.play_button.handle_event(event):
                self.start_simulation()

            if self.pause_button.handle_event(event):
                self.pause_simulation()

            if self.reset_button.handle_event(event):
                self.reset_simulation()

            # Handle camera controls
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click - start camera pan
                    # Check if click is not on UI panel
                    if event.pos[0] < self.renderer.width - self.ui_panel_width:
                        self.mouse_dragging = True
                        self.last_mouse_pos = event.pos

                elif event.button == 4:  # Scroll up - zoom in
                    self.renderer.camera.zoom(1.1, event.pos)

                elif event.button == 5:  # Scroll down - zoom out
                    self.renderer.camera.zoom(0.9, event.pos)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:  # Left click release
                    self.mouse_dragging = False
                    self.last_mouse_pos = None

            elif event.type == pygame.MOUSEMOTION:
                if self.mouse_dragging and self.last_mouse_pos:
                    # Calculate drag delta
                    dx = event.pos[0] - self.last_mouse_pos[0]
                    dy = event.pos[1] - self.last_mouse_pos[1]

                    # Pan camera (negative because we want to move opposite to drag)
                    self.renderer.camera.pan(-dx, -dy)

                    # Update last position
                    self.last_mouse_pos = event.pos

    def step_physics(self, dt):
        """Override physics step to handle pause"""
        if self.paused:
            return

        # Call parent's step_physics
        super().step_physics(dt)

    def render(self):
        """Override render to draw custom elements"""
        # Clear screen
        self.renderer.clear()

        # Draw grid
        if self.renderer.show_grid:
            self.renderer.draw_grid()

        # Draw constraints (rods and anchors)
        for i, anchor in enumerate(self.anchors):
            ball = self.balls[i]
            ball_pos = ball.get_position()

            # Convert to screen coordinates
            anchor_screen = self.renderer.camera.world_to_screen(anchor)
            ball_screen = self.renderer.camera.world_to_screen(ball_pos)

            # Draw rod
            pygame.draw.line(self.renderer.screen, (120, 120, 120),
                           anchor_screen, ball_screen, 2)

            # Draw anchor point
            pygame.draw.circle(self.renderer.screen, (255, 200, 100),
                             anchor_screen, 4)

        # Draw physics world (bodies)
        self.renderer.draw_world(self.world)

        # Draw stats
        self.draw_stats()

        # Draw UI controls
        self.draw_ui()

        # Draw status
        self.draw_status()

        if self.show_help:
            self.renderer.draw_help()

        # Show recording indicator
        if self.gif_recorder and self.gif_recorder.is_recording():
            self.renderer.draw_text("🎬 RECORDING", (10, 10),
                                   font='large', color='red')

        # Flip display
        self.renderer.flip()

        # Capture frame for GIF
        if self.gif_recorder:
            if not self.gif_recorder.capture(self.renderer.screen):
                # Recording finished
                self.gif_recorder.save()
                self.gif_recorder = None
                self.running = False  # Auto-quit after recording

    def draw_stats(self):
        """Draw statistics in top-left"""
        self.renderer.draw_text(f"FPS: {int(self.renderer.clock.get_fps())}", (10, 10), color='text')

    def draw_ui(self):
        """Draw UI controls on right panel"""
        # Draw semi-transparent background for right panel
        ui_bg = pygame.Surface((self.ui_panel_width, self.renderer.height))
        ui_bg.set_alpha(220)
        ui_bg.fill((25, 25, 25))
        self.renderer.screen.blit(ui_bg, (self.renderer.width - self.ui_panel_width, 0))

        # Draw title
        ui_x = self.renderer.width - self.ui_panel_width + 20
        title_surface = self.ui_font.render("Newton's Cradle", True, (255, 255, 100))
        self.renderer.screen.blit(title_surface, (ui_x + 40, 20))

        # Draw sliders (only editable in setup mode)
        self.slider_balls.draw(self.renderer.screen, self.ui_font_small)
        self.slider_length.draw(self.renderer.screen, self.ui_font_small)
        self.slider_angle.draw(self.renderer.screen, self.ui_font_small)

        # Draw buttons
        self.play_button.draw(self.renderer.screen, self.ui_font)
        self.pause_button.draw(self.renderer.screen, self.ui_font)
        self.reset_button.draw(self.renderer.screen, self.ui_font)

        # Draw energy display (if simulation is running)
        if self.balls:
            gravity_magnitude = abs(self.world.get_gravity().y)
            lowest_point = self.anchor_y - self.rod_length

            total_ke = 0.0
            total_pe = 0.0

            for ball in self.balls:
                total_ke += ball.get_kinetic_energy()
                mass = ball.get_mass()
                height = ball.get_position().y - lowest_point
                total_pe += mass * gravity_magnitude * height

            total_energy = total_ke + total_pe

            # Draw energy section
            energy_y = 450
            energy_title = self.ui_font.render("Energy", True, (200, 200, 255))
            self.renderer.screen.blit(energy_title, (ui_x + 80, energy_y))

            # Draw energy values
            energy_text = [
                f"Kinetic:    {total_ke:.2f} J",
                f"Potential:  {total_pe:.2f} J",
                f"Total:      {total_energy:.2f} J"
            ]

            for i, text in enumerate(energy_text):
                color = (255, 255, 100) if i == 2 else (220, 220, 220)
                text_surface = self.ui_font_small.render(text, True, color)
                self.renderer.screen.blit(text_surface, (ui_x, energy_y + 35 + i * 25))

        # Draw camera controls hint at bottom
        controls_y = self.renderer.height - 90
        controls_title = self.ui_font.render("Camera", True, (200, 200, 255))
        self.renderer.screen.blit(controls_title, (ui_x + 70, controls_y))

        camera_controls = [
            "Left-drag: Pan",
            "Scroll: Zoom"
        ]

        for i, text in enumerate(camera_controls):
            text_surface = self.ui_font_small.render(text, True, (180, 180, 180))
            self.renderer.screen.blit(text_surface, (ui_x, controls_y + 30 + i * 20))

    def draw_status(self):
        """Draw status message"""
        if not self.is_playing:
            status_text = "Configure parameters and click PLAY to start"
            color = 'highlight'
        elif self.paused:
            status_text = "PAUSED - Press SPACE to resume"
            color = 'highlight'
        else:
            status_text = "PLAYING"
            color = 'text'

        text_width = len(status_text) * 8  # Approximate
        self.renderer.draw_text(status_text,
                               (self.renderer.width // 2 - text_width // 2, 10),
                               font='large', color=color)


def main():
    sim = NewtonsCradle()
    sim.run()


if __name__ == "__main__":
    main()
