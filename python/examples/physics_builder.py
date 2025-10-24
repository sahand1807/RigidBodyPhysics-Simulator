#!/usr/bin/env python3
"""
Interactive Physics Builder

Build your own physics scenes with full control over objects, gravity, and initial conditions.
Click and drag to create objects, configure properties, and run simulations.
"""

import pygame
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from physics_viz import (
    Vector2, RigidBody, CircleCollider, BoxCollider,
    PhysicsWorld, Camera, Renderer
)
from physics_viz.physics_engine_core import ColliderType
import random
import math


class Button:
    """Simple button UI element"""
    def __init__(self, x, y, width, height, text, color=(100, 100, 100)):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = tuple(min(c + 30, 255) for c in color)
        self.active_color = tuple(min(c + 60, 255) for c in color)
        self.is_hovered = False
        self.is_active = False

    def draw(self, screen, font):
        """Draw button"""
        color = self.active_color if self.is_active else (self.hover_color if self.is_hovered else self.color)
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, (255, 255, 255), self.rect, 2)

        text_surface = font.render(self.text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def handle_event(self, event):
        """Handle mouse events"""
        if event.type == pygame.MOUSEMOTION:
            self.is_hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                return True
        return False


class InputField:
    """Text input field"""
    def __init__(self, x, y, width, height, default_text="", numeric=True):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = default_text
        self.numeric = numeric
        self.active = False
        self.cursor_visible = True
        self.cursor_timer = 0

    def draw(self, screen, font):
        """Draw input field"""
        color = (80, 80, 120) if self.active else (60, 60, 60)
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, (255, 255, 255) if self.active else (150, 150, 150), self.rect, 2)

        text_surface = font.render(self.text, True, (255, 255, 255))
        screen.blit(text_surface, (self.rect.x + 5, self.rect.y + 5))

        # Draw cursor
        if self.active and self.cursor_visible:
            cursor_x = self.rect.x + 5 + text_surface.get_width()
            pygame.draw.line(screen, (255, 255, 255),
                           (cursor_x, self.rect.y + 5),
                           (cursor_x, self.rect.y + self.rect.height - 5), 2)

    def handle_event(self, event):
        """Handle input events"""
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.active = self.rect.collidepoint(event.pos)

        elif event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_RETURN:
                self.active = False
            elif event.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            else:
                char = event.unicode
                if self.numeric:
                    # Allow digits, minus, and decimal point
                    if char.isdigit() or char in '.-':
                        self.text += char
                else:
                    self.text += char

    def get_value(self):
        """Get numeric value"""
        try:
            return float(self.text) if self.text else 0.0
        except ValueError:
            return 0.0

    def update(self, dt):
        """Update cursor blink"""
        self.cursor_timer += dt
        if self.cursor_timer > 0.5:
            self.cursor_visible = not self.cursor_visible
            self.cursor_timer = 0


class PhysicsBuilder:
    """Interactive physics scene builder"""

    def __init__(self):
        pygame.init()

        # Window setup
        self.width = 1600
        self.height = 900
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Interactive Physics Builder")

        # UI panel dimensions
        self.panel_width = 350
        self.panel_x = self.width - self.panel_width

        # Renderer and camera for simulation view
        self.camera = Camera(self.panel_x, self.height, pixels_per_meter=20.0)
        self.camera.focus_on(Vector2(0, 0))

        # Physics world
        self.world = PhysicsWorld()
        self.gravity_enabled = True
        self.world.set_gravity(Vector2(0, -9.81))

        # Simulation state
        self.running = True
        self.simulating = False
        self.paused = False

        # Fonts
        self.font_large = pygame.font.Font(None, 32)
        self.font_medium = pygame.font.Font(None, 24)
        self.font_small = pygame.font.Font(None, 20)

        # Timing
        self.clock = pygame.time.Clock()
        self.fixed_dt = 1.0 / 60.0
        self.accumulator = 0.0

        # UI state
        self.selected_shape = 'circle'  # 'circle' or 'box'

        # Create UI elements
        self.create_ui()

        # Object creation
        self.preview_object = None
        self.mouse_world_pos = Vector2(0, 0)

        # Store initial states for reset functionality
        self.initial_states = []  # List of (body, pos, vel, ang_vel, rotation)

    def create_ui(self):
        """Create all UI elements"""
        panel_x = self.panel_x
        y_offset = 20
        spacing = 10

        # Title
        self.title_y = y_offset
        y_offset += 50

        # Shape selection buttons
        btn_width = 150
        btn_height = 40
        self.btn_circle = Button(panel_x + 20, y_offset, btn_width, btn_height, "Circle", (60, 120, 180))
        self.btn_box = Button(panel_x + 20 + btn_width + spacing, y_offset, btn_width, btn_height, "Box", (60, 120, 180))
        self.btn_circle.is_active = True
        y_offset += btn_height + spacing * 2

        # Shape parameters
        input_width = 80
        input_height = 30
        label_x = panel_x + 20
        input_x = panel_x + 150

        # Circle radius
        self.label_radius_y = y_offset
        self.input_radius = InputField(input_x, y_offset, input_width, input_height, "1.0")
        y_offset += input_height + spacing

        # Box width
        self.label_width_y = y_offset
        self.input_width = InputField(input_x, y_offset, input_width, input_height, "100.0")
        y_offset += input_height + spacing

        # Box height
        self.label_height_y = y_offset
        self.input_height = InputField(input_x, y_offset, input_width, input_height, "2.0")
        y_offset += input_height + spacing * 2

        # Position
        self.label_pos_y = y_offset
        y_offset += 25
        self.label_pos_x_y = y_offset
        self.input_pos_x = InputField(input_x, y_offset, input_width, input_height, "0.0")
        y_offset += input_height + spacing
        self.label_pos_y_y = y_offset
        self.input_pos_y = InputField(input_x, y_offset, input_width, input_height, "5.0")
        y_offset += input_height + spacing * 2

        # Initial velocity
        self.label_vel_y = y_offset
        y_offset += 25
        self.label_vel_x_y = y_offset
        self.input_vel_x = InputField(input_x, y_offset, input_width, input_height, "0.0")
        y_offset += input_height + spacing
        self.label_vel_y_y = y_offset
        self.input_vel_y = InputField(input_x, y_offset, input_width, input_height, "0.0")
        y_offset += input_height + spacing * 2

        # Physical properties
        self.label_mass_y = y_offset
        self.input_mass = InputField(input_x, y_offset, input_width, input_height, "1.0")
        y_offset += input_height + spacing * 2

        # Hardcoded material properties (not exposed in UI)
        # Balanced restitution: bouncy enough to prevent quick settling, but safe at all widths
        self.default_restitution = 0.9  # Good bounce, no energy gain

        # Static checkbox
        self.label_static_y = y_offset
        self.is_static = False
        self.static_checkbox = pygame.Rect(input_x, y_offset, 25, 25)
        y_offset += 35

        # Add object button
        self.btn_add = Button(panel_x + 20, y_offset, self.panel_width - 40, 50,
                             "Add Object", (60, 180, 120))
        y_offset += 60

        # Gravity toggle
        self.btn_gravity = Button(panel_x + 20, y_offset, self.panel_width - 40, 40,
                                 "Gravity: ON", (180, 100, 60))
        self.btn_gravity.is_active = True
        y_offset += 50

        # Simulation controls
        self.btn_run = Button(panel_x + 20, y_offset, 150, 50, "RUN", (60, 180, 60))
        self.btn_pause = Button(panel_x + 180, y_offset, 140, 50, "PAUSE", (180, 180, 60))
        y_offset += 60

        self.btn_reset = Button(panel_x + 20, y_offset, 150, 50, "RESET", (180, 60, 60))
        self.btn_clear = Button(panel_x + 180, y_offset, 140, 50, "CLEAR", (180, 100, 100))

        # Input fields list for easy iteration
        self.input_fields = [
            self.input_radius, self.input_width, self.input_height,
            self.input_pos_x, self.input_pos_y,
            self.input_vel_x, self.input_vel_y,
            self.input_mass
        ]

    def handle_input(self):
        """Process input events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused

            # Handle button clicks
            if self.btn_circle.handle_event(event):
                self.selected_shape = 'circle'
                self.btn_circle.is_active = True
                self.btn_box.is_active = False

            if self.btn_box.handle_event(event):
                self.selected_shape = 'box'
                self.btn_box.is_active = True
                self.btn_circle.is_active = False

            if self.btn_add.handle_event(event):
                self.add_object()

            if self.btn_gravity.handle_event(event):
                self.gravity_enabled = not self.gravity_enabled
                if self.gravity_enabled:
                    self.world.set_gravity(Vector2(0, -9.81))
                    self.btn_gravity.text = "Gravity: ON"
                    self.btn_gravity.is_active = True
                else:
                    self.world.set_gravity(Vector2(0, 0))
                    self.btn_gravity.text = "Gravity: OFF"
                    self.btn_gravity.is_active = False

            if self.btn_run.handle_event(event):
                self.simulating = True

            if self.btn_pause.handle_event(event):
                self.paused = not self.paused

            if self.btn_reset.handle_event(event):
                self.reset_simulation()

            if self.btn_clear.handle_event(event):
                self.world.clear()
                self.initial_states.clear()
                self.simulating = False

            # Static checkbox
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.static_checkbox.collidepoint(event.pos):
                    self.is_static = not self.is_static

                # Mouse wheel zoom (only in simulation area)
                mouse_x, mouse_y = event.pos
                if mouse_x < self.panel_x:
                    if event.button == 4:  # Scroll up - zoom in
                        self.camera.zoom(1.1, (mouse_x, mouse_y))
                    elif event.button == 5:  # Scroll down - zoom out
                        self.camera.zoom(0.9, (mouse_x, mouse_y))

            # Handle input fields
            for field in self.input_fields:
                field.handle_event(event)

            # Mouse position tracking
            if event.type == pygame.MOUSEMOTION:
                mouse_x, mouse_y = event.pos
                if mouse_x < self.panel_x:  # Only in simulation area
                    self.mouse_world_pos = self.camera.screen_to_world((mouse_x, mouse_y))

    def add_object(self):
        """Add object to the world based on current UI settings"""
        body = RigidBody()

        # Create collider based on shape
        if self.selected_shape == 'circle':
            radius = max(0.1, self.input_radius.get_value())
            collider = CircleCollider(radius)
        else:  # box
            width = max(0.1, self.input_width.get_value())
            height = max(0.1, self.input_height.get_value())
            collider = BoxCollider(width, height)

        body.set_collider(collider)

        # Set position
        pos_x = self.input_pos_x.get_value()
        pos_y = self.input_pos_y.get_value()
        body.set_position(Vector2(pos_x, pos_y))

        # Set mass and inertia
        if self.is_static:
            body.set_static()
        else:
            mass = max(0.1, self.input_mass.get_value())
            inertia = collider.calculate_inertia(mass)
            body.set_mass(mass)
            body.set_moment_of_inertia(inertia)

        # Set velocity
        vel_x = self.input_vel_x.get_value()
        vel_y = self.input_vel_y.get_value()
        body.set_velocity(Vector2(vel_x, vel_y))

        # Set material properties (hardcoded)
        body.set_restitution(self.default_restitution)
        # Don't set friction to avoid C++ friction bug that causes energy gain

        # Add to world
        self.world.add_body(body)

        # Store initial state for reset
        initial_pos = Vector2(pos_x, pos_y)
        initial_vel = Vector2(vel_x, vel_y)
        initial_ang_vel = 0.0
        initial_rotation = 0.0
        self.initial_states.append((body, initial_pos, initial_vel, initial_ang_vel, initial_rotation))

    def reset_simulation(self):
        """Reset simulation to initial state"""
        # Restore all objects to their initial states
        for body, initial_pos, initial_vel, initial_ang_vel, initial_rotation in self.initial_states:
            body.set_position(initial_pos)
            body.set_velocity(initial_vel)
            body.set_angular_velocity(initial_ang_vel)
            body.set_rotation(initial_rotation)

        self.simulating = False
        self.paused = False

    def update(self, dt):
        """Update simulation"""
        # Update input field cursors
        for field in self.input_fields:
            field.update(dt)

        # Update physics if simulating
        if self.simulating and not self.paused:
            self.accumulator += dt
            while self.accumulator >= self.fixed_dt:
                self.world.step(self.fixed_dt)
                self.accumulator -= self.fixed_dt

    def draw_simulation_view(self):
        """Draw the physics simulation area"""
        # Background
        sim_rect = pygame.Rect(0, 0, self.panel_x, self.height)
        pygame.draw.rect(self.screen, (20, 20, 30), sim_rect)

        # Draw grid
        self.draw_grid()

        # Draw all bodies
        for body in self.world.get_bodies():
            self.draw_body(body)

        # Draw border
        pygame.draw.rect(self.screen, (100, 100, 100), sim_rect, 2)

        # Draw status
        status = "SIMULATING" if self.simulating else "PAUSED" if self.paused else "BUILDING"
        status_color = (60, 180, 60) if self.simulating else (180, 180, 60) if self.paused else (180, 180, 180)
        status_text = self.font_large.render(status, True, status_color)
        self.screen.blit(status_text, (20, 20))

        # Draw object count
        count_text = self.font_medium.render(f"Objects: {self.world.get_body_count()}", True, (200, 200, 200))
        self.screen.blit(count_text, (20, 60))

    def draw_grid(self):
        """Draw grid in simulation view"""
        grid_color = (40, 40, 50)

        # Vertical lines
        for x in range(-50, 51, 5):
            screen_x, screen_y_top = self.camera.world_to_screen(Vector2(x, 50))
            _, screen_y_bottom = self.camera.world_to_screen(Vector2(x, -50))
            if 0 <= screen_x < self.panel_x:
                pygame.draw.line(self.screen, grid_color,
                               (screen_x, 0), (screen_x, self.height), 1)

        # Horizontal lines
        for y in range(-50, 51, 5):
            screen_x_left, screen_y = self.camera.world_to_screen(Vector2(-50, y))
            screen_x_right, _ = self.camera.world_to_screen(Vector2(50, y))
            if 0 <= screen_y < self.height:
                pygame.draw.line(self.screen, grid_color,
                               (0, screen_y), (self.panel_x, screen_y), 1)

        # Draw axes
        # X-axis (red)
        origin_screen = self.camera.world_to_screen(Vector2(0, 0))
        right_screen = self.camera.world_to_screen(Vector2(100, 0))
        pygame.draw.line(self.screen, (180, 60, 60), origin_screen, right_screen, 2)

        # Y-axis (green)
        top_screen = self.camera.world_to_screen(Vector2(0, 100))
        pygame.draw.line(self.screen, (60, 180, 60), origin_screen, top_screen, 2)

    def draw_body(self, body):
        """Draw a rigid body"""
        collider = body.get_collider()
        pos = body.get_position()

        # Choose color
        if body.is_static():
            color = (100, 100, 100)
        else:
            color = (100, 150, 255)

        if collider.get_type() == ColliderType.Circle:
            radius = collider.get_radius()
            screen_pos = self.camera.world_to_screen(pos)
            screen_radius = int(radius * self.camera.pixels_per_meter)
            pygame.draw.circle(self.screen, color, screen_pos, screen_radius)
            pygame.draw.circle(self.screen, (255, 255, 255), screen_pos, screen_radius, 2)

            # Draw rotation indicator
            angle = body.get_rotation()
            end_x = pos.x + radius * math.cos(angle)
            end_y = pos.y + radius * math.sin(angle)
            end_screen = self.camera.world_to_screen(Vector2(end_x, end_y))
            pygame.draw.line(self.screen, (255, 255, 255), screen_pos, end_screen, 2)

        else:  # Box
            half_w = collider.get_half_width()
            half_h = collider.get_half_height()
            rotation = body.get_rotation()

            # Calculate corners
            local_corners = [
                Vector2(-half_w, -half_h),
                Vector2(half_w, -half_h),
                Vector2(half_w, half_h),
                Vector2(-half_w, half_h),
            ]

            screen_corners = []
            cos_r = math.cos(rotation)
            sin_r = math.sin(rotation)

            for local_corner in local_corners:
                rotated_x = local_corner.x * cos_r - local_corner.y * sin_r
                rotated_y = local_corner.x * sin_r + local_corner.y * cos_r
                world_corner = Vector2(pos.x + rotated_x, pos.y + rotated_y)
                screen_corner = self.camera.world_to_screen(world_corner)
                screen_corners.append(screen_corner)

            pygame.draw.polygon(self.screen, color, screen_corners)
            pygame.draw.polygon(self.screen, (255, 255, 255), screen_corners, 2)

    def draw_ui_panel(self):
        """Draw UI control panel"""
        # Panel background
        panel_rect = pygame.Rect(self.panel_x, 0, self.panel_width, self.height)
        pygame.draw.rect(self.screen, (40, 40, 50), panel_rect)
        pygame.draw.rect(self.screen, (100, 100, 100), panel_rect, 2)

        # Title
        title = self.font_large.render("Physics Builder", True, (255, 255, 255))
        self.screen.blit(title, (self.panel_x + 20, self.title_y))

        # Shape selection buttons
        self.btn_circle.draw(self.screen, self.font_medium)
        self.btn_box.draw(self.screen, self.font_medium)

        # Labels and input fields
        label_x = self.panel_x + 20

        # Shape parameters
        if self.selected_shape == 'circle':
            label = self.font_small.render("Radius (m):", True, (200, 200, 200))
            self.screen.blit(label, (label_x, self.label_radius_y + 5))
            self.input_radius.draw(self.screen, self.font_small)
        else:
            label = self.font_small.render("Width (m):", True, (200, 200, 200))
            self.screen.blit(label, (label_x, self.label_width_y + 5))
            self.input_width.draw(self.screen, self.font_small)

            label = self.font_small.render("Height (m):", True, (200, 200, 200))
            self.screen.blit(label, (label_x, self.label_height_y + 5))
            self.input_height.draw(self.screen, self.font_small)

        # Position
        header = self.font_medium.render("Position", True, (255, 200, 100))
        self.screen.blit(header, (label_x, self.label_pos_y))

        label = self.font_small.render("X (m):", True, (200, 200, 200))
        self.screen.blit(label, (label_x, self.label_pos_x_y + 5))
        self.input_pos_x.draw(self.screen, self.font_small)

        label = self.font_small.render("Y (m):", True, (200, 200, 200))
        self.screen.blit(label, (label_x, self.label_pos_y_y + 5))
        self.input_pos_y.draw(self.screen, self.font_small)

        # Velocity
        header = self.font_medium.render("Initial Velocity", True, (255, 200, 100))
        self.screen.blit(header, (label_x, self.label_vel_y))

        label = self.font_small.render("Vx (m/s):", True, (200, 200, 200))
        self.screen.blit(label, (label_x, self.label_vel_x_y + 5))
        self.input_vel_x.draw(self.screen, self.font_small)

        label = self.font_small.render("Vy (m/s):", True, (200, 200, 200))
        self.screen.blit(label, (label_x, self.label_vel_y_y + 5))
        self.input_vel_y.draw(self.screen, self.font_small)

        # Physical properties
        label = self.font_small.render("Mass (kg):", True, (200, 200, 200))
        self.screen.blit(label, (label_x, self.label_mass_y + 5))
        self.input_mass.draw(self.screen, self.font_small)

        # Material properties: restitution=0.9 (bouncy), friction not set

        # Static checkbox
        label = self.font_small.render("Static:", True, (200, 200, 200))
        self.screen.blit(label, (label_x, self.label_static_y + 5))

        checkbox_color = (60, 180, 60) if self.is_static else (60, 60, 60)
        pygame.draw.rect(self.screen, checkbox_color, self.static_checkbox)
        pygame.draw.rect(self.screen, (255, 255, 255), self.static_checkbox, 2)
        if self.is_static:
            pygame.draw.line(self.screen, (255, 255, 255),
                           (self.static_checkbox.x + 5, self.static_checkbox.y + 12),
                           (self.static_checkbox.x + 10, self.static_checkbox.y + 18), 3)
            pygame.draw.line(self.screen, (255, 255, 255),
                           (self.static_checkbox.x + 10, self.static_checkbox.y + 18),
                           (self.static_checkbox.x + 20, self.static_checkbox.y + 5), 3)

        # Buttons
        self.btn_add.draw(self.screen, self.font_medium)
        self.btn_gravity.draw(self.screen, self.font_medium)
        self.btn_run.draw(self.screen, self.font_medium)
        self.btn_pause.draw(self.screen, self.font_medium)
        self.btn_reset.draw(self.screen, self.font_medium)
        self.btn_clear.draw(self.screen, self.font_medium)

    def render(self):
        """Render everything"""
        self.screen.fill((0, 0, 0))

        self.draw_simulation_view()
        self.draw_ui_panel()

        pygame.display.flip()

    def run(self):
        """Main loop"""
        while self.running:
            dt = self.clock.tick(60) / 1000.0  # Convert to seconds

            self.handle_input()
            self.update(dt)
            self.render()

        pygame.quit()


def main():
    builder = PhysicsBuilder()
    builder.run()


if __name__ == "__main__":
    main()
