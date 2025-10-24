"""
Pygame renderer for 2D physics visualization

Renders rigid bodies, colliders, and debug information.
"""

import pygame
import math
from .physics_engine_core import Vector2, ColliderType
from .camera import Camera


class Renderer:
    """Pygame-based renderer for physics simulation

    Handles rendering of rigid bodies, colliders, forces, and debug info.
    Supports camera transformations and multiple rendering modes.
    """

    # Color scheme
    COLORS = {
        'background': (20, 20, 30),
        'grid': (40, 40, 50),
        'axis': (80, 80, 100),
        'static_body': (100, 100, 120),
        'dynamic_body': (100, 150, 255),
        'collider': (150, 200, 255),
        'velocity': (255, 100, 100),
        'force': (255, 200, 100),
        'text': (200, 200, 200),
        'highlight': (255, 255, 100),
    }

    def __init__(self, width=1280, height=720, title="2D Physics Simulator"):
        """Initialize renderer

        Args:
            width: Screen width in pixels
            height: Screen height in pixels
            title: Window title
        """
        pygame.init()
        self.width = width
        self.height = height

        # Create display
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption(title)

        # Create camera
        self.camera = Camera(width, height, pixels_per_meter=30.0)

        # Fonts
        self.font_small = pygame.font.Font(None, 24)
        self.font_large = pygame.font.Font(None, 36)

        # Rendering options
        self.show_grid = True
        self.show_axes = True
        self.show_velocity = True
        self.show_forces = False
        self.show_colliders = True
        self.show_stats = True

        # Clock for FPS
        self.clock = pygame.time.Clock()

    def clear(self):
        """Clear screen with background color"""
        self.screen.fill(self.COLORS['background'])

    def draw_grid(self, spacing=1.0):
        """Draw grid lines

        Args:
            spacing: Grid spacing in meters
        """
        if not self.show_grid:
            return

        min_x, max_x, min_y, max_y = self.camera.get_view_bounds()

        # Vertical lines
        start_x = math.floor(min_x / spacing) * spacing
        x = start_x
        while x <= max_x:
            screen_pos = self.camera.world_to_screen(Vector2(x, 0))
            color = self.COLORS['axis'] if x == 0 else self.COLORS['grid']
            width = 2 if x == 0 else 1
            pygame.draw.line(self.screen, color,
                           (screen_pos[0], 0),
                           (screen_pos[0], self.height),
                           width)
            x += spacing

        # Horizontal lines
        start_y = math.floor(min_y / spacing) * spacing
        y = start_y
        while y <= max_y:
            screen_pos = self.camera.world_to_screen(Vector2(0, y))
            color = self.COLORS['axis'] if y == 0 else self.COLORS['grid']
            width = 2 if y == 0 else 1
            pygame.draw.line(self.screen, color,
                           (0, screen_pos[1]),
                           (self.width, screen_pos[1]),
                           width)
            y += spacing

    def draw_circle_collider(self, body):
        """Draw a circle collider

        Args:
            body: RigidBody with CircleCollider
        """
        collider = body.get_collider()
        radius = collider.get_radius()

        # Get world position
        world_pos = body.get_position()
        screen_pos = self.camera.world_to_screen(world_pos)
        screen_radius = self.camera.world_to_screen_scalar(radius)

        # Skip if too small or off screen
        if screen_radius < 2:
            return

        # Choose color based on static/dynamic
        if body.is_static():
            color = self.COLORS['static_body']
        else:
            color = self.COLORS['dynamic_body']

        # Draw filled circle
        pygame.draw.circle(self.screen, color, screen_pos, screen_radius)

        # Draw outline
        if self.show_colliders:
            pygame.draw.circle(self.screen, self.COLORS['collider'],
                             screen_pos, screen_radius, 2)

        # Draw rotation indicator (line from center to edge)
        angle = body.get_rotation()
        end_x = world_pos.x + radius * math.cos(angle)
        end_y = world_pos.y + radius * math.sin(angle)
        end_screen = self.camera.world_to_screen(Vector2(end_x, end_y))
        pygame.draw.line(self.screen, (255, 255, 255),
                        screen_pos, end_screen, 2)

    def draw_box_collider(self, body):
        """Draw a box collider

        Args:
            body: RigidBody with BoxCollider
        """
        collider = body.get_collider()
        width = collider.get_width()
        height = collider.get_height()

        # Get world position and rotation
        world_pos = body.get_position()
        rotation = body.get_rotation()

        # Calculate the four corners in local space
        half_w = width / 2.0
        half_h = height / 2.0

        local_corners = [
            Vector2(-half_w, -half_h),  # Bottom-left
            Vector2(half_w, -half_h),   # Bottom-right
            Vector2(half_w, half_h),    # Top-right
            Vector2(-half_w, half_h),   # Top-left
        ]

        # Transform corners to world space and then to screen space
        screen_corners = []
        cos_r = math.cos(rotation)
        sin_r = math.sin(rotation)

        for local_corner in local_corners:
            # Rotate
            rotated_x = local_corner.x * cos_r - local_corner.y * sin_r
            rotated_y = local_corner.x * sin_r + local_corner.y * cos_r

            # Translate
            world_corner = Vector2(
                world_pos.x + rotated_x,
                world_pos.y + rotated_y
            )

            # Convert to screen space
            screen_corner = self.camera.world_to_screen(world_corner)
            screen_corners.append(screen_corner)

        # Skip if too small
        if len(screen_corners) < 3:
            return

        # Choose color based on static/dynamic
        if body.is_static():
            color = self.COLORS['static_body']
        else:
            color = self.COLORS['dynamic_body']

        # Draw filled polygon
        pygame.draw.polygon(self.screen, color, screen_corners)

        # Draw outline
        if self.show_colliders:
            pygame.draw.polygon(self.screen, self.COLORS['collider'],
                              screen_corners, 2)

        # Draw rotation indicator (line from center to right edge midpoint)
        center_screen = self.camera.world_to_screen(world_pos)
        indicator_end = Vector2(
            world_pos.x + half_w * cos_r,
            world_pos.y + half_w * sin_r
        )
        indicator_screen = self.camera.world_to_screen(indicator_end)
        pygame.draw.line(self.screen, (255, 255, 255),
                        center_screen, indicator_screen, 2)

    def draw_body(self, body):
        """Draw a rigid body

        Args:
            body: RigidBody to draw
        """
        if not body.has_collider():
            return

        collider = body.get_collider()
        collider_type = collider.get_type()

        if collider_type == ColliderType.Circle:
            self.draw_circle_collider(body)
        elif collider_type == ColliderType.Box:
            self.draw_box_collider(body)

    def draw_velocity_vector(self, body, scale=0.5):
        """Draw velocity vector

        Args:
            body: RigidBody
            scale: Vector scale factor
        """
        if not self.show_velocity:
            return

        vel = body.get_velocity()
        if vel.length() < 0.1:  # Skip if too small
            return

        pos = body.get_position()
        screen_start = self.camera.world_to_screen(pos)

        # Scale velocity for visibility
        end_pos = Vector2(pos.x + vel.x * scale, pos.y + vel.y * scale)
        screen_end = self.camera.world_to_screen(end_pos)

        # Draw arrow
        pygame.draw.line(self.screen, self.COLORS['velocity'],
                        screen_start, screen_end, 2)

        # Draw arrowhead
        self._draw_arrow_head(screen_start, screen_end, self.COLORS['velocity'])

    def _draw_arrow_head(self, start, end, color, size=10):
        """Draw arrow head at end of line

        Args:
            start: Start position (x, y)
            end: End position (x, y)
            color: Arrow color
            size: Arrowhead size in pixels
        """
        # Calculate direction
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx*dx + dy*dy)

        if length < 0.1:
            return

        # Normalize
        dx /= length
        dy /= length

        # Calculate arrowhead points
        angle = math.pi / 6  # 30 degrees
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        # Left point
        left_x = end[0] - size * (dx * cos_a - dy * sin_a)
        left_y = end[1] - size * (dy * cos_a + dx * sin_a)

        # Right point
        right_x = end[0] - size * (dx * cos_a + dy * sin_a)
        right_y = end[1] - size * (dy * cos_a - dx * sin_a)

        # Draw triangle
        pygame.draw.polygon(self.screen, color,
                          [end, (int(left_x), int(left_y)), (int(right_x), int(right_y))])

    def draw_world(self, world):
        """Draw entire physics world

        Args:
            world: PhysicsWorld to render
        """
        # Draw grid and axes
        self.draw_grid()

        # Draw all bodies
        bodies = world.get_bodies()
        for body in bodies:
            self.draw_body(body)
            self.draw_velocity_vector(body)

        # Draw stats
        if self.show_stats:
            self.draw_stats(world)

    def draw_stats(self, world):
        """Draw statistics overlay

        Args:
            world: PhysicsWorld
        """
        stats = [
            f"Bodies: {world.get_body_count()}",
            f"Zoom: {self.camera.pixels_per_meter:.1f} px/m",
            f"Camera: ({self.camera.position.x:.1f}, {self.camera.position.y:.1f})",
            f"FPS: {self.clock.get_fps():.0f}",
        ]

        y = 10
        for stat in stats:
            text = self.font_small.render(stat, True, self.COLORS['text'])
            self.screen.blit(text, (10, y))
            y += 25

    def draw_text(self, text, position, font='small', color='text'):
        """Draw text at position

        Args:
            text: Text string
            position: Screen position (x, y)
            font: 'small' or 'large'
            color: Color name from COLORS dict
        """
        font_obj = self.font_small if font == 'small' else self.font_large
        color_val = self.COLORS.get(color, self.COLORS['text'])
        surface = font_obj.render(text, True, color_val)
        self.screen.blit(surface, position)

    def draw_help(self):
        """Draw help overlay"""
        help_text = [
            "Controls:",
            "  Mouse Drag: Pan camera",
            "  Mouse Wheel: Zoom",
            "  Space: Pause/Resume",
            "  R: Reset simulation",
            "  G: Toggle grid",
            "  V: Toggle velocity vectors",
            "  H: Toggle this help",
            "  ESC: Quit",
        ]

        # Semi-transparent background
        overlay = pygame.Surface((300, 250))
        overlay.set_alpha(200)
        overlay.fill((20, 20, 30))
        self.screen.blit(overlay, (self.width - 320, 10))

        # Draw text
        y = 20
        for line in help_text:
            self.draw_text(line, (self.width - 310, y), font='small', color='text')
            y += 25

    def flip(self):
        """Update display"""
        pygame.display.flip()

    def tick(self, fps=60):
        """Limit frame rate

        Args:
            fps: Target frames per second

        Returns:
            float: Delta time in seconds
        """
        return self.clock.tick(fps) / 1000.0

    def quit(self):
        """Cleanup and quit"""
        pygame.quit()
