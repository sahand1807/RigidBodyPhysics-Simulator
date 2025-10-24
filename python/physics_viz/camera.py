"""
Camera system for 2D physics visualization

Provides viewport transformation from world space to screen space.
"""

import pygame
from .physics_engine_core import Vector2


class Camera:
    """2D camera with pan and zoom support

    The camera transforms world coordinates (meters) to screen coordinates (pixels).
    Supports panning, zooming, and following objects.
    """

    def __init__(self, screen_width, screen_height, pixels_per_meter=20.0):
        """Initialize camera

        Args:
            screen_width: Screen width in pixels
            screen_height: Screen height in pixels
            pixels_per_meter: Zoom level (pixels per meter)
        """
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.pixels_per_meter = pixels_per_meter

        # Camera position in world space (meters)
        self.position = Vector2(0, 0)

        # Zoom limits
        self.min_zoom = 5.0
        self.max_zoom = 200.0

    def world_to_screen(self, world_pos):
        """Convert world position (meters) to screen position (pixels)

        Args:
            world_pos: Vector2 in world space

        Returns:
            tuple: (x, y) in screen space
        """
        # Translate relative to camera
        rel_x = world_pos.x - self.position.x
        rel_y = world_pos.y - self.position.y

        # Apply zoom and convert to pixels
        screen_x = rel_x * self.pixels_per_meter + self.screen_width / 2
        screen_y = -rel_y * self.pixels_per_meter + self.screen_height / 2  # Flip Y

        return (int(screen_x), int(screen_y))

    def screen_to_world(self, screen_pos):
        """Convert screen position (pixels) to world position (meters)

        Args:
            screen_pos: tuple (x, y) in screen space

        Returns:
            Vector2 in world space
        """
        screen_x, screen_y = screen_pos

        # Remove screen center offset and convert to meters
        rel_x = (screen_x - self.screen_width / 2) / self.pixels_per_meter
        rel_y = -(screen_y - self.screen_height / 2) / self.pixels_per_meter  # Flip Y

        # Add camera position
        world_x = rel_x + self.position.x
        world_y = rel_y + self.position.y

        return Vector2(world_x, world_y)

    def world_to_screen_scalar(self, world_scalar):
        """Convert world distance (meters) to screen distance (pixels)

        Args:
            world_scalar: Distance in meters

        Returns:
            int: Distance in pixels
        """
        return int(world_scalar * self.pixels_per_meter)

    def screen_to_world_scalar(self, screen_scalar):
        """Convert screen distance (pixels) to world distance (meters)

        Args:
            screen_scalar: Distance in pixels

        Returns:
            float: Distance in meters
        """
        return screen_scalar / self.pixels_per_meter

    def pan(self, dx, dy):
        """Pan camera by screen offset

        Args:
            dx: Horizontal pan in pixels
            dy: Vertical pan in pixels
        """
        world_dx = dx / self.pixels_per_meter
        world_dy = -dy / self.pixels_per_meter  # Flip Y
        self.position.x += world_dx
        self.position.y += world_dy

    def zoom(self, factor, center=None):
        """Zoom camera by factor

        Args:
            factor: Zoom multiplier (> 1 = zoom in, < 1 = zoom out)
            center: Optional screen position to zoom toward (tuple)
        """
        old_ppm = self.pixels_per_meter
        self.pixels_per_meter *= factor

        # Clamp zoom
        self.pixels_per_meter = max(self.min_zoom, min(self.max_zoom, self.pixels_per_meter))

        # If zooming toward a point, adjust position to keep that point fixed
        if center:
            # Convert center to world space using old zoom
            old_world = self.screen_to_world(center)
            # Update zoom
            # Convert center to world space using new zoom
            new_world = self.screen_to_world(center)
            # Offset camera to keep point fixed
            self.position.x += (old_world.x - new_world.x)
            self.position.y += (old_world.y - new_world.y)

    def focus_on(self, world_pos):
        """Center camera on world position

        Args:
            world_pos: Vector2 in world space
        """
        self.position.x = world_pos.x
        self.position.y = world_pos.y

    def get_view_bounds(self):
        """Get visible world bounds

        Returns:
            tuple: (min_x, max_x, min_y, max_y) in world space
        """
        # Get corners in world space
        top_left = self.screen_to_world((0, 0))
        bottom_right = self.screen_to_world((self.screen_width, self.screen_height))

        return (top_left.x, bottom_right.x, bottom_right.y, top_left.y)
