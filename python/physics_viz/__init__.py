"""
Physics Visualization Package

This package provides a Python interface to the C++ physics engine,
along with visualization tools using Pygame.
"""

# Import C++ core module
try:
    from .physics_engine_core import (
        # Math
        Vector2,
        Transform,

        # Colliders
        Collider,
        ColliderType,
        CircleCollider,

        # Physics
        RigidBody,
        PhysicsWorld,
    )
except ImportError as e:
    # If C++ module not built yet, provide helpful error
    import sys
    print(f"Error importing C++ physics engine: {e}", file=sys.stderr)
    print("Please build the package with: pip install -e .", file=sys.stderr)
    raise

# Package version
__version__ = "0.1.0"

# Export all public classes
__all__ = [
    # Math
    'Vector2',
    'Transform',

    # Colliders
    'Collider',
    'ColliderType',
    'CircleCollider',

    # Physics
    'RigidBody',
    'PhysicsWorld',
]
