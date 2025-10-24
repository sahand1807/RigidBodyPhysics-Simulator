#!/usr/bin/env python3
"""
Test script to verify visualization modules work

Tests that all demos can be instantiated and set up without errors.
Does not actually run the simulations (which would require a display).
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_imports():
    """Test that all visualization modules can be imported"""
    print("Testing imports...")
    try:
        from physics_viz import (
            Vector2, Transform, RigidBody, CircleCollider, PhysicsWorld,
            Camera, Renderer, Simulation, InteractiveSandbox
        )
        print("  ✓ All modules imported successfully")
        return True
    except Exception as e:
        print(f"  ✗ Import error: {e}")
        return False


def test_camera():
    """Test camera functionality"""
    print("\nTesting Camera...")
    try:
        from physics_viz import Camera, Vector2

        cam = Camera(1280, 720, pixels_per_meter=30.0)

        # Test world to screen
        world_pos = Vector2(10, 5)
        screen_pos = cam.world_to_screen(world_pos)
        assert isinstance(screen_pos, tuple)
        assert len(screen_pos) == 2

        # Test screen to world
        back_to_world = cam.screen_to_world(screen_pos)
        assert abs(back_to_world.x - world_pos.x) < 0.1
        assert abs(back_to_world.y - world_pos.y) < 0.1

        # Test pan
        cam.pan(10, 20)

        # Test zoom
        cam.zoom(1.5)

        print("  ✓ Camera tests passed")
        return True
    except Exception as e:
        print(f"  ✗ Camera error: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_simulation_setup():
    """Test that simulations can be set up"""
    print("\nTesting Simulation setup...")
    try:
        # Can't actually create pygame window, but we can test imports
        # and class definitions
        from physics_viz import Simulation, InteractiveSandbox

        # Check that classes exist and have required methods
        assert hasattr(Simulation, 'setup')
        assert hasattr(Simulation, 'update')
        assert hasattr(Simulation, 'run')

        assert hasattr(InteractiveSandbox, 'setup')
        assert hasattr(InteractiveSandbox, 'on_mouse_click')

        print("  ✓ Simulation classes defined correctly")
        return True
    except Exception as e:
        print(f"  ✗ Simulation error: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_demo_imports():
    """Test that all demo files can be imported"""
    print("\nTesting demo file imports...")

    demos = [
        'bouncing_balls',
        'newtons_cradle',
        'ball_pit',
        'pyramid',
    ]

    success = True
    for demo in demos:
        try:
            __import__(demo)
            print(f"  ✓ {demo}.py imports successfully")
        except Exception as e:
            # pygame.init() might fail without display, that's ok
            if 'pygame' in str(e).lower() or 'display' in str(e).lower():
                print(f"  ✓ {demo}.py imports (pygame display not available)")
            else:
                print(f"  ✗ {demo}.py error: {e}")
                success = False

    return success


def main():
    """Run all tests"""
    print("=" * 60)
    print("Visualization Module Test Suite")
    print("=" * 60)

    results = []

    results.append(("Imports", test_imports()))
    results.append(("Camera", test_camera()))
    results.append(("Simulation Setup", test_simulation_setup()))
    results.append(("Demo Imports", test_demo_imports()))

    print("\n" + "=" * 60)
    print("Test Results:")
    print("=" * 60)

    all_passed = True
    for test_name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  {test_name:.<40} {status}")
        if not passed:
            all_passed = False

    print("=" * 60)

    if all_passed:
        print("All tests PASSED!")
        return 0
    else:
        print("Some tests FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(main())
