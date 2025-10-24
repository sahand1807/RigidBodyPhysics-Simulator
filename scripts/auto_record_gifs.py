#!/usr/bin/env python3
"""
Automatically record demo GIFs by capturing Pygame frames directly.

This script runs each demo, captures frames, and saves optimized GIFs.

Requirements:
    pip install pillow

Usage:
    python3 scripts/auto_record_gifs.py
"""

import sys
import os
import time
import subprocess
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

import pygame
from PIL import Image
import io


def capture_frames_from_pygame(demo_module, output_path, duration=10, fps=15, width=700):
    """
    Run a pygame demo and capture frames to create a GIF.

    Args:
        demo_module: Python module with run_demo() function
        output_path: Path to save GIF
        duration: Recording duration in seconds
        fps: Target FPS for GIF
        width: Target width (height auto-calculated)
    """
    print(f"🎬 Recording {output_path.name}...")
    print(f"   Duration: {duration}s @ {fps} FPS")

    frames = []
    target_frame_interval = 1.0 / fps
    last_capture_time = 0
    start_time = time.time()

    # Hook into Pygame to capture frames
    original_flip = pygame.display.flip

    def capture_flip():
        nonlocal last_capture_time
        original_flip()

        current_time = time.time() - start_time

        # Stop after duration
        if current_time >= duration:
            pygame.event.post(pygame.event.Event(pygame.QUIT))
            return

        # Capture frame at target FPS
        if current_time - last_capture_time >= target_frame_interval:
            # Get pygame surface
            screen = pygame.display.get_surface()

            # Convert to PIL Image
            # Pygame uses RGB, PIL expects RGB
            pygame_string = pygame.image.tostring(screen, 'RGB')
            pil_image = Image.frombytes('RGB', screen.get_size(), pygame_string)

            # Resize if needed
            if pil_image.width != width:
                aspect = pil_image.height / pil_image.width
                new_height = int(width * aspect)
                pil_image = pil_image.resize((width, new_height), Image.Resampling.LANCZOS)

            frames.append(pil_image)
            last_capture_time = current_time

            # Progress indicator
            if len(frames) % 15 == 0:
                print(f"   Captured {len(frames)} frames ({current_time:.1f}s / {duration}s)")

    # Replace pygame flip temporarily
    pygame.display.flip = capture_flip

    try:
        # Run the demo (this will call the modified flip)
        demo_module.main()
    except Exception as e:
        if "quit" not in str(e).lower():  # Normal quit is expected
            print(f"   Warning: {e}")
    finally:
        # Restore original flip
        pygame.display.flip = original_flip

    if not frames:
        print(f"   ❌ No frames captured!")
        return False

    print(f"   💾 Saving {len(frames)} frames to {output_path}...")

    # Save as optimized GIF
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Optimize: reduce colors to 256 using adaptive palette
    frames[0].save(
        output_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / fps),  # Duration per frame in ms
        loop=0,  # Infinite loop
        optimize=True
    )

    file_size = output_path.stat().st_size / (1024 * 1024)
    print(f"   ✅ Saved! ({file_size:.2f} MB, {len(frames)} frames)")

    return True


def record_demo_inline(demo_script, output_path, duration=10, fps=15, setup_func=None):
    """
    Record a demo by directly importing and running it.

    Args:
        demo_script: Path to demo script
        output_path: Output GIF path
        duration: Recording duration
        fps: GIF frame rate
        setup_func: Optional function to call before demo (for auto-configuration)
    """
    print(f"\n📹 Recording: {demo_script}")

    # Import the demo module
    import importlib.util
    spec = importlib.util.spec_from_file_location("demo", demo_script)
    demo_module = importlib.util.module_from_spec(spec)

    # Apply any setup before loading
    if setup_func:
        setup_func(demo_module)

    sys.modules['demo'] = demo_module
    spec.loader.exec_module(demo_module)

    # Capture and save
    success = capture_frames_from_pygame(demo_module, output_path, duration, fps)

    # Cleanup
    del sys.modules['demo']

    return success


def auto_configure_newtons_cradle():
    """Auto-start Newton's cradle demo"""
    def setup(module):
        # Patch to auto-start
        original_init = None
        if hasattr(module, 'NewtonsCradle'):
            original_init = module.NewtonsCradle.__init__

            def auto_init(self, *args, **kwargs):
                result = original_init(self, *args, **kwargs)
                # Auto-start with good settings
                self.num_balls_slider.value = 5
                self.initial_angle_slider.value = 30
                self.start_simulation()
                return result

            module.NewtonsCradle.__init__ = auto_init

    return setup


def main():
    """Record all demo GIFs automatically"""

    demos_to_record = [
        {
            'script': 'python/examples/newtons_cradle.py',
            'output': 'demos/newtons_cradle.gif',
            'duration': 12,
            'fps': 15,
            'setup': auto_configure_newtons_cradle()
        },
        {
            'script': 'python/examples/sandbox.py',
            'output': 'demos/sandbox.gif',
            'duration': 10,
            'fps': 15,
            'setup': None
        },
        {
            'script': 'python/examples/momentum_transfer.py',
            'output': 'demos/momentum_transfer.gif',
            'duration': 8,
            'fps': 15,
            'setup': None
        },
        {
            'script': 'python/examples/balls_chamber.py',
            'output': 'demos/balls_chamber.gif',
            'duration': 8,
            'fps': 15,
            'setup': None
        },
        {
            'script': 'python/examples/ball_pit.py',
            'output': 'demos/ball_pit.gif',
            'duration': 10,
            'fps': 15,
            'setup': None
        },
    ]

    print("=" * 60)
    print("  Automatic Demo GIF Recorder")
    print("=" * 60)
    print()
    print(f"Will record {len(demos_to_record)} demos...")
    print()

    results = []

    for i, demo in enumerate(demos_to_record, 1):
        print(f"\n[{i}/{len(demos_to_record)}] {demo['script']}")

        try:
            success = record_demo_inline(
                demo['script'],
                Path(demo['output']),
                duration=demo['duration'],
                fps=demo['fps'],
                setup_func=demo.get('setup')
            )
            results.append((demo['output'], success))
        except Exception as e:
            print(f"   ❌ Error: {e}")
            import traceback
            traceback.print_exc()
            results.append((demo['output'], False))

        # Small delay between recordings
        time.sleep(0.5)

    # Summary
    print("\n" + "=" * 60)
    print("  Recording Summary")
    print("=" * 60)

    for output_path, success in results:
        status = "✅" if success else "❌"
        print(f"{status} {output_path}")

    successful = sum(1 for _, s in results if s)
    print(f"\n{successful}/{len(results)} demos recorded successfully!")

    if successful < len(results):
        print("\nNote: Some demos may need to be recorded manually.")
        print("See demos/README.md for manual recording instructions.")


if __name__ == '__main__':
    main()
