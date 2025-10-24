"""
GIF Recorder for Pygame simulations.

Usage in a demo:
    from physics_viz.gif_recorder import GifRecorder

    recorder = GifRecorder("output.gif", duration=10, fps=15)

    # In your main loop:
    for frame in range(total_frames):
        # ... render your simulation ...
        pygame.display.flip()
        recorder.capture(screen)  # Capture frame

    recorder.save()  # Save GIF when done
"""

import pygame
from PIL import Image
import time
from pathlib import Path


class GifRecorder:
    """Records Pygame screen to animated GIF."""

    def __init__(self, output_path, duration=10, fps=15, target_width=700):
        """
        Initialize GIF recorder.

        Args:
            output_path: Path to save GIF file
            duration: Maximum recording duration in seconds
            fps: Target frames per second for GIF
            target_width: Target width in pixels (height auto-scaled)
        """
        self.output_path = Path(output_path)
        self.duration = duration
        self.fps = fps
        self.target_width = target_width

        self.frames = []
        self.start_time = None
        self.last_capture_time = 0
        self.frame_interval = 1.0 / fps
        self.recording = True

    def start(self):
        """Start recording timer."""
        self.start_time = time.time()
        self.recording = True
        print(f"🎬 Recording started: {self.output_path.name}")
        print(f"   Duration: {self.duration}s @ {self.fps} FPS")

    def capture(self, screen):
        """
        Capture current frame from Pygame screen.

        Args:
            screen: Pygame surface to capture

        Returns:
            bool: True if still recording, False if duration exceeded
        """
        if not self.recording:
            return False

        if self.start_time is None:
            self.start()

        current_time = time.time() - self.start_time

        # Check if recording duration exceeded
        if current_time >= self.duration:
            self.recording = False
            return False

        # Capture at target FPS interval
        if current_time - self.last_capture_time >= self.frame_interval:
            # Convert Pygame surface to PIL Image
            pygame_string = pygame.image.tostring(screen, 'RGB')
            pil_image = Image.frombytes('RGB', screen.get_size(), pygame_string)

            # Resize if needed
            if pil_image.width != self.target_width:
                aspect = pil_image.height / pil_image.width
                new_height = int(self.target_width * aspect)
                pil_image = pil_image.resize(
                    (self.target_width, new_height),
                    Image.Resampling.LANCZOS
                )

            self.frames.append(pil_image)
            self.last_capture_time = current_time

            # Progress indicator
            if len(self.frames) % self.fps == 0:  # Every second
                print(f"   📸 {len(self.frames)} frames ({current_time:.1f}s)")

        return True

    def save(self):
        """Save captured frames as GIF."""
        if not self.frames:
            print("   ⚠️  No frames captured, nothing to save")
            return False

        print(f"   💾 Saving {len(self.frames)} frames...")

        # Create output directory if needed
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        # Save as optimized GIF
        self.frames[0].save(
            self.output_path,
            save_all=True,
            append_images=self.frames[1:],
            duration=int(1000 / self.fps),  # ms per frame
            loop=0,  # Infinite loop
            optimize=True
        )

        file_size = self.output_path.stat().st_size / (1024 * 1024)
        print(f"   ✅ Saved {self.output_path.name} ({file_size:.2f} MB)")

        return True

    def is_recording(self):
        """Check if still recording."""
        return self.recording
