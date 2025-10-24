#!/usr/bin/env python3
"""
Script to record Pygame window as GIF for demos.

Requirements:
    pip install pillow

Usage:
    python3 scripts/record_demo.py python/examples/newtons_cradle.py demos/newtons_cradle.gif

    Or run a specific demo for a set duration:
    python3 scripts/record_demo.py python/examples/sandbox.py demos/sandbox.gif --duration 10
"""

import sys
import subprocess
import time
import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description='Record a Pygame demo as GIF')
    parser.add_argument('demo_script', help='Path to the demo Python script')
    parser.add_argument('output_gif', help='Output GIF file path')
    parser.add_argument('--duration', type=int, default=8, help='Recording duration in seconds (default: 8)')
    parser.add_argument('--fps', type=int, default=15, help='GIF frame rate (default: 15)')

    args = parser.parse_args()

    print(f"📹 Recording Demo GIF")
    print(f"   Demo: {args.demo_script}")
    print(f"   Output: {args.output_gif}")
    print(f"   Duration: {args.duration}s @ {args.fps} FPS")
    print()
    print("⚠️  IMPORTANT: This script requires manual screen recording.")
    print()
    print("🎬 RECOMMENDED APPROACH:")
    print()
    print("Option 1: Using SimpleScreenRecorder (Linux)")
    print("  1. Install: sudo apt-get install simplescreenrecorder")
    print("  2. Run your demo: python3", args.demo_script)
    print("  3. Use SimpleScreenRecorder to capture the Pygame window")
    print("  4. Export as video, then convert to GIF:")
    print(f"     ffmpeg -i recording.mp4 -vf 'fps={args.fps},scale=700:-1:flags=lanczos' -loop 0 {args.output_gif}")
    print()
    print("Option 2: Using FFmpeg with X11 (Linux)")
    print("  1. Run your demo: python3", args.demo_script)
    print("  2. Find window ID: xwininfo (click on Pygame window)")
    print("  3. Record with ffmpeg:")
    print(f"     ffmpeg -f x11grab -video_size 800x600 -i :0.0+100,200 -t {args.duration} -vf 'fps={args.fps},scale=700:-1:flags=lanczos' {args.output_gif}")
    print()
    print("Option 3: Using QuickTime (macOS)")
    print("  1. Run your demo: python3", args.demo_script)
    print("  2. QuickTime Player → File → New Screen Recording")
    print("  3. Select the Pygame window area")
    print("  4. Stop after a few seconds, save as .mov")
    print("  5. Convert to GIF:")
    print(f"     ffmpeg -i recording.mov -vf 'fps={args.fps},scale=700:-1:flags=lanczos' -loop 0 {args.output_gif}")
    print()
    print("Option 4: Using OBS Studio (Cross-platform)")
    print("  1. Install OBS Studio")
    print("  2. Run your demo: python3", args.demo_script)
    print("  3. Add 'Window Capture' source, select Pygame window")
    print("  4. Record for a few seconds")
    print("  5. Convert to GIF:")
    print(f"     ffmpeg -i recording.mkv -vf 'fps={args.fps},scale=700:-1:flags=lanczos' -loop 0 {args.output_gif}")
    print()
    print("💡 TIP: Keep GIFs under 5MB for fast loading on GitHub")
    print()

if __name__ == '__main__':
    main()
