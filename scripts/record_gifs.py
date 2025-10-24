#!/usr/bin/env python3
"""
Record all demo GIFs automatically.
Simply run: python3 scripts/record_gifs.py
"""

import sys
from pathlib import Path

# Add parent to path
sys.path.insert(0, str(Path(__file__).parent.parent))

print("=" * 70)
print("  Recording Demo GIFs")
print("=" * 70)
print()

# Import and run each demo with GIF recording
demos = [
    {
        'name': 'Momentum Transfer',
        'module': 'python.examples.momentum_transfer',
        'class': 'MomentumTransfer',
        'output': 'demos/momentum_transfer.gif',
        'duration': 10
    },
    {
        'name': 'Balls Chamber',
        'module': 'python.examples.balls_chamber',
        'class': 'BallsChamber',
        'output': 'demos/balls_chamber.gif',
        'duration': 8
    },
    {
        'name': 'Ball Pit',
        'module': 'python.examples.ball_pit',
        'class': 'BallPit',
        'output': 'demos/ball_pit.gif',
        'duration': 10
    },
    {
        'name': 'Bouncing Balls',
        'module': 'python.examples.bouncing_balls',
        'class': 'BouncingBalls',
        'output': 'demos/bouncing_balls.gif',
        'duration': 8
    },
]

for i, demo in enumerate(demos, 1):
    print(f"\n[{i}/{len(demos)}] {demo['name']}")
    print("-" * 70)

    try:
        # Import the module
        module = __import__(demo['module'], fromlist=[demo['class']])
        SimClass = getattr(module, demo['class'])

        # Create instance and add GIF recorder
        sim = SimClass()

        # Import and set up GIF recorder
        from physics_viz.gif_recorder import GifRecorder
        sim.gif_recorder = GifRecorder(demo['output'], duration=demo['duration'], fps=15)

        # Run simulation (will auto-quit when recording finishes)
        sim.run()

        print(f"✅ {demo['name']} recorded successfully!\n")

    except Exception as e:
        print(f"❌ Error: {e}\n")
        import traceback
        traceback.print_exc()

print("\n" + "=" * 70)
print("  Done! GIFs saved to demos/ directory")
print("=" * 70)
print("\nTo commit: git add demos/*.gif && git commit -m 'Add demo GIFs' && git push")
