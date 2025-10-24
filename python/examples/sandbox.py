#!/usr/bin/env python3
"""
Interactive Physics Sandbox

Left Click: Spawn random ball
Right Click: (future) Apply impulse
Space: Pause/Resume
R: Reset
G: Toggle grid
V: Toggle velocity vectors
H: Toggle help
ESC: Quit
"""

from physics_viz import InteractiveSandbox

def main():
    sandbox = InteractiveSandbox()
    sandbox.run()

if __name__ == "__main__":
    main()
