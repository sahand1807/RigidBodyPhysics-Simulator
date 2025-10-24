# Demo GIFs

This directory contains GIF recordings of the physics simulations for the README gallery.

**Note**: The GIFs have not been recorded yet. See below for instructions on how to create them.

## Creating Demo GIFs

### Quick Guide

1. **Run the demo**:
   ```bash
   python3 python/examples/newtons_cradle.py
   ```

2. **Record your screen** using one of these tools:
   - **Linux**: SimpleScreenRecorder, Peek, or FFmpeg
   - **macOS**: QuickTime Screen Recording or Kap
   - **Windows**: OBS Studio or ScreenToGif
   - **Cross-platform**: OBS Studio

3. **Convert to optimized GIF**:
   ```bash
   ffmpeg -i recording.mp4 \
          -vf "fps=15,scale=700:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
          -loop 0 \
          demos/newtons_cradle.gif
   ```

### Recommended Settings

- **Resolution**: 700-800px width (maintains aspect ratio)
- **FPS**: 15 frames per second (smooth but not too large)
- **Duration**: 6-10 seconds per demo
- **File size**: Under 5MB each (GitHub loads faster)

### Demos to Record

- [ ] `newtons_cradle.gif` - Newton's cradle with 5 balls, pull back 1-2 balls
- [ ] `physics_builder.gif` - Create a scene with mixed circles and boxes
- [ ] `momentum_transfer.gif` - Ball hitting a chain of stationary balls
- [ ] `balls_chamber.gif` - Chamber full of bouncing balls
- [ ] `ball_pit.gif` - Balls continuously spawning into container
- [ ] `sandbox.gif` - Click around to spawn balls interactively
- [ ] `bouncing_balls.gif` - (Optional) Simple bouncing demonstration

### Tools

#### Peek (Linux - Easiest)
```bash
sudo apt-get install peek
# GUI tool - just select area and record
```

#### FFmpeg (All platforms - Most control)
```bash
# Linux (X11 screen capture)
ffmpeg -video_size 800x600 -framerate 60 -f x11grab -i :0.0+100,200 -t 8 recording.mp4

# Then convert to GIF
ffmpeg -i recording.mp4 -vf "fps=15,scale=700:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 output.gif
```

#### Kap (macOS - Simple)
```bash
brew install --cask kap
# GUI tool with GIF export built-in
```

#### OBS Studio (Cross-platform)
```bash
# Install from https://obsproject.com/
# Add Window Capture source, record, export as MP4
# Then convert to GIF with ffmpeg (see above)
```

### Tips for Great Demo GIFs

1. **Clean window**: No overlapping windows or distractions
2. **Interesting action**: Show the most dynamic/impressive moment
3. **Loop seamlessly**: Try to end in a similar state to the beginning
4. **Optimize size**: Use palette generation for smaller, higher quality GIFs
5. **Test on GitHub**: Preview looks different than local, check after pushing

### Helper Script

Use the provided script for recording instructions:
```bash
python3 scripts/record_demo.py python/examples/newtons_cradle.py demos/newtons_cradle.gif
```

This will print platform-specific recording instructions.
