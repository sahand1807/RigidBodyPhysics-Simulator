# Newton's Cradle Interactive Example

## Overview

The Newton's Cradle example (`python/examples/newtons_cradle.py`) demonstrates:
- **Conservation of momentum** and **conservation of energy**
- **XPBD distance constraints** for rigid rods
- **Interactive GUI controls** with sliders and buttons
- **Real-time energy tracking** (kinetic, potential, total)

This is a complete, production-ready example showcasing the constraint system's capabilities.

## Running the Example

```bash
python3 python/examples/newtons_cradle.py
```

## Features

### Configuration Controls (Setup Mode)
- **Number of Balls slider**: 3-15 balls
- **Rod Length slider**: 2-10 meters
- **Initial Angle slider**: 0-60 degrees (pulls back first ball)

### Simulation Controls
- **PLAY button**: Start simulation with current configuration
- **PAUSE button**: Pause/resume during simulation
- **RESET button**: Return to setup mode to adjust parameters

### Camera Controls
- **Left-drag**: Pan the camera
- **Mouse wheel**: Zoom in/out

### Real-Time Display
- **Energy metrics**:
  - Kinetic Energy (J)
  - Potential Energy (J)
  - Total Energy (J)
- **FPS counter**
- **Status indicators**

## Architecture

### Custom UI Components

#### Slider Class
```python
class Slider:
    def __init__(self, x, y, width, min_val, max_val, initial_val, label, step=1):
        # Creates draggable slider with real-time value updates
        # Supports integer and float values with custom step sizes
```

**Features**:
- Smooth dragging with mouse
- Hover effects on handle
- Real-time value display
- Configurable step sizes

#### Button Class
```python
class Button:
    def __init__(self, x, y, width, height, label, color_scheme='green'):
        # Creates clickable button with hover effects
        # Supports color schemes: 'green', 'orange', 'blue'
```

**Features**:
- Enable/disable state
- Hover highlighting
- Color-coded for different actions
- Centered text labels

### NewtonsCradle Class

Main simulation class inheriting from `Simulation`:

```python
class NewtonsCradle(Simulation):
    def __init__(self):
        # Setup UI panel (right side)
        # Create sliders and buttons
        # Initialize simulation state

    def setup(self):
        # Create balls with precise spacing (radius * 2.0)
        # Create distance constraints for each ball
        # Apply initial angle to first ball

    def handle_input(self):
        # Process slider adjustments
        # Handle button clicks
        # Manage camera controls

    def render(self):
        # Draw physics world
        # Draw UI panel
        # Display energy metrics
```

## Physics Configuration

### Ball Properties
```python
# Ball creation
ball = RigidBody()
collider = CircleCollider(ball_radius)  # Default: 0.5m
ball.set_collider(collider)

# Mass and inertia
density = 1.0
mass = collider.calculate_mass(density)
inertia = collider.calculate_inertia(mass)
ball.set_mass(mass)
ball.set_moment_of_inertia(inertia)

# Material properties
ball.set_restitution(0.99)  # Very elastic for momentum transfer
ball.set_friction(0.0)       # Frictionless for clean collisions
```

### Constraint Setup
```python
# Rigid rod constraint
rod = DistanceConstraint(
    ball,                    # Body to constrain
    Vector2(0, 0),          # Attach at ball center
    anchor_point,           # Fixed point above ball
    rod_length,             # Distance from anchor to ball
    0.0001                  # Very small compliance = nearly rigid
)
world.add_constraint(rod)
```

**Why 0.0001 compliance?**
- Zero compliance (`0.0`) is mathematically perfect but numerically unstable
- Small non-zero value (`0.0001`) provides stability while maintaining rigidity
- Critical for energy conservation in the simulation

### Ball Spacing
```python
ball_spacing = ball_radius * 2.0  # Balls exactly touching
```

**Why exactly touching?**
- Ensures immediate collision when balls swing back
- No gaps = no momentum loss
- Demonstrates clean momentum transfer

### Initial Angle
```python
if i == 0 and initial_angle > 0:
    angle_rad = math.radians(initial_angle)
    # Calculate displaced position
    x = start_x - rod_length * math.sin(angle_rad)
    y = anchor_y - rod_length * math.cos(angle_rad)
    ball.set_position(Vector2(x, y))
```

Positions the first ball at the specified angle, creating potential energy that converts to kinetic energy when released.

## Energy Calculation

### Kinetic Energy
```python
total_ke = 0.0
for ball in balls:
    total_ke += ball.get_kinetic_energy()
```

Calculated using: `KE = ½mv² + ½Iω²` (linear + rotational)

### Potential Energy
```python
gravity_magnitude = abs(world.get_gravity().y)
lowest_point = anchor_y - rod_length

total_pe = 0.0
for ball in balls:
    mass = ball.get_mass()
    height = ball.get_position().y - lowest_point
    total_pe += mass * gravity_magnitude * height
```

Calculated using: `PE = mgh` (height relative to lowest swing point)

### Total Energy
```python
total_energy = total_ke + total_pe
```

Should remain approximately constant during simulation (small decreases due to numerical integration and constraint compliance).

## UI Layout

```
+------------------------------------------+
|  Status: "Configure and click PLAY"     |  <- Center-top
+------------------------------------------+
|                                    +-----+
|                                    | UI  |
|     Physics Simulation Area        | Pa  |
|     (balls, rods, anchors)         | nel |
|                                    |     |
|                                    | Sli |
|  FPS: 60                           | der |
|                                    | s   |
|                                    |     |
|                                    | But |
|                                    | ton |
|                                    | s   |
|                                    |     |
|                                    | Ene |
|                                    | rgy |
|                                    |     |
|                                    | Cam |
+------------------------------------+-----+
```

**Panel Dimensions**:
- Width: 280px
- Position: Right side of screen
- Semi-transparent dark background (alpha: 220)

## Key Implementation Details

### Setup vs. Playing Modes

#### Setup Mode (`is_playing = False`)
- Sliders are active and editable
- Configuration changes rebuild the simulation
- Physics is paused
- PLAY button is enabled
- PAUSE button is disabled

#### Playing Mode (`is_playing = True`)
- Sliders are locked (not editable)
- Physics is running (unless paused)
- PLAY button is disabled
- PAUSE and RESET buttons are enabled

### State Management
```python
# Three states
self.is_playing  # False = setup, True = simulation running
self.paused      # True = physics paused, False = physics active

# Button behavior
if not is_playing:
    # In setup mode - can adjust sliders

if paused:
    # Physics step skipped
```

### Camera Pan Implementation
```python
# Check click is not on UI panel
if event.pos[0] < renderer.width - ui_panel_width:
    mouse_dragging = True
    last_mouse_pos = event.pos

# Pan on drag
if mouse_dragging:
    dx = event.pos[0] - last_mouse_pos[0]
    dy = event.pos[1] - last_mouse_pos[1]
    renderer.camera.pan(-dx, -dy)  # Negative for intuitive drag
```

## Customization Ideas

### More/Fewer Balls
Already implemented! Use the slider (3-15 balls).

### Different Rod Lengths
Already implemented! Use the slider (2-10m).

### Two-Ball Collision
Set slider to 3 balls, angle to 30°, and watch perfect momentum transfer.

### Variable Mass
Modify the code to set different masses:
```python
if i == 0:
    ball.set_mass(2.0)  # Heavier first ball
else:
    ball.set_mass(1.0)
```

### Different Materials
Adjust restitution for different behaviors:
```python
ball.set_restitution(0.9)   # Less elastic (some energy loss)
ball.set_restitution(0.99)  # Nearly perfect elastic
ball.set_restitution(1.0)   # Perfect elastic (theoretical)
```

### Friction Effects
Add friction to observe damping:
```python
ball.set_friction(0.1)  # Slight friction with air/pivot
```

## Physics Validation

### What to Observe

1. **Momentum Transfer**:
   - Pull back N balls → N balls swing out on other side
   - Total momentum is conserved

2. **Energy Conservation**:
   - Total energy stays approximately constant
   - Small decreases are due to numerical integration error and constraint compliance
   - Less than 1% energy loss per second is excellent

3. **Symmetry**:
   - Balls should swing to approximately the same height on both sides
   - Multiple cycles should maintain consistency

4. **Collision Timing**:
   - Collisions should be instantaneous (no visible penetration)
   - No gaps between balls at rest

### Expected Behavior

| Configuration | Expected Result |
|---------------|----------------|
| 1 ball pulled, 5 total | 1 ball swings out opposite side |
| 2 balls pulled, 5 total | 2 balls swing out opposite side |
| 30° angle | Returns to approximately 30° on opposite side |
| High restitution (0.99) | Minimal energy loss, many cycles |
| Low restitution (0.8) | Noticeable damping, fewer cycles |

## Troubleshooting

### Energy Loss Too High
- **Cause**: Too many constraint solver iterations
- **Fix**: Constraint iterations are hardcoded to 3+2=5 total (optimal for this demo)

### Balls Not Colliding
- **Cause**: Ball spacing incorrect
- **Fix**: Spacing is set to `radius * 2.0` (correct)

### Unstable Motion
- **Cause**: Compliance too low or time step too large
- **Fix**: Compliance is 0.0001 (optimal), dt is 1/60 (good)

### UI Not Responding
- **Cause**: Click might be outside slider/button area
- **Fix**: Sliders are large and buttons have hover feedback

## Performance

- **60 FPS** target with up to 15 balls
- Constraint solving is O(iterations × num_constraints)
- Each ball has one constraint = linear scaling
- 15 balls = 15 constraints, easily handled

## Code Organization

```
newtons_cradle.py
├── Slider class (lines 27-109)
├── Button class (lines 112-160)
├── NewtonsCradle class (lines 163-453)
│   ├── __init__: Setup UI
│   ├── setup: Create physics objects
│   ├── on_key_press: Keyboard input
│   ├── start_simulation: Begin physics
│   ├── pause_simulation: Toggle pause
│   ├── reset_simulation: Return to setup
│   ├── handle_input: Process all input
│   ├── step_physics: Physics update
│   ├── render: Draw everything
│   ├── draw_stats: FPS display
│   ├── draw_ui: Right panel
│   └── draw_status: Top status bar
└── main function (lines 455-461)
```

## Educational Value

This example teaches:
1. **XPBD constraints** in action
2. **Energy conservation** in elastic collisions
3. **Momentum transfer** through collision chains
4. **GUI programming** with Pygame
5. **Event-driven architecture** for interactive simulations
6. **State management** (setup vs. playing modes)
7. **Real-time physics** visualization

## See Also

- `10_Constraint_System_Documentation.md` - Constraint theory and API
- `03_RigidBody_Documentation.md` - Rigid body physics
- `06_Collision_System_Documentation.md` - Collision resolution
