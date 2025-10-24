# Interactive Physics Builder - User Guide

## Overview

The **Interactive Physics Builder** is a full-featured scene editor that allows you to design custom physics simulations with complete control over every parameter. Build scenes by adding objects one at a time with precise configuration, then run the simulation to see your creation in action.

## Launching the Builder

```bash
python3 python/examples/physics_builder.py
```

## Interface Layout

The builder window is divided into two main areas:

### Simulation View (Left)
- Large canvas showing your physics scene
- Grid background with coordinate axes
- Real-time rendering of all objects
- Status display (BUILDING/SIMULATING/PAUSED)
- Object counter

### Control Panel (Right)
- Shape selection buttons
- Parameter input fields
- Simulation controls
- Add/Remove object buttons

## Building a Scene

### Step 1: Select Shape Type

Choose between two shape types:
- **Circle**: Circular objects (balls, wheels, etc.)
- **Box**: Rectangular objects (platforms, walls, crates)

Click the corresponding button to select the shape type.

### Step 2: Configure Shape Parameters

**For Circles:**
- **Radius (m)**: Size of the circle in meters
  - Example: `1.0` for a 1-meter radius ball

**For Boxes:**
- **Width (m)**: Horizontal dimension in meters
- **Height (m)**: Vertical dimension in meters
  - Example: `10.0` width, `1.0` height for a platform

### Step 3: Set Position

Define where the object will appear in the world:
- **X (m)**: Horizontal position (negative = left, positive = right)
- **Y (m)**: Vertical position (negative = down, positive = up)
  - Example: `(0, 5)` places object at center, 5 meters up

**Coordinate System:**
- Origin (0, 0) is at the center of the view
- X-axis points right (red line)
- Y-axis points up (green line)

### Step 4: Set Initial Velocity

Define the starting motion of the object:
- **Vx (m/s)**: Horizontal velocity (negative = left, positive = right)
- **Vy (m/s)**: Vertical velocity (negative = down, positive = up)
  - Example: `Vx=5, Vy=0` for horizontal launch
  - Example: `Vx=0, Vy=-10` for downward throw

**Note:** Static objects ignore velocity.

### Step 5: Configure Physical Properties

**Mass (kg):**
- Defines how heavy the object is
- Heavier objects are harder to move
- Range: 0.1 to infinity (practical: 0.5 - 100 kg)
- Example: `1.0` for a light ball, `10.0` for a heavy ball

**Material Properties (Hardcoded):**
- **Restitution**: 0.9 (very bouncy, minimal energy loss)
- **Friction**: Not set (frictionless motion)
- These values are optimized to avoid physics engine bugs and provide good bounce behavior
- Objects will bounce well and maintain energy longer

### Step 6: Set Static Property

**Static Checkbox:**
- **Unchecked (Dynamic)**: Object moves and responds to forces
- **Checked (Static)**: Object is immovable (ground, walls, platforms)

**Use Static For:**
- Ground planes
- Container walls
- Fixed platforms
- Obstacles

**Note:** Static objects have infinite mass and don't move.

### Step 7: Add Object to Scene

Click the **"Add Object"** button to create the object with your configured settings.

The object will appear in the simulation view at the specified position.

## Simulation Controls

### Gravity Toggle

**"Gravity: ON/OFF" Button:**
- **ON** (default): Standard Earth gravity (9.81 m/s² downward)
- **OFF**: Zero gravity (space environment)

Objects will float when gravity is off.

### Running the Simulation

**"RUN" Button:**
- Starts the physics simulation
- Objects begin moving according to physics laws
- All forces and collisions are calculated
- Status changes to "SIMULATING"

**"PAUSE" Button:**
- Freezes the simulation at current state
- Objects stop moving
- Press again to resume
- Status changes to "PAUSED"

**"RESET" Button:**
- Stops simulation and returns to building mode
- Resets all object velocities to zero
- Keeps all objects in scene
- Status changes to "BUILDING"

**"CLEAR" Button:**
- Removes ALL objects from the scene
- Resets to empty scene
- Returns to building mode

## Example Scenes

### Example 1: Simple Bouncing Ball

1. Select **Circle**
2. Set Radius: `1.0`
3. Position: X=`0`, Y=`10`
4. Velocity: Vx=`0`, Vy=`0`
5. Mass: `1.0`
6. Restitution: `0.8` (bouncy)
7. Friction: `0.3`
8. Static: **Unchecked**
9. Click **Add Object**

10. Select **Box** (for ground)
11. Width: `20.0`, Height: `1.0`
12. Position: X=`0`, Y=`-5`
13. Static: **Checked**
14. Click **Add Object**

15. Gravity: **ON**
16. Click **RUN**

**Result:** Ball falls and bounces on platform.

### Example 2: Projectile Launch

1. Create box ground (same as Example 1, step 10-14)

2. Select **Circle**
3. Radius: `0.5`
4. Position: X=`-10`, Y=`2`
5. Velocity: Vx=`15`, Vy=`8` (launch angle)
6. Mass: `1.0`
7. Restitution: `0.6`
8. Friction: `0.3`
9. Click **Add Object**

10. Click **RUN**

**Result:** Projectile follows parabolic arc and lands on ground.

### Example 3: Newton's Cradle (Simplified)

1. Create ground platform (box, static)

2. Create 5 circles in a row:
   - Circle 1: X=`-4`, Y=`5`, Vx=`0`, Vy=`0`, Mass=`1.0`
   - Circle 2: X=`-2`, Y=`5`, Vx=`0`, Vy=`0`, Mass=`1.0`
   - Circle 3: X=`0`, Y=`5`, Vx=`0`, Vy=`0`, Mass=`1.0`
   - Circle 4: X=`2`, Y=`5`, Vx=`0`, Vy=`0`, Mass=`1.0`
   - Circle 5: X=`4`, Y=`5`, Vx=`0`, Vy=`0`, Mass=`1.0`
   - All with Restitution=`1.0` (perfect elastic)

3. Create projectile:
   - Circle: X=`-10`, Y=`5`, Vx=`10`, Vy=`0`, Mass=`1.0`, Restitution=`1.0`

4. Click **RUN**

**Result:** Momentum transfers through the chain.

### Example 4: Container with Falling Objects

1. Create container:
   - Bottom: Box, Width=`20`, Height=`1`, X=`0`, Y=`-8`, Static
   - Left Wall: Box, Width=`1`, Height=`20`, X=`-10`, Y=`0`, Static
   - Right Wall: Box, Width=`1`, Height=`20`, X=`10`, Y=`0`, Static

2. Add multiple balls:
   - Create 5-10 circles at Y=`15` with random X positions
   - Various radii (0.5 - 1.5)
   - Zero initial velocity
   - Mass: `1.0`
   - Restitution: `0.5-0.7`

3. Gravity: **ON**
4. Click **RUN**

**Result:** Balls fall into container and pile up.

### Example 5: Zero Gravity Chaos

1. Create several objects with high velocities
   - Mix of circles and boxes
   - Random positions
   - Random velocities in all directions
   - Restitution: `1.0` (perfect bounce)

2. Gravity: **OFF**
3. Click **RUN**

**Result:** Objects collide and bounce with no energy loss, creating perpetual motion.

### Example 6: Ramp and Rolling

1. Create angled platform:
   - Box, Width=`15`, Height=`0.5`
   - Position: X=`0`, Y=`0`
   - **Note:** Cannot set rotation in current version, so use horizontal platform

2. Create rolling ball:
   - Circle, Radius=`1.0`
   - Position: X=`-5`, Y=`5`
   - Velocity: Vx=`3`, Vy=`0` (horizontal roll)
   - Friction: `0.5` (enables rolling)

3. Click **RUN**

**Result:** Ball rolls along platform with rotational motion.

## Tips and Best Practices

### Object Placement
- Start with Y > 0 for falling objects
- Use Y < 0 for ground planes
- Leave space between objects to avoid initial overlaps

### Realistic Physics
- Use Restitution 0.6-0.8 for most bouncing scenarios
- Use Friction 0.3-0.5 for normal interactions
- Keep Mass between 0.5-10 kg for stable simulations

### Performance
- Keep total objects under 50 for smooth performance
- Use **CLEAR** button to remove all objects and start fresh
- Static objects are computationally cheap

### Debugging
- Watch the object count in the simulation view
- Use **PAUSE** to freeze and inspect the scene
- Use **RESET** to restart without rebuilding

### Common Issues

**Objects fall through ground:**
- Ensure ground is **Static**
- Check ground Y position is below objects
- Verify objects don't start overlapping

**Objects don't bounce:**
- Check Restitution > 0
- Verify both objects have appropriate restitution values
- Static objects can have restitution too

**Objects slide too much:**
- Increase Friction value
- Check both objects have friction set

**Simulation is too fast/slow:**
- This is real-time physics (1 second = 1 second)
- Adjust velocities for desired effect

## Keyboard Shortcuts

- **Space**: Pause/Resume simulation
- **ESC**: Quit application

## Physics Parameters Reference

### Recommended Values

**Small Ball (tennis ball):**
- Radius: 0.3-0.5 m
- Mass: 0.5 kg
- Restitution: 0.7
- Friction: 0.4

**Large Ball (basketball):**
- Radius: 1.0-1.5 m
- Mass: 2.0 kg
- Restitution: 0.8
- Friction: 0.5

**Heavy Ball (bowling ball):**
- Radius: 1.0 m
- Mass: 7.0 kg
- Restitution: 0.3
- Friction: 0.6

**Platform/Ground:**
- Box: Width=10-50 m, Height=1-2 m
- Static: Yes
- Restitution: 0.5-0.7
- Friction: 0.5

**Wall:**
- Box: Width=1 m, Height=20 m
- Static: Yes
- Restitution: 0.6
- Friction: 0.5

## Advanced Techniques

### Creating Stable Stacks
1. Build from bottom up
2. Use low restitution (0.2-0.4) to reduce bouncing
3. High friction (0.6-0.8) to prevent sliding
4. Start with zero velocity
5. Let gravity settle before adding more

### Launching Projectiles
1. Position at desired starting point
2. Set Vx and Vy to create launch angle
3. Formula: Vx = speed × cos(angle), Vy = speed × sin(angle)
4. Example: 45° launch at 10 m/s: Vx=7.07, Vy=7.07

### Creating Pendulums
**Note:** True pendulums require constraints (not yet implemented).
Approximate with:
1. Ball at high position
2. Initial horizontal velocity
3. Restitution near 1.0
4. Will swing but not maintain perfect arc

## Troubleshooting

**Problem: Can't type in input fields**
- Click on the input field to activate it
- Field will highlight when active
- Press Enter when done

**Problem: Object appears in wrong position**
- Check X/Y values (positive/negative)
- Remember: Y increases upward, X increases right
- Grid shows 5-meter intervals

**Problem: Simulation doesn't start**
- Click **RUN** button
- Ensure objects are added (check object count)
- Try **RESET** then **RUN** again

**Problem: Objects explode/fly away**
- Reduce initial velocities
- Check for overlapping objects at start
- Verify mass values are reasonable (0.5-100 kg)

## Future Enhancements

Potential features for future versions:
- Click and drag to position objects
- Visual object preview before adding
- Save/Load scenes to files
- Rotation angle input for boxes
- Angular velocity input
- Delete individual objects
- Duplicate object feature
- Preset templates (pendulum, Newton's cradle, etc.)
- Time scale adjustment
- Replay/record functionality

## Conclusion

The Interactive Physics Builder provides a powerful and intuitive way to explore 2D rigid body physics. Experiment with different configurations, test physical principles, and create your own unique simulations!

**Have fun building!**
