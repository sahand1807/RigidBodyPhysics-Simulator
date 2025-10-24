<p align="center">
  <img src="Logo.png" alt="2D Rigid Body Physics Simulator Logo" width="400"/>
</p>

# 2D Rigid Body Physics Simulator

A high-performance 2D rigid body physics engine written in C++ from scratch, with Python bindings for visualization and interaction.

## Overview

This project implements a complete 2D physics simulation system from scratch, featuring:

- **Rigid body dynamics**: Circles and boxes with mass, velocity, and angular motion
- **Collision detection**: Efficient algorithms for circle-circle and circle-box collisions
- **Collision resolution**: Impulse-based physics with friction and restitution
- **Python integration**: Easy-to-use Python API for creating scenes and visualizations
- **Interactive demos**: Real-time visualization with Pygame

## Features

### Core Physics Engine (C++)

- **Rigid Bodies**
  - Circle and box objects
  - Mass, position, velocity, angular velocity
  - Semi-implicit Euler integration for stability

- **Collision System**
  - Circle-circle collision detection
  - Circle-box collision detection
  - SAT (Separating Axis Theorem) for box collisions
  - Impulse-based collision resolution
  - Configurable friction and restitution coefficients

- **Forces**
  - Gravity simulation
  - User-applied impulses
  - Force accumulation system

### Python Visualization Layer

- Real-time rendering with Pygame
- Interactive scene creation
- Multiple demo scenarios included

## Project Structure

```
physics_engine/
├── cpp/                    # C++ physics engine
│   ├── include/           # Header files
│   │   ├── math/         # Vector2, Transform
│   │   ├── physics/      # RigidBody, Collider, PhysicsWorld
│   │   └── bindings/     # Python binding headers
│   ├── src/              # Implementation files
│   └── CMakeLists.txt    # Build configuration
├── python/               # Python wrapper and visualization
│   ├── physics_viz/     # Rendering and scene management
│   └── examples/        # Demo scenarios
├── tests/               # Unit tests
└── README.md           
```

## Building from Source

### Prerequisites

- CMake >= 3.12
- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- Python >= 3.8
- pybind11

### Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd RigidBodyPhysics-Simulator
```

2. Install Python dependencies:
```bash
pip install -r requirements.txt
```

3. Build and install the package:
```bash
pip install -e .
```

## Usage

### Basic Example

```python
from physics_viz import PhysicsWorld, RigidBody

# Create a physics world
world = PhysicsWorld()

# Add a circle
circle = RigidBody(shape='circle', radius=1.0, mass=1.0)
circle.position = (5.0, 10.0)
world.add_body(circle)

# Simulate
for i in range(100):
    world.step(dt=0.016)  # 60 FPS
    print(f"Position: {circle.position}")
```

### Running Demos

```bash
# Interactive sandbox (click to spawn balls)
python3 python/examples/sandbox.py

# Bouncing balls demonstration
python3 python/examples/bouncing_balls.py

# Momentum transfer through collision chain
python3 python/examples/momentum_transfer.py

# Ball pit with dynamic spawning
python3 python/examples/ball_pit.py
```

**Controls:**
- **Mouse Drag**: Pan camera
- **Mouse Wheel**: Zoom in/out
- **Space**: Pause/Resume simulation
- **R**: Reset simulation
- **G**: Toggle grid display
- **V**: Toggle velocity vectors
- **H**: Toggle help overlay
- **ESC**: Quit

## Demo Scenarios

1. **Interactive Sandbox**: Click to spawn random balls and watch them interact
2. **Bouncing Balls**: Simple demonstration of balls bouncing with varying properties
3. **Momentum Transfer**: Projectile hits stationary chain, demonstrating elastic collision momentum transfer
4. **Ball Pit**: Continuous spawning of balls into a container

## Physics Theory

### Rigid Body Dynamics

A rigid body is an idealized solid object that doesn't deform. Each body can be fully defined using its:
- **Position** (x, y): Center of mass location
- **Velocity** (vx, vy): Linear motion
- **Angle** θ: Orientation
- **Angular velocity** ω: Rotational motion
- **Mass** m: Resistance to linear acceleration
- **Moment of inertia** I: Resistance to angular acceleration

### Integration

In this prject we use **semi-implicit Euler integration**:
```
v(t+dt) = v(t) + a(t) * dt
x(t+dt) = x(t) + v(t+dt) * dt
```

This provides better energy conservation than explicit Euler. Explicit Euler tends to add energy over time which leads to the simulated object speeding up or oscillating.

### Collision Response

Collisions are resolved using **impulse-based resolution**:
1. Detect collision and compute contact normal
2. Calculate relative velocity at contact point
3. Compute impulse magnitude: `j = -(1 + e) * v_rel · n / (1/m1 + 1/m2)`
4. Apply equal and opposite impulses to bodies

Where:
- `e` is the coefficient of restitution (bounciness)
- `n` is the collision normal
- `v_rel` is the relative velocity

## Development Roadmap

- [x] Phase 1: Project setup
- [x] Phase 2: Core math (Vector2, Transform)
- [x] Phase 3: Basic physics (RigidBody, CircleCollider, PhysicsWorld)
- [x] Phase 4: Collision system (circle-circle detection and response)
- [x] Phase 5: Python bindings (pybind11, full C++ API exposure)
- [x] Phase 6: Visualization (Pygame, camera system, interactive demos)
- [x] Phase 7: Enhanced features (BoxCollider, box-circle collision, realistic ground planes)


## Creator

Sahand Sadeghi - [Intelligent Dynamics](http://inteldynamic.com/)