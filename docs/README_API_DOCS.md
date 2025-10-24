# API Documentation Generation Guide

This guide explains how to generate comprehensive API documentation for both C++ and Python code.

## C++ API Documentation (Doxygen)

### Prerequisites

Install Doxygen and Graphviz:

```bash
# Ubuntu/Debian
sudo apt-get install doxygen graphviz

# macOS
brew install doxygen graphviz

# Windows
# Download from https://www.doxygen.nl/download.html
# Download Graphviz from https://graphviz.org/download/
```

### Generating Documentation

From the project root directory:

```bash
doxygen Doxyfile
```

This will create documentation in `docs/api/cpp/html/`.

### Viewing Documentation

Open in your web browser:

```bash
# Linux/macOS
open docs/api/cpp/html/index.html

# Or just navigate to the file in your browser
```

### What's Included

The C++ documentation includes:
- **Class hierarchy** with inheritance diagrams
- **Function documentation** with parameters and return values
- **Call graphs** showing function dependencies
- **Collaboration diagrams** showing class relationships
- **Source code browsing** with cross-references
- **File documentation** for all headers and source files

### Key Classes Documented

- `Vector2` - 2D vector math
- `Transform` - Position and rotation
- `RigidBody` - Physics object
- `Collider`, `CircleCollider`, `BoxCollider` - Collision shapes
- `PhysicsWorld` - Main physics simulation
- `Constraint`, `DistanceConstraint` - Physics constraints
- `CollisionDetection`, `CollisionResponse` - Collision system

## Python API Documentation (Sphinx)

### Prerequisites

Install Sphinx and extensions:

```bash
# Using the project's virtual environment
source venv/bin/activate
pip install sphinx sphinx-rtd-theme sphinx-autodoc-typehints
```

### Setting Up Sphinx (First Time Only)

```bash
cd python
sphinx-quickstart docs/api
```

When prompted:
- Separate source and build directories: **Yes**
- Project name: **2D Rigid Body Physics Simulator**
- Author: **Your Name**
- Version: **1.0.0**
- Language: **en**

### Configuration

Edit `python/docs/api/source/conf.py`:

```python
import os
import sys
sys.path.insert(0, os.path.abspath('../..'))

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx_autodoc_typehints',
]

html_theme = 'sphinx_rtd_theme'

autodoc_member_order = 'bysource'
```

### Generating Documentation

```bash
cd python
sphinx-apidoc -f -o docs/api/source ../physics_viz
cd docs/api
make html
```

### Viewing Documentation

```bash
open python/docs/api/build/html/index.html
```

### What's Included

The Python documentation includes:
- **Module documentation** for all Python packages
- **Class documentation** with methods and attributes
- **Function signatures** with type hints
- **Examples** from docstrings
- **Cross-references** to related classes

### Key Modules Documented

- `physics_viz.physics_engine_core` - C++ bindings (Vector2, RigidBody, etc.)
- `physics_viz.camera` - Camera system
- `physics_viz.renderer` - Rendering engine
- `physics_viz.simulation` - Base simulation class

## Quick Documentation Commands

Add these to your workflow:

```bash
# Generate both C++ and Python docs
make docs  # (You can add this to a Makefile)

# Or manually:
doxygen Doxyfile && cd python/docs/api && make html && cd ../../..
```

## Continuous Documentation

### Adding Documentation to C++ Code

Use Doxygen comment style:

```cpp
/**
 * @brief Calculates the distance between two vectors
 *
 * @param other The other vector
 * @return float The Euclidean distance
 */
float distance(const Vector2& other) const;
```

### Adding Documentation to Python Code

Use docstrings:

```python
def calculate_energy(self):
    """Calculate the total energy of the system.

    Returns:
        float: Total energy in Joules (kinetic + potential)

    Example:
        >>> energy = world.calculate_energy()
        >>> print(f"Total energy: {energy} J")
    """
    return self.kinetic_energy + self.potential_energy
```

## Updating Documentation

Whenever you add new classes or functions:

1. **Add proper documentation comments** to your code
2. **Regenerate the documentation**:
   ```bash
   doxygen Doxyfile  # For C++
   cd python/docs/api && make html  # For Python
   ```
3. **Commit the updated docs** to version control

## Hosting Documentation

### GitHub Pages

You can host the documentation on GitHub Pages:

1. Generate documentation
2. Copy `docs/api/cpp/html` to `docs/cpp`
3. Copy `python/docs/api/build/html` to `docs/python`
4. Enable GitHub Pages in repository settings
5. Set source to `docs` folder

### Read the Docs

For Python documentation, you can use Read the Docs:

1. Create account at https://readthedocs.org
2. Link your GitHub repository
3. RTD will automatically build Sphinx documentation

## Documentation Structure

```
docs/
├── 01_Vector2_Documentation.md          # Manual docs
├── 02_Transform_Documentation.md
├── ...
├── 10_Constraint_System_Documentation.md
├── 11_Newtons_Cradle_Example.md
├── api/                                  # Generated API docs
│   ├── cpp/                             # Doxygen output
│   │   └── html/
│   │       └── index.html
│   └── python/                          # Sphinx output (if setup)
│       └── build/
│           └── html/
│               └── index.html
└── README_API_DOCS.md                   # This file
```

## Tips

- **Keep documentation up to date** - Update docs when you change code
- **Use examples** - Show how to use classes and functions
- **Cross-reference** - Link related classes and functions
- **Be concise** - Document what, not how (code shows how)
- **Test examples** - Ensure code examples actually work

## Troubleshooting

### Doxygen not finding files
- Check `INPUT` paths in Doxyfile
- Ensure `RECURSIVE = YES`

### Sphinx import errors
- Ensure virtual environment is activated
- Check `sys.path` in `conf.py`
- Install missing dependencies

### Graphs not generating
- Install Graphviz: `sudo apt-get install graphviz`
- Set `HAVE_DOT = YES` in Doxyfile

### Documentation looks broken
- Clear build directory: `rm -rf docs/api/*/build`
- Regenerate from scratch

## Further Reading

- [Doxygen Manual](https://www.doxygen.nl/manual/)
- [Sphinx Documentation](https://www.sphinx-doc.org/)
- [Read the Docs Tutorial](https://docs.readthedocs.io/en/stable/tutorial/)
