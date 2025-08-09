# OrbitSim

OrbitSim is an interactive 2D N‑body orbital mechanics sandbox built with Python, Pygame (viewport) and Dear PyGui (controls). It lets you load presets, add bodies with SI units, and watch gravitational dynamics evolve with trails, collisions, and multiple classic 3‑body configurations.

## Features

- Realistic N‑body gravity (SI units) with RK4 integration
- Interactive dual‑window interface:
  - Pygame viewport for rendering, zooming, panning, and selection
  - Dear PyGui control panel for editing bodies and simulation settings
- Built‑in collision handling: Merge, Elastic, or Accretion modes
- Motion trails with configurable length
- Presets and JSON templates system (templates/)
- Celestial body library (celestial_bodies/) and UI loader to insert bodies by JSON definition
- Several classic three‑body problem setups (figure‑eight, Lagrange triangle)
- Modular core: physics, collisions, data models, vector utils, constants, camera, and loader utilities

## Requirements

- Python 3.10+
- Windows, macOS, or Linux

Install dependencies:

```
pip install pygame dearpygui
```

## Run

```
python orbit_sim.py
```

Two windows will appear:
- Orbit Simulator - Viewport (Pygame)
- Orbit Simulator - Controls (Dear PyGui)

## Controls (Viewport)

- Mouse wheel: Zoom in/out (cursor‑anchored)
- Left‑click + drag: Move selected body
- Right/Middle‑click + drag or Arrow keys: Pan
- Spacebar (in controls window): Play/Pause

## Controls (Control Panel)

- Preset dropdown + Load: Load a scene template from templates/ or built‑ins
- Add New Body: Create a body with SI units (kg, m) and optional Earth/Solar conversions
- Current Bodies: Select a body for operations
- Apply Velocity: Set velocity (m/s) on the selected body
- Edit Selected Body: Edit mass, radius, position, and color, then Apply Body Edits
- Simulation Controls:
  - Play/Pause and Step ▶
  - Trails toggle and length
  - Speed (real‑time scale) and Softening (gravity softening length)
  - Collisions: mode, restitution, radius scale, and enable/disable
  - Integrator: RK4
- Body Library:
  - Choose a JSON from celestial_bodies/
  - Enter position and velocity
  - Insert From Library to add it to the scene
  - Refresh Library to rescan files

## Presets and Templates

OrbitSim auto‑discovers JSON templates placed in templates/. Each template can specify:

```
{
  "name": "Solar System (scaled)",
  "description": "Optional description",
  "time_scale": 3500000.0,  // optional
  "bodies": [
    {
      "name": "Sun",
      "mass": 1.98847e30,
      "radius": 6.96342e8,
      "position": [0.0, 0.0],
      "velocity": [0.0, 0.0],
      "color": [255, 204, 0]
    }
  ]
}
```

Place body definitions in celestial_bodies/ to reuse across scenes:

```
{
  "name": "Earth",
  "mass": 5.972e24,
  "radius": 6.371e6,
  "color": [100, 149, 237]
}
```

Built‑in templates and examples are included; you can also use the built‑in presets when no JSONs are found.

## Recommended Workflow

1. Start with the Empty custom preset (default)
2. Insert a primary (e.g., Sun) via Body Library
3. Insert orbiters (e.g., Earth, Moon), set positions and initial velocities
4. Use Auto‑fit Camera and adjust Speed/Softening as needed
5. Toggle trails to visualize motion

Tip: For a circular orbit of radius r around a primary of mass M, use v ≈ sqrt(G*M/r) tangential to the radius.

## File Structure

- orbit_sim.py — entry point and UI/renderer orchestration
- core/
  - physics.py — RK4 integration and gravity
  - collisions.py — collision detection and response
  - data_models.py — Body dataclass
  - vector_utils.py — vector helpers
  - constants.py — physical constants
  - camera.py — 2D camera transforms
  - presets_loader.py — JSON loading utilities
- templates/ — scene templates (.json)
- celestial_bodies/ — reusable body definitions (.json)

## Troubleshooting

- Edit panel not applying: Ensure you click "Apply Body Edits" after changes. The panel now avoids overwriting while typing and updates only when selection changes.
- Simulation too slow or fast: Adjust Speed slider (time scale) and Softening. Trails and velocity vectors can impact performance.
- Bodies flying apart in 3‑body: Use the provided Figure‑eight or Lagrange templates, or compute initial velocities carefully.
- Nothing visible: Use Auto‑fit Camera and ensure bodies are added; Empty custom starts blank by design.

## Roadmap / Ideas

- Additional integrators (Velocity Verlet, Symplectic)
- Energy/angular momentum diagnostics
- Barnes–Hut or GPU acceleration
- In‑app template editor and JSON export
- Auto‑compute circular/elliptic orbital velocities around a selected primary

## License

MIT License (add your preferred license here)

