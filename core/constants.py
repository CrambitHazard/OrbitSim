#!/usr/bin/env python3
"""
Shared constants for Orbit Simulator (SI units unless stated otherwise).

Keeping constants in one place helps ensure values are consistent across the
codebase and makes tuning easier.
"""

# Physical constants
G = 6.67430e-11  # m^3 kg^-1 s^-2
EARTH_MASS = 5.972e24  # kg
SOLAR_MASS = 1.98847e30  # kg
EARTH_RADIUS = 6.371e6  # m
SOLAR_RADIUS = 6.96342e8  # m

# Physics controls
DEFAULT_SOFTENING = 1e7  # m; gravitational softening to avoid singularities
BASE_DT = 1 / 120.0  # seconds of simulation time per physics substep
MAX_SUBSTEPS = 1000  # cap per frame for stability/perf

# Rendering (viewport)
VIEW_WIDTH = 1100
VIEW_HEIGHT = 800
BACKGROUND_COLOR = (10, 12, 18)
GRID_COLOR = (40, 45, 60)
GRID_FINE_COLOR = (30, 34, 50)
SELECTION_COLOR = (255, 255, 0)
VELOCITY_VECTOR_COLOR = (255, 255, 255)

# Camera zoom bounds (meters-per-pixel)
DEFAULT_METERS_PER_PIXEL = 2.5e7
MIN_METERS_PER_PIXEL = 1e3
MAX_METERS_PER_PIXEL = 1e11

# Safety: avoid drawing outside reasonable integer pixel ranges
SAFE_COORD_LIMIT = 30000

