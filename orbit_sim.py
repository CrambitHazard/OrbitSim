#!/usr/bin/env python3
"""
Orbit Simulator application entry point and UI/renderer coordination.

What this module does
- Starts two event loops: a Pygame rendering thread (viewport) and the Dear PyGui UI
  (running on the main thread).
- Maintains a shared SimulationController that owns the bodies and simulation settings;
  all access is guarded by a re-entrant lock for thread-safety.
- Provides built-in presets, camera and rendering utilities, and a Dear PyGui-based UI
  for creating/editing bodies, loading JSON presets, and controlling the simulation.

Threading model
- PygameRenderer runs in a background thread and performs: input handling (for the viewport),
  stepping physics, and drawing. It locks the SimulationController around short critical
  sections to read/update shared state.
- The UI class runs in the main thread via Dear PyGui. It updates controls on a periodic
  frame callback and invokes SimulationController methods as needed; these are lock-protected.

Units and conventions
- SI units throughout: meters [m], kilograms [kg], seconds [s]. Camera stores meters-per-pixel.
- Colors are RGB tuples in 0..255.

Running
1) Install dependencies: `pip install pygame dearpygui`
2) Run this module: `python orbit_sim.py`

Windows/OS notes
- Two windows will open: the viewport (Pygame) and the controls (Dear PyGui). Closing either
  will shut down the application cleanly.
"""

import math
import sys
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

# GUI and Rendering libs
import pygame
from pygame import gfxdraw
import dearpygui.dearpygui as dpg

# Modularized imports
from core.physics import NBodyPhysics
from core.collisions import CollisionSettings, handle_collisions
from core.presets_loader import (
    list_templates as list_json_templates,
    load_template as load_json_template,
    list_celestial_bodies,
    load_celestial_body,
)

# ============================================================
# Constants and Units
# ============================================================

# Gravitational constant (SI)
G = 6.67430e-11  # m^3 kg^-1 s^-2

# Softening default (to avoid singularities)
DEFAULT_SOFTENING = 1e7  # meters, adjustable in UI

# Exact conversion constants (SI)
# Earth mass: 5.972e24 kg
EARTH_MASS = 5.972e24  # kg
# Solar mass: 1.98847e30 kg
SOLAR_MASS = 1.98847e30  # kg
# Earth radius: 6.371e6 m
EARTH_RADIUS = 6.371e6  # m
# Solar radius: 6.96342e8 m
SOLAR_RADIUS = 6.96342e8  # m

# Default camera settings
DEFAULT_METERS_PER_PIXEL = 2.5e7  # meters per pixel; changed by zoom
MIN_METERS_PER_PIXEL = 1e3
MAX_METERS_PER_PIXEL = 1e11

# Rendering
BACKGROUND_COLOR = (10, 12, 18)
GRID_COLOR = (40, 45, 60)
GRID_FINE_COLOR = (30, 34, 50)
SELECTION_COLOR = (255, 255, 0)
VELOCITY_VECTOR_COLOR = (255, 255, 255)

# Simulation base timestep (seconds of simulation time per physics sub-step, scaled by speed)
BASE_DT = 1 / 120.0

# Pygame window size
VIEW_WIDTH = 1100
VIEW_HEIGHT = 800

# ============================================================
# Utility and Vector helpers
# ============================================================

def clamp(x, a, b):
    return max(a, min(b, x))

def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1])

def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def vec_scale(a, s):
    return (a[0] * s, a[1] * s)

def vec_len(a):
    return math.hypot(a[0], a[1])

def vec_norm(a):
    l = vec_len(a)
    if l == 0:
        return (0.0, 0.0)
    return (a[0] / l, a[1] / l)

def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1]

def try_float(val) -> Optional[float]:
    try:
        return float(val)
    except Exception:
        return None

# ============================================================
# Data Model
# ============================================================

@dataclass
class Body:
    name: str
    mass: float  # kg
    radius: float  # meters (for rendering and collision visuals)
    position: Tuple[float, float]  # meters
    velocity: Tuple[float, float]  # meters/second
    color: Tuple[int, int, int] = (200, 200, 255)
    trail: deque = field(default_factory=lambda: deque(maxlen=200))

    def add_trail_point(self):
        self.trail.append(self.position)

# ============================================================
# Physics Engine with RK4 integrator
# ============================================================

# Physics moved to core.physics.NBodyPhysics (rk4_integration_step)

# ============================================================
# Camera and Rendering Utilities
# ============================================================

class Camera2D:
    """
    Simple 2D camera for converting between world (meters) and screen (pixels).

    Attributes:
        center: world-space center (meters) kept as a mutable list [x, y].
        mpp: meters-per-pixel zoom level (smaller means zoomed-in).
        viewport_size: (width, height) in pixels.

    Methods:
        world_to_screen: convert a world position to integer pixel coordinates.
        screen_to_world: inverse transform from pixels to meters.
        zoom: change meters-per-pixel with optional pivot so the point under the cursor stays fixed.
        pan_pixels: move the camera center by a pixel offset (scaled by mpp).
    """
    def __init__(self, center=(0.0, 0.0), meters_per_pixel=DEFAULT_METERS_PER_PIXEL):
        self.center = list(center)  # world coords (m)
        self.mpp = meters_per_pixel  # meters per pixel
        self.viewport_size = (VIEW_WIDTH, VIEW_HEIGHT)

    def set_viewport_size(self, w, h):
        self.viewport_size = (w, h)

    def world_to_screen(self, pos: Tuple[float, float]) -> Tuple[int, int]:
        cx, cy = self.center
        mpp = self.mpp
        px = (pos[0] - cx) / mpp + self.viewport_size[0] / 2
        py = (pos[1] - cy) / mpp + self.viewport_size[1] / 2
        return (int(px), int(py))

    def screen_to_world(self, screen: Tuple[int, int]) -> Tuple[float, float]:
        cx, cy = self.center
        mpp = self.mpp
        wx = (screen[0] - self.viewport_size[0] / 2) * mpp + cx
        wy = (screen[1] - self.viewport_size[1] / 2) * mpp + cy
        return (wx, wy)

    def zoom(self, factor, pivot_screen: Optional[Tuple[int, int]] = None):
        """
        Zoom with optional pivot point (screen space) so that the world point under the cursor remains stationary.
        """
        factor = clamp(factor, 0.05, 20.0)
        before = None
        if pivot_screen is not None:
            before = self.screen_to_world(pivot_screen)
        self.mpp = clamp(self.mpp * (1.0 / factor), MIN_METERS_PER_PIXEL, MAX_METERS_PER_PIXEL)
        if pivot_screen is not None and before is not None:
            after = self.screen_to_world(pivot_screen)
            # Adjust center so the pivot world point stays under cursor
            self.center[0] += (before[0] - after[0])
            self.center[1] += (before[1] - after[1])

    def pan_pixels(self, dx_pixels, dy_pixels):
        self.center[0] -= dx_pixels * self.mpp
        self.center[1] -= dy_pixels * self.mpp

# ============================================================
# Simulation Controller (Shared State)
# ============================================================

class SimulationController:
    """
    Shared state between UI thread (DearPyGui) and rendering thread (Pygame).
    Includes thread-safe operations guarded by a lock.
    """
    def __init__(self):
        self.lock = threading.RLock()
        self.bodies: List[Body] = []
        self.running = True  # app running
        self.playing = True  # simulation running
        self.time_scale = 10000.0  # x real-time
        self.softening = DEFAULT_SOFTENING
        self.show_trails = True
        self.trail_length = 250
        self.show_velocity_vectors = False
        self.integrator = "RK4"
        self.selected_index: Optional[int] = None

        # Collisions settings
        self.enable_collisions = True
        self.collision_mode = "Accretion"  # Merge | Elastic | Accretion (merge if below escape speed)
        self.restitution = 0.2  # coefficient of restitution for Elastic/Accretion bounce [0..1]
        self.collision_radius_scale = 1.0  # scale physical radii for collision detection
        self.last_collision_msg: Optional[str] = None

        self.clear_trails_next = False

        # Internal accumulators
        self._accumulator = 0.0
        self._trail_step_counter = 0

        self.physics = NBodyPhysics(self.softening)

    def set_softening(self, val: float):
        with self.lock:
            self.softening = float(val)
            self.physics.set_softening(self.softening)

    def set_time_scale(self, s: float):
        with self.lock:
            self.time_scale = float(s)

    def set_trail_length(self, n: int):
        with self.lock:
            self.trail_length = int(n)
            for b in self.bodies:
                b.trail = deque(b.trail, maxlen=self.trail_length)

    def clear_trails(self):
        with self.lock:
            for b in self.bodies:
                b.trail.clear()

    def add_body(self, body: Body):
        with self.lock:
            body.trail = deque(maxlen=self.trail_length)
            self.bodies.append(body)
            self.selected_index = len(self.bodies) - 1

    def delete_selected(self):
        with self.lock:
            if self.selected_index is not None and 0 <= self.selected_index < len(self.bodies):
                del self.bodies[self.selected_index]
                if not self.bodies:
                    self.selected_index = None
                else:
                    self.selected_index = min(self.selected_index, len(self.bodies) - 1)

    def select_body_at(self, world_pos: Tuple[float, float], pick_radius_m: float) -> Optional[int]:
        with self.lock:
            idx = None
            min_d = float("inf")
            for i, b in enumerate(self.bodies):
                d = vec_len(vec_sub(b.position, world_pos))
                # Use larger of visual radius and pick_radius_m for usability
                pr = max(b.radius * 2, pick_radius_m)
                if d < pr and d < min_d:
                    min_d = d
                    idx = i
            self.selected_index = idx
            return idx

    def step_physics(self, dt_real_seconds: float):
        """
        Advance physics using RK4 with adaptive substeps per frame.
        This avoids huge iteration counts at extreme speeds while preserving stability.
        """
        with self.lock:
            # Use a safe minimum time scale to avoid a frozen simulation if UI sets 0
            effective_scale = self.time_scale if self.time_scale and self.time_scale > 0.0 else 1.0
            sim_dt = dt_real_seconds * effective_scale  # total simulation time to advance this frame

            if sim_dt <= 0:
                return

            # Determine number of substeps based on desired BASE_DT, but clamp for performance
            max_steps = 1000  # tighter ceiling for smoother rendering
            steps = int(sim_dt / BASE_DT)
            steps = clamp(steps, 1, max_steps)
            dt_per = sim_dt / steps

            for _ in range(steps):
                if self.integrator == "RK4":
                    self.physics.rk4_integration_step(self.bodies, dt_per)
                else:
                    self.physics.rk4_integration_step(self.bodies, dt_per)
                if self.enable_collisions:
                    msg = handle_collisions(
                        self.bodies,
                        CollisionSettings(
                            enable=self.enable_collisions,
                            mode=self.collision_mode,
                            restitution=self.restitution,
                            radius_scale=self.collision_radius_scale,
                        ),
                    )
                    if msg:
                        self.last_collision_msg = msg
                # Throttle trail sampling to reduce draw cost
                self._trail_step_counter = (self._trail_step_counter + 1) % 3
                if self.show_trails and self._trail_step_counter == 0:
                    for b in self.bodies:
                        b.add_trail_point()

    def get_selected_body(self) -> Optional[Body]:
        with self.lock:
            if self.selected_index is not None and 0 <= self.selected_index < len(self.bodies):
                return self.bodies[self.selected_index]
            return None

    def set_selected_position(self, pos: Tuple[float, float]):
        with self.lock:
            b = self.get_selected_body()
            if b:
                b.position = pos

    def set_selected_velocity(self, vel: Tuple[float, float]):
        with self.lock:
            b = self.get_selected_body()
            if b:
                b.velocity = vel

    def replace_bodies(self, new_bodies: List[Body]):
        with self.lock:
            self.bodies = new_bodies
            for b in self.bodies:
                b.trail = deque(maxlen=self.trail_length)
            self.selected_index = 0 if self.bodies else None

    # Collision handling moved to core.collisions.handle_collisions

# ============================================================
# Presets and Templates
# ============================================================

def circular_orbit_velocity(GM, r):
    return math.sqrt(GM / r)  # m/s

def template_solar_system_scaled() -> List[Body]:
    """
    Sun + Earth + Moon + Mars + Jupiter simplified.
    Distances are real SI; velocities computed for circular orbits around Sun where applicable.
    The camera will auto-zoom to fit after loading.
    """
    bodies = []

    # Sun
    sun = Body(
        name="Sun",
        mass=SOLAR_MASS,
        radius=SOLAR_RADIUS,
        position=(0.0, 0.0),
        velocity=(0.0, 0.0),
        color=(255, 204, 0),
    )
    bodies.append(sun)

    # Earth
    r_earth = 1.495978707e11  # 1 AU in meters
    v_earth = circular_orbit_velocity(G * sun.mass, r_earth)
    earth = Body(
        name="Earth",
        mass=EARTH_MASS,
        radius=EARTH_RADIUS,
        position=(r_earth, 0.0),
        velocity=(0.0, v_earth),
        color=(100, 149, 237),
    )
    bodies.append(earth)

    # Moon
    r_moon = 384400e3  # meters
    v_moon_rel = circular_orbit_velocity(G * earth.mass, r_moon)
    moon = Body(
        name="Moon",
        mass=7.34767309e22,
        radius=1.7374e6,
        position=(earth.position[0] + r_moon, earth.position[1]),
        velocity=(earth.velocity[0], earth.velocity[1] + v_moon_rel),
        color=(180, 180, 200),
    )
    bodies.append(moon)

    # Mars
    m_mars = 6.4171e23
    r_mars = 2.279e11
    v_mars = circular_orbit_velocity(G * sun.mass, r_mars)
    mars = Body(
        name="Mars",
        mass=m_mars,
        radius=3.3895e6,
        position=(r_mars, 0.0),
        velocity=(0.0, v_mars),
        color=(188, 39, 50),
    )
    bodies.append(mars)

    # Jupiter
    m_jupiter = 1.898e27
    r_jupiter = 7.785e11
    v_jupiter = circular_orbit_velocity(G * sun.mass, r_jupiter)
    jupiter = Body(
        name="Jupiter",
        mass=m_jupiter,
        radius=6.9911e7,
        position=(r_jupiter, 0.0),
        velocity=(0.0, v_jupiter),
        color=(210, 180, 140),
    )
    bodies.append(jupiter)

    return bodies

def template_three_body_lagrange() -> List[Body]:
    """
    Classic 3-body Lagrange triangular configuration (equal masses on an equilateral triangle).
    All rotate around the center of mass with angular velocity ω^2 = G*(m1+m2+m3)/R^3.
    """
    bodies = []
    m = 5e24  # about Earth mass scale (but slightly smaller)
    R = 1.5e9  # distance from CM to each mass

    # Use a corrected angular frequency factor for equal-mass Lagrange triangle (approximate)
    # omega^2 ≈ G*m / (R^3)
    omega = math.sqrt(G * m / (R ** 3))
    v = omega * R

    p1 = (R, 0.0)
    p2 = (-R / 2, +R * math.sqrt(3) / 2)
    p3 = (-R / 2, -R * math.sqrt(3) / 2)

    def tangent_velocity(pos):
        t = (-pos[1], pos[0])
        t = vec_norm(t)
        return (t[0] * v, t[1] * v)

    bodies.append(Body("A", m, 6.0e6, p1, tangent_velocity(p1), (255, 120, 120)))
    bodies.append(Body("B", m, 6.0e6, p2, tangent_velocity(p2), (120, 255, 120)))
    bodies.append(Body("C", m, 6.0e6, p3, tangent_velocity(p3), (120, 120, 255)))

    return bodies

def template_three_body_figure_eight() -> List[Body]:
    """Classic equal-mass figure-eight periodic solution (Chenciner-Montgomery), scaled to SI.
    Dimensionless initial conditions (G=1, m=1):
    r1=(-0.97000436, 0.24308753), r2=(0.97000436,-0.24308753), r3=(0,0)
    v1=(0.4662036850, 0.4323657300), v2=(0.4662036850, 0.4323657300), v3=(-0.93240737,-0.86473146)
    We scale by choosing mass m_si and length L, then time T = sqrt(L^3/(G*m_si)), velocity V=L/T.
    """
    m_si = 5e24
    L = 1.0e9  # meters
    V = math.sqrt(G * m_si / L)

    r1 = (-0.97000436, 0.24308753)
    r2 = (0.97000436, -0.24308753)
    r3 = (0.0, 0.0)
    v1 = (0.4662036850, 0.4323657300)
    v2 = (0.4662036850, 0.4323657300)
    v3 = (-0.93240737, -0.86473146)

    bodies = []
    bodies.append(Body("A", m_si, 6.0e6, (r1[0]*L, r1[1]*L), (v1[0]*V, v1[1]*V), (255,120,120)))
    bodies.append(Body("B", m_si, 6.0e6, (r2[0]*L, r2[1]*L), (v2[0]*V, v2[1]*V), (120,255,120)))
    bodies.append(Body("C", m_si, 6.0e6, (r3[0]*L, r3[1]*L), (v3[0]*V, v3[1]*V), (120,120,255)))
    return bodies

def template_empty() -> List[Body]:
    return []

# ============================================================
# Pygame Renderer Thread
# ============================================================

class PygameRenderer(threading.Thread):
    """
    Pygame loop: draws bodies, trails, velocity vectors, grid.
    Handles selection, dragging, camera panning and zoom.
    """
    def __init__(self, sim: SimulationController):
        super().__init__(daemon=True)
        self.sim = sim
        self.camera = Camera2D(center=(0.0, 0.0))
        self.surface = None
        self.clock = None
        self.dragging_body = False
        self.dragging_background = False
        self.drag_start_screen = (0, 0)
        self.last_mouse_screen = (0, 0)
        self.pan_speed_keys = 600  # pixels per second
        self.velocity_preview = True
        self.running = True

    def auto_frame_camera(self):
        """
        Adjust camera to fit all bodies into view with margin.
        """
        with self.sim.lock:
            if not self.sim.bodies:
                self.camera.center = [0.0, 0.0]
                self.camera.mpp = DEFAULT_METERS_PER_PIXEL
                return
            xs = [b.position[0] for b in self.sim.bodies]
            ys = [b.position[1] for b in self.sim.bodies]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        cx = (minx + maxx) / 2
        cy = (miny + maxy) / 2
        width_m = (maxx - minx) * 1.3 + 1.0
        height_m = (maxy - miny) * 1.3 + 1.0
        if width_m == 0: width_m = 1.0
        if height_m == 0: height_m = 1.0
        mpp_x = width_m / max(self.camera.viewport_size[0], 1)
        mpp_y = height_m / max(self.camera.viewport_size[1], 1)
        self.camera.center = [cx, cy]
        self.camera.mpp = clamp(max(mpp_x, mpp_y), MIN_METERS_PER_PIXEL, MAX_METERS_PER_PIXEL)

    def run(self):
        pygame.init()
        pygame.display.set_caption("Orbit Simulator - Viewport")
        self.surface = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.RESIZABLE)
        self.camera.set_viewport_size(VIEW_WIDTH, VIEW_HEIGHT)
        self.clock = pygame.time.Clock()
        self.auto_frame_camera()

        last_time = time.perf_counter()
        while self.running and self.sim.running:
            now = time.perf_counter()
            real_dt = now - last_time
            last_time = now

            # Input handling
            self.handle_events(real_dt)

            # Physics step
            with self.sim.lock:
                playing = self.sim.playing
            if playing:
                self.sim.step_physics(real_dt)

            # Draw
            self.draw()

            # Limit FPS
            self.clock.tick(60)

        pygame.quit()

    def handle_events(self, real_dt):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            self.camera.pan_pixels(self.pan_speed_keys * real_dt, 0)
        if keys[pygame.K_RIGHT]:
            self.camera.pan_pixels(-self.pan_speed_keys * real_dt, 0)
        if keys[pygame.K_UP]:
            self.camera.pan_pixels(0, self.pan_speed_keys * real_dt)
        if keys[pygame.K_DOWN]:
            self.camera.pan_pixels(0, -self.pan_speed_keys * real_dt)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.sim.running = False
                self.running = False

            elif event.type == pygame.VIDEORESIZE:
                self.surface = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                self.camera.set_viewport_size(event.w, event.h)

            elif event.type == pygame.MOUSEWHEEL:
                factor = 1.1 if event.y > 0 else 1.0/1.1
                self.camera.zoom(factor, pygame.mouse.get_pos())

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # left click
                    mouse = pygame.mouse.get_pos()
                    world = self.camera.screen_to_world(mouse)
                    # Start body drag if clicked near a body; otherwise, start background drag
                    idx = self.sim.select_body_at(world, pick_radius_m=self.camera.mpp * 10)
                    if idx is not None:
                        self.dragging_body = True
                    else:
                        self.dragging_background = True
                        self.drag_start_screen = mouse
                elif event.button in (2, 3):  # middle/right also pan
                    self.dragging_background = True
                    self.drag_start_screen = pygame.mouse.get_pos()

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.dragging_body = False
                    # body stays at new position; velocity unchanged
                if event.button in (2, 3, 1):
                    self.dragging_background = False

            elif event.type == pygame.MOUSEMOTION:
                mouse = pygame.mouse.get_pos()
                if self.dragging_body:
                    # Move selected body to cursor world position
                    world = self.camera.screen_to_world(mouse)
                    self.sim.set_selected_position(world)
                elif self.dragging_background:
                    dx = mouse[0] - self.drag_start_screen[0]
                    dy = mouse[1] - self.drag_start_screen[1]
                    self.camera.pan_pixels(dx, dy)
                    self.drag_start_screen = mouse

        self.last_mouse_screen = pygame.mouse.get_pos()

    def draw_grid(self, surf):
        w, h = self.camera.viewport_size
        mpp = self.camera.mpp

        # Grid scale: every grid spacing in meters
        # Choose scale so grid lines at approximately 100 pixels apart
        target_px = 100
        spacing_m = mpp * target_px
        # Round spacing to 1-2-5 sequence
        pow10 = 10 ** math.floor(math.log10(spacing_m)) if spacing_m > 0 else 1
        mant = spacing_m / pow10
        if mant < 2:
            spacing = 1 * pow10
        elif mant < 5:
            spacing = 2 * pow10
        else:
            spacing = 5 * pow10

        # Fine grid (1/5 spacing)
        fine = spacing / 5

        # Determine world bounds of screen
        top_left = self.camera.screen_to_world((0, 0))
        bottom_right = self.camera.screen_to_world((w, h))
        x0 = math.floor(top_left[0] / fine) * fine
        y0 = math.floor(top_left[1] / fine) * fine

        # Draw fine grid
        x = x0
        while x <= bottom_right[0]:
            sx, _ = self.camera.world_to_screen((x, 0))
            pygame.draw.line(surf, GRID_FINE_COLOR, (sx, 0), (sx, h), 1)
            x += fine
        y = y0
        while y <= bottom_right[1]:
            _, sy = self.camera.world_to_screen((0, y))
            pygame.draw.line(surf, GRID_FINE_COLOR, (0, sy), (w, sy), 1)
            y += fine

        # Draw main grid
        x = x0
        while x <= bottom_right[0]:
            sx, _ = self.camera.world_to_screen((x, 0))
            pygame.draw.line(surf, GRID_COLOR, (sx, 0), (sx, h), 1)
            x += spacing
        y = y0
        while y <= bottom_right[1]:
            _, sy = self.camera.world_to_screen((0, y))
            pygame.draw.line(surf, GRID_COLOR, (0, sy), (w, sy), 1)
            y += spacing

    def draw(self):
        surf = self.surface
        surf.fill(BACKGROUND_COLOR)

        self.draw_grid(surf)

        # Copy bodies snapshot for consistency during draw
        with self.sim.lock:
            bodies = list(self.sim.bodies)
            selected_idx = self.sim.selected_index
            show_trails = self.sim.show_trails
            show_vel = self.sim.show_velocity_vectors

        # Draw trails
        if show_trails:
            for i, b in enumerate(bodies):
                if len(b.trail) > 1:
                    pts = []
                    for p in b.trail:
                        sp = self.camera.world_to_screen(p)
                        sp_s = _safe_point(sp)
                        if sp_s:
                            pts.append(sp_s)
                    if len(pts) > 1:
                        try:
                            pygame.draw.aalines(surf, b.color, False, pts)
                        except Exception:
                            pass

        # Draw bodies
        for i, b in enumerate(bodies):
            screen_pos = self.camera.world_to_screen(b.position)
            screen_pos_s = _safe_point(screen_pos)
            # Visual radius in pixels: ensure it's not too tiny
            vis_r = max(2, int(max(b.radius / self.camera.mpp, 2)))
            if vis_r > 50:
                vis_r = 50  # limit large circles for performance
            if screen_pos_s:
                try:
                    gfxdraw.filled_circle(surf, screen_pos_s[0], screen_pos_s[1], vis_r, b.color)
                    gfxdraw.aacircle(surf, screen_pos_s[0], screen_pos_s[1], vis_r, (0, 0, 0))
                except Exception:
                    pass

                # Selection ring
                if i == selected_idx:
                    sel_r = vis_r + 4
                    try:
                        gfxdraw.aacircle(surf, screen_pos_s[0], screen_pos_s[1], sel_r, SELECTION_COLOR)
                    except Exception:
                        pass

            # Velocity vector
            if show_vel:
                v = b.velocity
                v_len = vec_len(v)
                if v_len > 0:
                    # scale velocity vector to reasonable pixel length
                    scale = 1.0 / (self.camera.mpp * 3.0)
                    end_world = (b.position[0] + v[0] * scale, b.position[1] + v[1] * scale)
                    end_screen = self.camera.world_to_screen(end_world)
                    start_s = _safe_point(screen_pos)
                    end_s = _safe_point(end_screen)
                    if start_s and end_s:
                        try:
                            pygame.draw.line(surf, VELOCITY_VECTOR_COLOR, start_s, end_s, 2)
                        except Exception:
                            pass
                        # arrow head
                        draw_arrow_head(surf, end_s, start_s, VELOCITY_VECTOR_COLOR)

        # Velocity preview while dragging a body (keeps current velocity)
        # Already covered by show_vel; if dragging, highlight could be enough.

        # HUD text
        draw_text(surf, "Left-drag: move body | Right/Middle-drag: pan | Wheel: zoom | Arrows: pan | Space: Pause/Play", 10, 10, (200, 200, 200))
        with self.sim.lock:
            ts = self.sim.time_scale
            playing = self.sim.playing
        draw_text(surf, f"Speed: {ts:.0f}x  [{'Playing' if playing else 'Paused'}]", 10, 30, (200, 200, 200))

        pygame.display.flip()

_cached_font = None

def draw_text(surface, text, x, y, color):
    global _cached_font
    if not pygame.font.get_init():
        try:
            pygame.font.init()
        except Exception:
            pass
    if _cached_font is None:
        try:
            _cached_font = pygame.font.SysFont("consolas", 16)
        except Exception:
            _cached_font = pygame.font.Font(None, 16)
    img = _cached_font.render(text, True, color)
    surface.blit(img, (x, y))

SAFE_COORD_LIMIT = 30000

def _safe_point(pt):
    try:
        x, y = int(pt[0]), int(pt[1])
    except Exception:
        return None
    if -SAFE_COORD_LIMIT <= x <= SAFE_COORD_LIMIT and -SAFE_COORD_LIMIT <= y <= SAFE_COORD_LIMIT:
        return (x, y)
    return None

def draw_arrow_head(surface, tip, tail, color):
    # Small triangle for arrow head
    tip_s = _safe_point(tip)
    tail_s = _safe_point(tail)
    if tip_s is None or tail_s is None:
        return
    dx = tip_s[0] - tail_s[0]
    dy = tip_s[1] - tail_s[1]
    ang = math.atan2(dy, dx)
    size = 8
    left = (tip_s[0] - size * math.cos(ang - math.pi / 6), tip_s[1] - size * math.sin(ang - math.pi / 6))
    right = (tip_s[0] - size * math.cos(ang + math.pi / 6), tip_s[1] - size * math.sin(ang + math.pi / 6))
    left_s = _safe_point(left)
    right_s = _safe_point(right)
    if left_s and right_s:
        try:
            pygame.draw.polygon(surface, color, [tip_s, left_s, right_s])
        except Exception:
            pass

# ============================================================
# Dear PyGui UI
# ============================================================

class UI:
    """
    Dear PyGui interface: add body form, SI readouts, presets, simulation controls.
    """
    def __init__(self, sim: SimulationController, renderer: PygameRenderer):
        self.sim = sim
        self.renderer = renderer

        # IDs for widgets
        self.mass_value_id = None
        self.mass_unit_id = None
        self.mass_si_label_id = None

        self.radius_value_id = None
        self.radius_unit_id = None
        self.radius_si_label_id = None

        self.pos_x_id = None
        self.pos_y_id = None
        self.vel_x_id = None
        self.vel_y_id = None
        self.color_id = None
        self.name_id = None

        self.status_msg_id = None

        # Body library widgets
        self.lib_combo_id = None
        self.lib_pos_x_id = None
        self.lib_pos_y_id = None
        self.lib_vel_x_id = None
        self.lib_vel_y_id = None

        self.body_list_id = None
        self.sel_pos_x_id = None
        the_sel_pos_y = None
        self.sel_vel_x_id = None
        self.sel_vel_y_id = None

        # Edit Selected Body fields
        self.edit_name_id = None
        self.edit_mass_value_id = None
        self.edit_mass_unit_id = None
        self.edit_mass_si_label_id = None
        self.edit_radius_value_id = None
        self.edit_radius_unit_id = None
        self.edit_radius_si_label_id = None
        self.edit_pos_x_id = None
        self.edit_pos_y_id = None
        self.edit_color_id = None

        # Track selection to avoid overwriting edits during typing
        self._last_edit_selected_idx = None

        self._build_ui()

        # Schedule initial setup and periodic UI sync without using timers (for wider DPG version support)
        dpg.set_frame_callback(1, self._post_setup)
        self._schedule_sync()

    def _post_setup(self):
        # Load default scene (Empty)
        self.load_template("Empty custom")

    def _schedule_sync(self):
        """Reschedule the periodic sync callback using frame callbacks (approx ~10Hz)."""
        try:
            current = dpg.get_frame_count()
        except Exception:
            current = 0
        # schedule next sync ~ every 6 frames (~100ms at 60 FPS)
        dpg.set_frame_callback(current + 6, self._sync_ui_with_sim)

    # -----------------------
    # Unit conversion helpers
    # -----------------------

    @staticmethod
    def mass_to_si(val: float, unit: str) -> float:
        if unit == "kg":
            return val
        elif unit == "Earth mass":
            return val * EARTH_MASS
        elif unit == "Solar mass":
            return val * SOLAR_MASS
        else:
            return val

    @staticmethod
    def radius_to_si(val: float, unit: str) -> float:
        if unit == "m":
            return val
        elif unit == "Earth radius":
            return val * EARTH_RADIUS
        elif unit == "Solar radius":
            return val * SOLAR_RADIUS
        else:
            return val

    def _update_mass_si_label(self):
        val = try_float(dpg.get_value(self.mass_value_id))
        unit = dpg.get_value(self.mass_unit_id)
        if val is None:
            dpg.set_value(self.mass_si_label_id, "Mass (SI): invalid")
            return
        si = self.mass_to_si(val, unit)
        dpg.set_value(self.mass_si_label_id, f"Mass (SI): {si:.3e} kg")

    def _update_radius_si_label(self):
        val = try_float(dpg.get_value(self.radius_value_id))
        unit = dpg.get_value(self.radius_unit_id)
        if val is None:
            dpg.set_value(self.radius_si_label_id, "Radius (SI): invalid")
            return
        si = self.radius_to_si(val, unit)
        dpg.set_value(self.radius_si_label_id, f"Radius (SI): {si:.3e} m")

    def _update_edit_mass_si_label(self):
        val = try_float(dpg.get_value(self.edit_mass_value_id))
        unit = dpg.get_value(self.edit_mass_unit_id)
        if val is None:
            dpg.set_value(self.edit_mass_si_label_id, "Mass (SI): invalid")
            return
        si = self.mass_to_si(val, unit)
        dpg.set_value(self.edit_mass_si_label_id, f"Mass (SI): {si:.3e} kg")

    def _update_edit_radius_si_label(self):
        val = try_float(dpg.get_value(self.edit_radius_value_id))
        unit = dpg.get_value(self.edit_radius_unit_id)
        if val is None:
            dpg.set_value(self.edit_radius_si_label_id, "Radius (SI): invalid")
            return
        si = self.radius_to_si(val, unit)
        dpg.set_value(self.edit_radius_si_label_id, f"Radius (SI): {si:.3e} m")

    # -----------------------
    # UI Construction
    # -----------------------

    def _build_ui(self):
        dpg.create_context()
        dpg.create_viewport(title='Orbit Simulator - Controls', width=520, height=800)

        # Dear PyGui: no explicit default font registration needed in current versions.

        with dpg.window(label="Controls", width=500, height=780, pos=(10, 10), tag="main_window"):
            with dpg.group(horizontal=True):
                dpg.add_text("Preset:")
                # Build template list from templates/ JSONs; fallback to built-ins if missing
                self._template_map = {}
                try:
                    for fn, display in list_json_templates():
                        self._template_map[display] = fn
                except Exception:
                    pass
                preset_items = list(self._template_map.keys()) or [
                    "Solar System (scaled)", "Classic 3-body (Figure-eight)", "Classic 3-body (Lagrange)", "Empty custom"
                ]
                if "Empty custom" in preset_items:
                    default_item = "Empty custom"
                else:
                    default_item = preset_items[0] if preset_items else "Solar System (scaled)"
                dpg.add_combo(preset_items,
                              default_value=default_item,
                              width=260,
                              callback=lambda s, a, u: self.load_template(a),
                              tag="preset_combo")
                dpg.add_button(label="Load", callback=lambda: self.load_template(dpg.get_value("preset_combo")))
                dpg.add_button(label="Auto-fit Camera", callback=self.renderer.auto_frame_camera)

            dpg.add_separator()

            dpg.add_text("Add New Body")
            with dpg.group():
                self.name_id = dpg.add_input_text(label="Name", default_value="Body", width=200)
                with dpg.group(horizontal=True):
                    self.mass_value_id = dpg.add_input_text(label="Mass", default_value="1.0", width=120,
                                                            callback=lambda s, a, u: self._update_mass_si_label())
                    self.mass_unit_id = dpg.add_combo(label="Mass Unit",
                                                      items=["kg", "Earth mass", "Solar mass"],
                                                      default_value="kg",
                                                      width=120,
                                                      callback=lambda s, a, u: self._update_mass_si_label())
                self.mass_si_label_id = dpg.add_text("Mass (SI): 1.000e+00 kg")

                with dpg.group(horizontal=True):
                    self.radius_value_id = dpg.add_input_text(label="Radius", default_value="1.0", width=120,
                                                              callback=lambda s, a, u: self._update_radius_si_label())
                    self.radius_unit_id = dpg.add_combo(label="Radius Unit",
                                                        items=["m", "Earth radius", "Solar radius"],
                                                        default_value="m",
                                                        width=120,
                                                        callback=lambda s, a, u: self._update_radius_si_label())
                self.radius_si_label_id = dpg.add_text("Radius (SI): 1.000e+00 m")

                with dpg.group(horizontal=True):
                    self.pos_x_id = dpg.add_input_text(label="Position X (m)", default_value="0.0", width=150)
                    self.pos_y_id = dpg.add_input_text(label="Position Y (m)", default_value="0.0", width=150)

                with dpg.group(horizontal=True):
                    self.vel_x_id = dpg.add_input_text(label="Velocity X (m/s)", default_value="0.0", width=150)
                    self.vel_y_id = dpg.add_input_text(label="Velocity Y (m/s)", default_value="0.0", width=150)

                self.color_id = dpg.add_color_edit(default_value=(200, 200, 255, 255), label="Color", no_alpha=False, width=220)

                with dpg.group(horizontal=True):
                    dpg.add_button(label="Add Body", callback=self._on_add_body_clicked)
                    dpg.add_button(label="Delete Selected", callback=lambda: (self.sim.delete_selected(), self._refresh_body_list(), self._set_status("Deleted selected body.")))
                self.status_msg_id = dpg.add_text("")

            dpg.add_separator()

            # Body Library loader
            dpg.add_text("Body Library")
            with dpg.group():
                # Build library list once; add refresh button to rescan
                try:
                    lib_items = list_celestial_bodies()
                except Exception:
                    lib_items = []
                display_items = lib_items if lib_items else ["No bodies found (add JSONs to celestial_bodies/)"]
                self.lib_combo_id = dpg.add_combo(items=display_items, width=300,
                                                  default_value=(display_items[0] if display_items else ""))
                with dpg.group(horizontal=True):
                    self.lib_pos_x_id = dpg.add_input_text(label="Pos X (m)", default_value="0.0", width=150)
                    self.lib_pos_y_id = dpg.add_input_text(label="Pos Y (m)", default_value="0.0", width=150)
                with dpg.group(horizontal=True):
                    self.lib_vel_x_id = dpg.add_input_text(label="Vel X (m/s)", default_value="0.0", width=150)
                    self.lib_vel_y_id = dpg.add_input_text(label="Vel Y (m/s)", default_value="0.0", width=150)
                with dpg.group(horizontal=True):
                    dpg.add_button(label="Insert From Library", callback=self._on_insert_library_body)
                    dpg.add_button(label="Refresh Library", callback=self._on_refresh_library)

            dpg.add_separator()

            dpg.add_text("Current Bodies")
            self.body_list_id = dpg.add_listbox(items=[], width=480, num_items=6, callback=self._on_select_body)

            with dpg.group(horizontal=True):
                self.sel_pos_x_id = dpg.add_input_text(label="Sel Pos X (m)", readonly=True, width=150)
                self.sel_pos_y_id = dpg.add_input_text(label="Sel Pos Y (m)", readonly=True, width=150)
            with dpg.group(horizontal=True):
                self.sel_vel_x_id = dpg.add_input_text(label="Sel Vel X (m/s)", width=150)
                self.sel_vel_y_id = dpg.add_input_text(label="Sel Vel Y (m/s)", width=150)
                dpg.add_button(label="Apply Velocity", callback=self._apply_selected_velocity)

            dpg.add_separator()

            dpg.add_text("Edit Selected Body")
            with dpg.group():
                with dpg.group(horizontal=True):
                    self.edit_name_id = dpg.add_input_text(label="Name", default_value="", width=200)
                with dpg.group(horizontal=True):
                    self.edit_mass_value_id = dpg.add_input_text(label="Mass", default_value="", width=120,
                                                                 callback=lambda s, a, u: self._update_edit_mass_si_label())
                    self.edit_mass_unit_id = dpg.add_combo(label="Mass Unit",
                                                           items=["kg", "Earth mass", "Solar mass"],
                                                           default_value="kg",
                                                           width=120,
                                                           callback=lambda s, a, u: self._update_edit_mass_si_label())
                self.edit_mass_si_label_id = dpg.add_text("Mass (SI): ")
                with dpg.group(horizontal=True):
                    self.edit_radius_value_id = dpg.add_input_text(label="Radius", default_value="", width=120,
                                                                   callback=lambda s, a, u: self._update_edit_radius_si_label())
                    self.edit_radius_unit_id = dpg.add_combo(label="Radius Unit",
                                                             items=["m", "Earth radius", "Solar radius"],
                                                             default_value="m",
                                                             width=120,
                                                             callback=lambda s, a, u: self._update_edit_radius_si_label())
                self.edit_radius_si_label_id = dpg.add_text("Radius (SI): ")
                with dpg.group(horizontal=True):
                    self.edit_pos_x_id = dpg.add_input_text(label="Pos X (m)", default_value="", width=150)
                    self.edit_pos_y_id = dpg.add_input_text(label="Pos Y (m)", default_value="", width=150)
                self.edit_color_id = dpg.add_color_edit(default_value=(200, 200, 255, 255), label="Color", no_alpha=False, width=220)
                dpg.add_button(label="Apply Body Edits", callback=self._apply_selected_body_edits)

            dpg.add_separator()

            dpg.add_text("Simulation Controls")
            with dpg.group(horizontal=True):
                dpg.add_button(label="Play/Pause", callback=self._toggle_play)
                dpg.add_button(label="Step ▶", callback=self._step_once)
                dpg.add_checkbox(label="Trails", default_value=True, callback=self._toggle_trails)
                dpg.add_input_int(label="Trail length", default_value=250, min_value=10, max_value=10000, width=100,
                                  callback=self._on_trail_length, tag="trail_length_input")
            with dpg.group(horizontal=True):
                dpg.add_text("Speed (x real-time):")
                dpg.add_slider_float(min_value=0.0, max_value=1000000000.0, default_value=10000000.0, width=300,
                                     callback=lambda s, a, u: self.sim.set_time_scale(float(a) if a is not None else 1.0), tag="speed_slider")
            with dpg.group(horizontal=True):
                dpg.add_text("Softening (m):")
                dpg.add_slider_float(min_value=0.0, max_value=1e9, default_value=DEFAULT_SOFTENING, width=300,
                                     callback=lambda s, a, u: self.sim.set_softening(a), tag="softening_slider")
            dpg.add_text("Collisions")
            with dpg.group(horizontal=True):
                dpg.add_combo(["Merge", "Elastic", "Accretion"], default_value="Accretion", width=150,
                               callback=lambda s, a, u: self._set_collision_mode(a), tag="collision_mode_combo")
                dpg.add_slider_float(label="Restitution", min_value=0.0, max_value=1.0, default_value=0.2, width=200,
                                     callback=lambda s, a, u: self._set_restitution(a), tag="restitution_slider")
                dpg.add_checkbox(label="Enable", default_value=True, callback=lambda s, a, u: self._toggle_collisions(a))
                dpg.add_slider_float(label="Radius scale", min_value=0.1, max_value=5.0, default_value=1.0, width=200,
                                     callback=lambda s, a, u: self._set_collision_radius_scale(a), tag="collision_radius_slider")
            with dpg.group(horizontal=True):
                dpg.add_text("Integrator:")
                dpg.add_combo(["RK4"], default_value="RK4", width=120, callback=lambda s, a, u: self._set_integrator(a))
                dpg.add_checkbox(label="Show velocity vectors", default_value=True,
                                 callback=lambda s, a, u: self._toggle_velocity_vectors(a))

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main_window", True)

    # -----------------------
    # UI Callbacks
    # -----------------------

    def _set_status(self, msg: str, color=(180, 220, 180)):
        dpg.set_value(self.status_msg_id, msg)
        dpg.configure_item(self.status_msg_id, color=color)

    def _set_error(self, msg: str):
        dpg.set_value(self.status_msg_id, msg)
        dpg.configure_item(self.status_msg_id, color=(255, 120, 120))

    def _on_add_body_clicked(self):
        name = dpg.get_value(self.name_id).strip() or "Body"
        mass_text = dpg.get_value(self.mass_value_id)
        radius_text = dpg.get_value(self.radius_value_id)
        pos_x_text = dpg.get_value(self.pos_x_id)
        pos_y_text = dpg.get_value(self.pos_y_id)
        vel_x_text = dpg.get_value(self.vel_x_id)
        vel_y_text = dpg.get_value(self.vel_y_id)
        color_rgba = dpg.get_value(self.color_id)  # RGBA 0-255

        mass_val = try_float(mass_text)
        radius_val = try_float(radius_text)
        pos_x = try_float(pos_x_text)
        pos_y = try_float(pos_y_text)
        vel_x = try_float(vel_x_text)
        vel_y = try_float(vel_y_text)
        if None in (mass_val, radius_val, pos_x, pos_y, vel_x, vel_y):
            self._set_error("Invalid numeric input. Please correct fields highlighted.")
            return

        mass_si = self.mass_to_si(mass_val, dpg.get_value(self.mass_unit_id))
        radius_si = self.radius_to_si(radius_val, dpg.get_value(self.radius_unit_id))

        # Validate mass and radius
        if mass_si <= 0:
            self._set_error("Mass must be positive and non-zero.")
            return
        if radius_si <= 0:
            self._set_error("Radius must be positive and non-zero.")
            return
        # Reasonable bounds
        if mass_si > 100 * SOLAR_MASS:
            self._set_error("Mass too large ( > 100 Solar masses ).")
            return
        if radius_si > 100 * SOLAR_RADIUS:
            self._set_error("Radius too large ( > 100 Solar radii ).")
            return

        color = (int(color_rgba[0]), int(color_rgba[1]), int(color_rgba[2]))

        body = Body(
            name=name,
            mass=mass_si,
            radius=radius_si,
            position=(pos_x, pos_y),
            velocity=(vel_x, vel_y),
            color=color
        )
        self.sim.add_body(body)
        self._refresh_body_list()
        self._set_status(f"Added body '{name}'.")
        # Update SI labels
        self._update_mass_si_label()
        self._update_radius_si_label()

    def _refresh_body_list(self):
        with self.sim.lock:
            items = [b.name for b in self.sim.bodies]
            sel = self.sim.selected_index if self.sim.selected_index is not None else 0
        dpg.configure_item(self.body_list_id, items=items)
        if items:
            dpg.set_value(self.body_list_id, items[clamp(sel, 0, len(items) - 1)])

    def _on_select_body(self, sender, app_data, user_data):
        # app_data is selected string
        with self.sim.lock:
            names = [b.name for b in self.sim.bodies]
            if app_data in names:
                idx = names.index(app_data)
                self.sim.selected_index = idx
        # Populate edit fields only when selection changes
        self._populate_edit_fields_from_selected()
        with self.sim.lock:
            self._last_edit_selected_idx = self.sim.selected_index

    def _on_refresh_library(self):
        # Rescan celestial_bodies directory and update combo items
        try:
            lib_items = list_celestial_bodies()
        except Exception:
            lib_items = []
        display_items = lib_items if lib_items else ["No bodies found (add JSONs to celestial_bodies/)"]
        dpg.configure_item(self.lib_combo_id, items=display_items)
        if display_items:
            dpg.set_value(self.lib_combo_id, display_items[0])
        self._set_status("Body library refreshed.")

    def _on_insert_library_body(self):
        choice = dpg.get_value(self.lib_combo_id)
        if not choice or choice.startswith("No bodies"):
            self._set_error("No library body selected.")
            return
        pos_x = try_float(dpg.get_value(self.lib_pos_x_id))
        pos_y = try_float(dpg.get_value(self.lib_pos_y_id))
        vel_x = try_float(dpg.get_value(self.lib_vel_x_id))
        vel_y = try_float(dpg.get_value(self.lib_vel_y_id))
        if None in (pos_x, pos_y, vel_x, vel_y):
            self._set_error("Invalid position/velocity for library insertion.")
            return
        body = None
        try:
            body = load_celestial_body(choice, position=(pos_x, pos_y), velocity=(vel_x, vel_y))
        except Exception:
            body = None
        if body is None:
            self._set_error(f"Failed to load body '{choice}'.")
            return
        self.sim.add_body(body)
        self._refresh_body_list()
        self._set_status(f"Inserted library body '{body.name}'.")

    def _apply_selected_velocity(self):
        vx = try_float(dpg.get_value(self.sel_vel_x_id))
        vy = try_float(dpg.get_value(self.sel_vel_y_id))
        if None in (vx, vy):
            self._set_error("Invalid velocity input for selected body.")
            return
        self.sim.set_selected_velocity((vx, vy))
        self._set_status("Updated selected body's velocity.")

    def _populate_edit_fields_from_selected(self):
        b = self.sim.get_selected_body()
        if not b:
            return
        # Name
        dpg.set_value(self.edit_name_id, b.name)
        # Mass and radius in kg/m into fields (default units kg/m)
        dpg.set_value(self.edit_mass_unit_id, "kg")
        dpg.set_value(self.edit_mass_value_id, f"{b.mass:.6e}")
        self._update_edit_mass_si_label()
        dpg.set_value(self.edit_radius_unit_id, "m")
        dpg.set_value(self.edit_radius_value_id, f"{b.radius:.6e}")
        self._update_edit_radius_si_label()
        # Position
        dpg.set_value(self.edit_pos_x_id, f"{b.position[0]:.6e}")
        dpg.set_value(self.edit_pos_y_id, f"{b.position[1]:.6e}")
        # Color
        dpg.set_value(self.edit_color_id, (b.color[0], b.color[1], b.color[2], 255))

    def _apply_selected_body_edits(self):
        b = self.sim.get_selected_body()
        if not b:
            self._set_error("No body selected to edit.")
            return
        name = dpg.get_value(self.edit_name_id).strip() or b.name
        # Mass
        mass_val = try_float(dpg.get_value(self.edit_mass_value_id))
        mass_unit = dpg.get_value(self.edit_mass_unit_id)
        radius_val = try_float(dpg.get_value(self.edit_radius_value_id))
        radius_unit = dpg.get_value(self.edit_radius_unit_id)
        pos_x = try_float(dpg.get_value(self.edit_pos_x_id))
        pos_y = try_float(dpg.get_value(self.edit_pos_y_id))
        color_rgba = dpg.get_value(self.edit_color_id)
        if None in (mass_val, radius_val, pos_x, pos_y):
            self._set_error("Invalid inputs in edit form.")
            return
        mass_si = self.mass_to_si(mass_val, mass_unit)
        radius_si = self.radius_to_si(radius_val, radius_unit)
        if mass_si <= 0 or radius_si <= 0:
            self._set_error("Mass and Radius must be positive.")
            return
        # Apply
        with self.sim.lock:
            b.name = name
            b.mass = mass_si
            b.radius = radius_si
            b.position = (pos_x, pos_y)
            b.color = (int(color_rgba[0]), int(color_rgba[1]), int(color_rgba[2]))
        self._refresh_body_list()
        self._set_status("Applied edits to selected body.")

    def _toggle_play(self):
        with self.sim.lock:
            self.sim.playing = not self.sim.playing
            state = "Playing" if self.sim.playing else "Paused"
        self._set_status(f"Simulation {state}.")

    def _step_once(self):
        # Perform a single physics step when paused
        with self.sim.lock:
            was_playing = self.sim.playing
            self.sim.playing = False
        self.sim.step_physics(1 / 60.0)  # advance by 1/60 real second scaled by time_scale
        with self.sim.lock:
            self.sim.playing = was_playing
        self._set_status("Stepped one frame.")

    def _toggle_trails(self, sender, value, user_data=None):
        with self.sim.lock:
            self.sim.show_trails = bool(value)
        if not value:
            self.sim.clear_trails()
        self._set_status(f"Trails {'ON' if value else 'OFF'}.")

    def _on_trail_length(self, sender, app_data, user_data=None):
        # app_data should be the int value from the input; fall back to reading the widget value if needed
        try:
            length = int(app_data if app_data is not None else 250)
        except Exception:
            try:
                length = int(dpg.get_value("trail_length_input"))
            except Exception:
                length = 250
        if length < 10:
            length = 10
        self.sim.set_trail_length(length)
        # Reflect clamped value back into the UI so +/- and typing stay consistent
        try:
            dpg.set_value("trail_length_input", length)
        except Exception:
            pass
        self._set_status(f"Trail length set to {length}.")

    def _set_integrator(self, value):
        with self.sim.lock:
            self.sim.integrator = value
        self._set_status(f"Integrator: {value}")

    def _toggle_velocity_vectors(self, value):
        with self.sim.lock:
            self.sim.show_velocity_vectors = bool(value)

    def _toggle_collisions(self, value):
        with self.sim.lock:
            self.sim.enable_collisions = bool(value)

    def _set_collision_mode(self, mode):
        with self.sim.lock:
            if mode in ("Merge", "Elastic", "Accretion"):
                self.sim.collision_mode = mode
        self._set_status(f"Collision mode: {mode}")

    def _set_restitution(self, val):
        with self.sim.lock:
            try:
                self.sim.restitution = float(val)
            except Exception:
                pass

    def _set_collision_radius_scale(self, val):
        with self.sim.lock:
            try:
                self.sim.collision_radius_scale = max(0.01, float(val))
            except Exception:
                pass

    def load_template(self, name: str):
        name = name.strip()
        bodies = None
        # Try JSON templates first
        if hasattr(self, "_template_map") and name in self._template_map:
            fn = self._template_map[name]
            try:
                bodies_loaded, time_scale, display_name = load_json_template(fn)
                if bodies_loaded:
                    bodies = bodies_loaded
                    # Apply optional time scale from template
                    if time_scale is not None:
                        with self.sim.lock:
                            self.sim.time_scale = float(time_scale)
                        try:
                            dpg.set_value("speed_slider", float(time_scale))
                        except Exception:
                            pass
            except Exception:
                bodies = None
        # Fallback to built-in Python presets for compatibility
        if bodies is None:
            if name == "Solar System (scaled)":
                bodies = template_solar_system_scaled()
                with self.sim.lock:
                    self.sim.time_scale = 3_500_000.0
                try:
                    dpg.set_value("speed_slider", 3_500_000.0)
                except Exception:
                    pass
            elif name == "Classic 3-body (Figure-eight)":
                bodies = template_three_body_figure_eight()
            elif name == "Classic 3-body (Lagrange)":
                bodies = template_three_body_lagrange()
            elif name == "Empty custom":
                bodies = template_empty()
            else:
                bodies = template_empty()

        self.sim.replace_bodies(bodies)
        self._refresh_body_list()
        self._set_status(f"Loaded template: {name}")
        self.renderer.auto_frame_camera()

    def _sync_ui_with_sim(self):
        """
        Periodic UI update to reflect selected body's position/velocity and update SI readouts as needed.
        """
        # Update SI readouts for add-body form
        self._update_mass_si_label()
        self._update_radius_si_label()

        # Sync selected body info
        b = self.sim.get_selected_body()
        if b:
            # Update body list selection text
            with self.sim.lock:
                names = [bb.name for bb in self.sim.bodies]
                try:
                    dpg.set_value(self.body_list_id, names[self.sim.selected_index if self.sim.selected_index is not None else 0])
                except Exception:
                    pass
            # Positions (read-only summary)
            dpg.set_value(self.sel_pos_x_id, f"{b.position[0]:.6e}")
            dpg.set_value(self.sel_pos_y_id, f"{b.position[1]:.6e}")
            # Only repopulate edit fields when selection changes to avoid clobbering user edits
            with self.sim.lock:
                current_idx = self.sim.selected_index
            if current_idx != self._last_edit_selected_idx:
                self._populate_edit_fields_from_selected()
                self._last_edit_selected_idx = current_idx
            # Only update velocity fields if user isn't currently editing;
            # DearPyGui doesn't expose editing state easily, so we set gently.
            try:
                vx_field = dpg.get_value(self.sel_vel_x_id)
                vy_field = dpg.get_value(self.sel_vel_y_id)
                # If fields are empty or equal to previous set, refresh with current value
                if vx_field.strip() == "" or try_float(vx_field) is None:
                    dpg.set_value(self.sel_vel_x_id, f"{b.velocity[0]:.6e}")
                if vy_field.strip() == "" or try_float(vy_field) is None:
                    dpg.set_value(self.sel_vel_y_id, f"{b.velocity[1]:.6e}")
            except Exception:
                pass
        # Show recent collision info if available
        try:
            with self.sim.lock:
                msg = self.sim.last_collision_msg
                self.sim.last_collision_msg = None
            if msg:
                self._set_status(msg)
        except Exception:
            pass
        # Reschedule next sync
        self._schedule_sync()

# ============================================================
# Default Scene and Application Entry
# ============================================================

def build_default_scene(sim: SimulationController):
    # Start with an empty scene by default
    bodies = template_empty()
    sim.replace_bodies(bodies)

def main():
    sim = SimulationController()

    # Ensure a working default scene is present before any UI callbacks
    build_default_scene(sim)

    renderer = PygameRenderer(sim)

    # Start Pygame renderer thread
    renderer.start()

    ui = UI(sim, renderer)

    # Keyboard shortcut in UI window to toggle play/pause (Space)
    with dpg.handler_registry():
        def key_down(sender, app_data):
            key = app_data
            # Space (32) in DPG key codes: dpg.mvKey_Spacebar
            if key == dpg.mvKey_Spacebar:
                ui._toggle_play()
        dpg.add_key_down_handler(callback=key_down)

    # Run Dear PyGui event loop
    try:
        dpg.start_dearpygui()
    finally:
        # Stop simulation and renderer
        sim.running = False
        renderer.running = False
        renderer.join(timeout=2.0)
        dpg.destroy_context()

if __name__ == "__main__":
    main()

# ============================================================
# Suggested Future Improvements (not required)
# ============================================================
# - Add energy and angular momentum diagnostics; show total energy drift to assess integrator accuracy.
# - Implement additional integrators (Velocity Verlet, Symplectic Leapfrog) and allow hot-swapping per body group.
# - Add collision detection and merging/splitting of bodies with conservation laws.
# - Implement saving/loading scenes to JSON, and presets editor inside the UI.
# - GPU acceleration for N-body forces (e.g., via CuPy) and Barnes-Hut tree (O(N log N)) for larger N.
# - Allow "launch vector" editing: drag from a selected body to set its velocity interactively before resuming.
# - Add unit overlays (AU, km) and dynamic scale bar in the viewport HUD.

