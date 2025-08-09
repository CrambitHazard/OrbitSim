#!/usr/bin/env python3
"""
Data models for Orbit Simulator.

This module defines the core Body dataclass shared between physics, rendering, and UI.

Units and usage
- position is in meters [m], velocity in meters per second [m/s], radius in meters [m], mass in kg.
- trail stores past positions to render motion paths; it is mutated by the simulation thread.
- Access to Body instances is coordinated by SimulationController using a lock.
"""
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple


@dataclass
class Body:
    """
    Represents a celestial body in the simulation.
    
    Fields:
    - name: Identifier for the body
    - mass: Mass in kilograms
    - radius: Visual/physical radius in meters
    - position: 2D position (x, y) in meters
    - velocity: 2D velocity (vx, vy) in meters/second
    - color: RGB tuple used for rendering
    - trail: Deque of past positions for drawing motion trails
    """
    name: str
    mass: float
    radius: float
    position: Tuple[float, float]
    velocity: Tuple[float, float]
    color: Tuple[int, int, int] = (200, 200, 255)
    trail: Deque[Tuple[float, float]] = field(default_factory=lambda: deque(maxlen=200))

    def add_trail_point(self) -> None:
        """Append the current position to the trail."""
        self.trail.append(self.position)

