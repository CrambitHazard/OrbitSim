#!/usr/bin/env python3
"""
Core Physics Engine for Orbit Simulator

This module handles the gravitational N-body physics calculations using 
Newton's law of universal gravitation and numerical integration methods.

Features:
- Softened gravity to prevent singularities when bodies get very close
- RK4 (Runge-Kutta 4th order) integration for accurate orbital mechanics
- Thread-safe physics stepping with configurable timesteps
- Support for variable time scaling (simulation speed control)

Author: Orbit Simulator Project
"""

import math
from typing import List, Tuple
from .data_models import Body
from .vector_utils import vec_add, vec_scale


# Physical constants (SI units)
G = 6.67430e-11  # Gravitational constant in m^3 kg^-1 s^-2
DEFAULT_SOFTENING = 1e7  # Default softening parameter in meters


class NBodyPhysics:
    """
    N-body gravitational physics engine with softened gravity.
    
    This class implements Newton's law of universal gravitation with a
    softening parameter to prevent numerical instabilities when bodies
    approach each other very closely.
    
    The gravitational force between two bodies is:
    F = G * m1 * m2 * r_hat / (r^2 + eps^2)^(3/2)
    
    Where eps is the softening parameter.
    """
    
    def __init__(self, softening: float = DEFAULT_SOFTENING):
        """
        Initialize the physics engine.
        
        Args:
            softening: Softening parameter in meters to prevent singularities
        """
        self.softening = max(0.0, float(softening))
    
    def set_softening(self, softening: float) -> None:
        """
        Update the softening parameter.
        
        Args:
            softening: New softening parameter in meters (must be >= 0)
        """
        self.softening = max(0.0, float(softening))
    
    def compute_accelerations(self, bodies: List[Body], 
                            positions: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Compute gravitational accelerations for all bodies.
        
        For each body, calculates the total gravitational acceleration due to
        all other bodies using the softened gravity formula:
        a_i = sum_j(G * m_j * r_ij / (|r_ij|^2 + eps^2)^(3/2))
        
        Args:
            bodies: List of Body objects containing masses
            positions: List of (x, y) position tuples corresponding to each body
            
        Returns:
            List of (ax, ay) acceleration tuples for each body
        """
        n = len(bodies)
        accelerations = [(0.0, 0.0) for _ in range(n)]
        
        # Precompute softening squared for efficiency
        eps_squared = self.softening * self.softening
        
        # Calculate pairwise gravitational forces
        for i in range(n):
            ax_total, ay_total = 0.0, 0.0
            xi, yi = positions[i]
            
            # Sum contributions from all other bodies
            for j in range(n):
                if i == j:
                    continue  # Skip self-interaction
                    
                xj, yj = positions[j]
                
                # Vector from body i to body j
                dx = xj - xi
                dy = yj - yi
                
                # Distance squared with softening
                r_squared_soft = dx * dx + dy * dy + eps_squared
                
                # Inverse distance cubed (for acceleration calculation)
                inv_r_cubed = 1.0 / (r_squared_soft * math.sqrt(r_squared_soft))
                
                # Gravitational acceleration magnitude
                acceleration_magnitude = G * bodies[j].mass * inv_r_cubed
                
                # Add acceleration components
                ax_total += dx * acceleration_magnitude
                ay_total += dy * acceleration_magnitude
            
            accelerations[i] = (ax_total, ay_total)
        
        return accelerations
    
    def rk4_integration_step(self, bodies: List[Body], timestep: float) -> None:
        """
        Perform one Runge-Kutta 4th order integration step.
        
        RK4 is a high-accuracy numerical integration method that evaluates
        derivatives at four points to estimate the change over a timestep.
        This gives much better accuracy than simple Euler integration.
        
        The method works by:
        1. k1: Derivative at start of timestep
        2. k2: Derivative at middle using k1
        3. k3: Derivative at middle using k2  
        4. k4: Derivative at end using k3
        5. Combine: (k1 + 2*k2 + 2*k3 + k4) / 6
        
        Args:
            bodies: List of Body objects to integrate (modified in place)
            timestep: Time step size in seconds
        """
        n = len(bodies)
        
        # Store initial states
        initial_positions = [body.position for body in bodies]
        initial_velocities = [body.velocity for body in bodies]
        
        # k1: Evaluate derivatives at start of timestep
        k1_accelerations = self.compute_accelerations(bodies, initial_positions)
        k1_velocities = initial_velocities
        
        # k2: Evaluate derivatives at timestep/2 using k1
        k2_positions = [
            vec_add(initial_positions[i], vec_scale(k1_velocities[i], timestep * 0.5))
            for i in range(n)
        ]
        k2_velocities = [
            vec_add(initial_velocities[i], vec_scale(k1_accelerations[i], timestep * 0.5))
            for i in range(n)
        ]
        k2_accelerations = self.compute_accelerations(bodies, k2_positions)
        
        # k3: Evaluate derivatives at timestep/2 using k2
        k3_positions = [
            vec_add(initial_positions[i], vec_scale(k2_velocities[i], timestep * 0.5))
            for i in range(n)
        ]
        k3_velocities = [
            vec_add(initial_velocities[i], vec_scale(k2_accelerations[i], timestep * 0.5))
            for i in range(n)
        ]
        k3_accelerations = self.compute_accelerations(bodies, k3_positions)
        
        # k4: Evaluate derivatives at end of timestep using k3
        k4_positions = [
            vec_add(initial_positions[i], vec_scale(k3_velocities[i], timestep))
            for i in range(n)
        ]
        k4_velocities = [
            vec_add(initial_velocities[i], vec_scale(k3_accelerations[i], timestep))
            for i in range(n)
        ]
        k4_accelerations = self.compute_accelerations(bodies, k4_positions)
        
        # Combine all k values using RK4 formula
        for i in range(n):
            # Calculate position change: (k1 + 2*k2 + 2*k3 + k4) / 6 * dt
            position_change = vec_scale(
                vec_add(
                    vec_add(k1_velocities[i], vec_scale(vec_add(k2_velocities[i], k3_velocities[i]), 2.0)),
                    k4_velocities[i]
                ),
                timestep / 6.0
            )
            
            # Calculate velocity change: (k1 + 2*k2 + 2*k3 + k4) / 6 * dt
            velocity_change = vec_scale(
                vec_add(
                    vec_add(k1_accelerations[i], vec_scale(vec_add(k2_accelerations[i], k3_accelerations[i]), 2.0)),
                    k4_accelerations[i]
                ),
                timestep / 6.0
            )
            
            # Update body state
            bodies[i].position = vec_add(bodies[i].position, position_change)
            bodies[i].velocity = vec_add(bodies[i].velocity, velocity_change)


def circular_orbit_velocity(central_mass: float, orbital_radius: float) -> float:
    """
    Calculate the velocity needed for a circular orbit.
    
    For a circular orbit, the gravitational force provides exactly the
    centripetal force needed. This gives us:
    G * M / r = v^2 / r
    Therefore: v = sqrt(G * M / r)
    
    Args:
        central_mass: Mass of the central body in kg
        orbital_radius: Orbital radius in meters
        
    Returns:
        Orbital velocity in m/s for a circular orbit
    """
    if orbital_radius <= 0:
        return 0.0
    
    return math.sqrt(G * central_mass / orbital_radius)


def escape_velocity(total_mass: float, separation: float) -> float:
    """
    Calculate the escape velocity at a given separation.
    
    The escape velocity is the minimum speed needed to escape the
    gravitational influence of a body. At the surface or separation
    distance, this is: v_escape = sqrt(2 * G * M / r)
    
    Args:
        total_mass: Total mass of the system in kg
        separation: Distance from center of mass in meters
        
    Returns:
        Escape velocity in m/s
    """
    if separation <= 0 or total_mass <= 0:
        return 0.0
        
    return math.sqrt(2.0 * G * total_mass / separation)
