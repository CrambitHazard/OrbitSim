#!/usr/bin/env python3
"""
Vector helper functions for 2D operations.

These are small, fast functions for vector math used throughout the app.
"""
import math
from typing import Tuple


def clamp(x: float, a: float, b: float) -> float:
    """Clamp x to the inclusive range [a, b]."""
    return max(a, min(b, x))


def vec_add(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (a[0] + b[0], a[1] + b[1])


def vec_sub(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])


def vec_scale(a: Tuple[float, float], s: float) -> Tuple[float, float]:
    return (a[0] * s, a[1] * s)


def vec_len(a: Tuple[float, float]) -> float:
    return math.hypot(a[0], a[1])


def vec_norm(a: Tuple[float, float]) -> Tuple[float, float]:
    l = vec_len(a)
    if l == 0:
        return (0.0, 0.0)
    return (a[0] / l, a[1] / l)


def vec_dot(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]

