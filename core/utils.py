#!/usr/bin/env python3
"""
General utilities for Orbit Simulator.
"""
from typing import Optional


def try_float(val) -> Optional[float]:
    try:
        return float(val)
    except Exception:
        return None

