#!/usr/bin/env python3
"""
Preset and body JSON loading utilities.

This module defines simple JSON schemas and loaders for:
- Scene templates: a list of bodies to spawn (templates/*.json)
- Celestial body definitions: reusable body definitions (celestial_bodies/*.json)

Schemas
=======
Template JSON (templates/*.json):
{
  "name": "Human-friendly preset name",
  "description": "Optional description",
  "time_scale": 3500000.0,           # optional, default None
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

Celestial body JSON (celestial_bodies/*.json):
{
  "name": "Earth",
  "mass": 5.972e24,
  "radius": 6.371e6,
  "color": [100, 149, 237]
}

Users can add their own JSON files into these folders and they'll be picked up by the loader.
"""
import json
import os
from typing import Dict, List, Optional, Tuple
from .data_models import Body

TEMPLATES_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "templates")
BODIES_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "celestial_bodies")


def _read_json(path: str) -> Optional[dict]:
  try:
    with open(path, "r", encoding="utf-8") as f:
      return json.load(f)
  except Exception:
    return None


def _coerce_color(c: List[int]) -> Tuple[int, int, int]:
  try:
    r, g, b = int(c[0]), int(c[1]), int(c[2])
    r = max(0, min(255, r)); g = max(0, min(255, g)); b = max(0, min(255, b))
    return (r, g, b)
  except Exception:
    return (200, 200, 255)


def list_templates() -> List[Tuple[str, str]]:
  """Return list of (file_name, display_name) for available templates."""
  items: List[Tuple[str, str]] = []
  if not os.path.isdir(TEMPLATES_DIR):
    return items
  for fn in sorted(os.listdir(TEMPLATES_DIR)):
    if not fn.lower().endswith(".json"):
      continue
    path = os.path.join(TEMPLATES_DIR, fn)
    data = _read_json(path) or {}
    display = data.get("name") or os.path.splitext(fn)[0]
    items.append((fn, display))
  return items


def load_template(file_name: str) -> Tuple[List[Body], Optional[float], Optional[str]]:
  """
  Load a template JSON by file name.
  Returns (bodies, time_scale, display_name)
  """
  path = os.path.join(TEMPLATES_DIR, file_name)
  data = _read_json(path) or {}
  display_name = data.get("name") or os.path.splitext(file_name)[0]
  time_scale = data.get("time_scale")
  bodies: List[Body] = []
  for b in data.get("bodies", []):
    try:
      bodies.append(Body(
        name=b.get("name", "Body"),
        mass=float(b["mass"]),
        radius=float(b["radius"]),
        position=(float(b["position"][0]), float(b["position"][1])),
        velocity=(float(b["velocity"][0]), float(b["velocity"][1])),
        color=_coerce_color(b.get("color", [200,200,255]))
      ))
    except Exception:
      continue
  return bodies, time_scale, display_name


def list_celestial_bodies() -> List[str]:
  """List JSON files available in the celestial_bodies directory."""
  if not os.path.isdir(BODIES_DIR):
    return []
  return sorted([fn for fn in os.listdir(BODIES_DIR) if fn.lower().endswith(".json")])


def load_celestial_body(file_name: str, position=(0.0,0.0), velocity=(0.0,0.0)) -> Optional[Body]:
  """Load a single body definition and instantiate it with given pose."""
  path = os.path.join(BODIES_DIR, file_name)
  data = _read_json(path)
  if not data:
    return None
  try:
    return Body(
      name=data.get("name", os.path.splitext(file_name)[0]),
      mass=float(data["mass"]),
      radius=float(data["radius"]),
      position=(float(position[0]), float(position[1])),
      velocity=(float(velocity[0]), float(velocity[1])),
      color=_coerce_color(data.get("color", [200,200,255]))
    )
  except Exception:
    return None

