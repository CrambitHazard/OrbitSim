#!/usr/bin/env python3
"""
Camera utilities for 2D world-to-screen transforms.
"""
from typing import Optional, Tuple
from .constants import (
    DEFAULT_METERS_PER_PIXEL,
    MIN_METERS_PER_PIXEL,
    MAX_METERS_PER_PIXEL,
    VIEW_WIDTH,
    VIEW_HEIGHT,
)
from .vector_utils import clamp


class Camera2D:
    """
    Simple 2D camera that maps world coordinates (meters) to screen pixels.
    """

    def __init__(self, center=(0.0, 0.0), meters_per_pixel=DEFAULT_METERS_PER_PIXEL):
        self.center = [center[0], center[1]]
        self.mpp = meters_per_pixel
        self.viewport_size = (VIEW_WIDTH, VIEW_HEIGHT)

    def set_viewport_size(self, w: int, h: int) -> None:
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
        factor = clamp(factor, 0.05, 20.0)
        before = None
        if pivot_screen is not None:
            before = self.screen_to_world(pivot_screen)
        self.mpp = clamp(self.mpp * (1.0 / factor), MIN_METERS_PER_PIXEL, MAX_METERS_PER_PIXEL)
        if pivot_screen is not None and before is not None:
            after = self.screen_to_world(pivot_screen)
            self.center[0] += (before[0] - after[0])
            self.center[1] += (before[1] - after[1])

    def pan_pixels(self, dx_pixels, dy_pixels):
        self.center[0] -= dx_pixels * self.mpp
        self.center[1] -= dy_pixels * self.mpp

