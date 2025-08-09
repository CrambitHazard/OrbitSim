#!/usr/bin/env python3
"""
Collision handling for Orbit Simulator.

Collision modes
- Merge: Perfectly inelastic merge conserving linear momentum; radii combine by
  conserving volume (r^3 additive). Trails are cleared on merge.
- Elastic: Resolves interpenetration and applies a 1D impulse along the contact
  normal with coefficient of restitution e ∈ [0,1].
- Accretion: Heuristic astrophysical model. If relative speed is below the local
  mutual escape velocity based on the sum of radii, bodies merge; otherwise bounce
  elastically with restitution e.

Settings knobs
- enable: master switch.
- mode: one of {"Merge","Elastic","Accretion"}.
- restitution: elasticity for Elastic/Accretion bounces.
- radius_scale: multiplies visual radii for collision tests (useful when radii are
  tiny relative to camera scale or timestep).

Implementation notes
- Pairwise O(N^2) detection with simple sphere (circle) overlap in 2D.
- Overlap resolution uses mass-weighted positional correction along the normal.
- The function mutates the bodies list (removals) when merges occur; call sites should
  not rely on indices staying stable across a call.
"""
from typing import List, Optional, Set
import math
from .data_models import Body
from .vector_utils import clamp
from .physics import escape_velocity


class CollisionSettings:
    """Container for collision-related settings."""
    def __init__(self, enable: bool = True, mode: str = "Accretion", restitution: float = 0.2, radius_scale: float = 1.0):
        self.enable = enable
        self.mode = mode  # "Merge" | "Elastic" | "Accretion"
        self.restitution = float(restitution)
        self.radius_scale = max(0.01, float(radius_scale))


def handle_collisions(bodies: List[Body], settings: CollisionSettings) -> Optional[str]:
    """
    Detect and resolve collisions between bodies in-place.

    For each overlapping pair, resolves interpenetration and applies either a merge
    or an elastic impulse depending on settings. In Accretion mode a merge is chosen
    when the relative speed is less than the escape velocity computed from the total
    mass and the (scaled) sum of radii.

    Args:
        bodies: Mutable list of Body instances (will be modified; items can be removed).
        settings: CollisionSettings controlling mode and parameters.

    Returns:
        Optional human-readable message describing the last collision processed, or None.
    """
    if not settings.enable or len(bodies) < 2:
        return None

    last_msg: Optional[str] = None
    to_remove: Set[int] = set()
    merged_any = False

    n = len(bodies)
    for i in range(n):
        if i in to_remove:
            continue
        bi = bodies[i]
        for j in range(i + 1, n):
            if j in to_remove:
                continue
            bj = bodies[j]

            # Separation
            dx = bj.position[0] - bi.position[0]
            dy = bj.position[1] - bi.position[1]
            dist = math.hypot(dx, dy)
            r_sum = (bi.radius + bj.radius) * settings.radius_scale

            if dist <= r_sum and dist > 0:
                # Collision normal
                nx, ny = dx / dist, dy / dist
                # Resolve overlap inversely proportional to mass
                overlap = r_sum - dist
                inv_mi = 0.0 if bi.mass == 0 else 1.0 / bi.mass
                inv_mj = 0.0 if bj.mass == 0 else 1.0 / bj.mass
                inv_sum = inv_mi + inv_mj
                if inv_sum > 0:
                    corr_i = -overlap * (inv_mi / inv_sum)
                    corr_j = +overlap * (inv_mj / inv_sum)
                    bi.position = (bi.position[0] + nx * corr_i, bi.position[1] + ny * corr_i)
                    bj.position = (bj.position[0] + nx * corr_j, bj.position[1] + ny * corr_j)

                # Relative normal velocity
                vrx = bj.velocity[0] - bi.velocity[0]
                vry = bj.velocity[1] - bi.velocity[1]
                vn = vrx * nx + vry * ny

                if settings.mode in ("Elastic", "Accretion") and vn > 0:
                    # Already separating
                    continue

                if settings.mode == "Merge":
                    _merge_pair(bodies, i, j, to_remove)
                    merged_any = True
                    last_msg = f"Merged {bi.name} + {bj.name}"
                elif settings.mode == "Elastic":
                    e = clamp(settings.restitution, 0.0, 1.0)
                    _apply_elastic_impulse(bi, bj, nx, ny, vn, e)
                    last_msg = f"Elastic collision: {bi.name} ↔ {bj.name}"
                elif settings.mode == "Accretion":
                    v_rel = math.hypot(vrx, vry)
                    v_esc = escape_velocity(bi.mass + bj.mass, max(r_sum, 1e-6))
                    if v_rel < v_esc:
                        _merge_pair(bodies, i, j, to_remove)
                        merged_any = True
                        last_msg = f"Merged (accretion) {bi.name} + {bj.name}"
                    else:
                        e = clamp(settings.restitution, 0.0, 1.0)
                        _apply_elastic_impulse(bi, bj, nx, ny, vn, e)
                        last_msg = f"Bounce (accretion off): {bi.name} ↔ {bj.name}"

    if merged_any:
        for idx in sorted(to_remove, reverse=True):
            del bodies[idx]

    return last_msg


def _merge_pair(bodies: List[Body], i: int, j: int, to_remove: Set[int]) -> None:
    if i < 0 or j < 0 or i == j:
        return
    if j in to_remove:
        return

    bi = bodies[i]
    bj = bodies[j]

    # Ensure bi is heavier for stable replacement
    if bj.mass > bi.mass:
        bi, bj = bj, bi
        i, j = j, i

    m_total = bi.mass + bj.mass
    if m_total <= 0:
        return

    new_pos = ((bi.position[0] * bi.mass + bj.position[0] * bj.mass) / m_total,
               (bi.position[1] * bi.mass + bj.position[1] * bj.mass) / m_total)
    new_vel = ((bi.velocity[0] * bi.mass + bj.velocity[0] * bj.mass) / m_total,
               (bi.velocity[1] * bi.mass + bj.velocity[1] * bj.mass) / m_total)
    # Combine radii by volume (r^3 adds)
    new_radius = (bi.radius ** 3 + bj.radius ** 3) ** (1.0 / 3.0)
    new_color = bi.color if bi.mass >= bj.mass else bj.color

    bi.mass = m_total
    bi.position = new_pos
    bi.velocity = new_vel
    bi.radius = new_radius
    bi.color = new_color
    bi.name = f"{bi.name}+{bj.name}"
    bi.trail.clear()

    to_remove.add(j)


def _apply_elastic_impulse(bi: Body, bj: Body, nx: float, ny: float, vn: float, e: float) -> None:
    """Apply a 1D elastic impulse along the collision normal.

    Args:
        bi, bj: colliding bodies (mutated in-place).
        nx, ny: unit collision normal from i to j.
        vn: relative normal velocity (positive means separating).
        e: coefficient of restitution in [0,1].
    """
    inv_mi = 0.0 if bi.mass == 0 else 1.0 / bi.mass
    inv_mj = 0.0 if bj.mass == 0 else 1.0 / bj.mass
    denom = inv_mi + inv_mj
    if denom <= 0:
        return

    j_imp = -(1.0 + e) * vn / denom
    ix = j_imp * nx
    iy = j_imp * ny

    bi.velocity = (bi.velocity[0] - ix * inv_mi, bi.velocity[1] - iy * inv_mi)
    bj.velocity = (bj.velocity[0] + ix * inv_mj, bj.velocity[1] + iy * inv_mj)
