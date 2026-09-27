#!/usr/bin/env python3
"""Seedable, strategy-independent terrain randomness (paper-hardening PR).

Common Random Numbers (CRN): every random quantity is a deterministic hash of
(seed, cell, salt), so under one seed two strategies face the *identical* random
environment and only their mode/path decisions differ. There is no global RNG
state, so the order in which cells are visited does not change any draw.

Truth / perception separation (avoids the self-fulfilling-prophecy critique):
- ``theta_true(cell)``  : ground-truth traversability. Used ONLY by the physics
  (ground controller stuck/slip) and the logger; never by a strategy.
- strategies observe a noisy estimate ``perceived_trav = theta_true + noise``
  plus an uncertainty proxy. They never see ``theta_true``.
- the stuck/slip Bernoulli is drawn against ``theta_true`` (not the perceived
  value), so a strategy cannot make the ground safer just by mis-perceiving it.
"""
import hashlib
import math


def clamp(value, lo=0.0, hi=1.0):
    return max(lo, min(hi, value))


def _u01(seed, ix, iy, salt):
    """Deterministic uniform in [0, 1) keyed by (seed, cell, salt)."""
    key = "%d|%d|%d|%s" % (int(seed), int(ix), int(iy), salt)
    digest = hashlib.sha256(key.encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "big") / float(1 << 64)


def cell_index(x, y, cell_size):
    cell_size = cell_size if cell_size > 1e-6 else 1.0
    return int(math.floor(x / cell_size)), int(math.floor(y / cell_size))


def signed_noise(seed, ix, iy, salt, scale):
    """Zero-mean noise in [-scale, scale]."""
    return (2.0 * _u01(seed, ix, iy, salt) - 1.0) * scale


def theta_true(seed, x, y, structural_trav, cell_size, noise_scale):
    """Ground-truth traversability at the robot's cell."""
    ix, iy = cell_index(x, y, cell_size)
    return clamp(structural_trav + signed_noise(seed, ix, iy, "theta", noise_scale))


def perceived_trav(seed, x, y, theta, cell_size, percept_noise_scale):
    """Noisy traversability estimate the strategies observe (p_trav)."""
    ix, iy = cell_index(x, y, cell_size)
    return clamp(theta + signed_noise(seed, ix, iy, "percept", percept_noise_scale))


def stuck_draw(seed, x, y, theta, cell_size):
    """Bernoulli stuck outcome governed by theta_true. True => cell is a trap."""
    ix, iy = cell_index(x, y, cell_size)
    p_fail = clamp(1.0 - theta)
    return _u01(seed, ix, iy, "stuck") < p_fail
