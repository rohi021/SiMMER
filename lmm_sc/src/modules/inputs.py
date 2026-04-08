#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# SiMMER Hexapod — System Parameters
#
# This file defines ALL physical, kinematic, and control
# parameters for the SiMMER hexapod robot.
#
# ARCHITECTURE NOTE:
#   Currently hardcoded in Python. Future migration path:
#   1. Move to YAML config file (like joints.yaml)
#   2. Load via rosparam server in master.py
#   3. Pass to modules via function arguments (already done)
#   This keeps the parameter interface stable while enabling
#   runtime tuning without recompilation.
#
# UNITS:
#   Length → meters (m)
#   Angle  → radians (rad)
#   Time   → seconds (s)
#   Speed  → meters per second (m/s)
#
# COORDINATE FRAME:
#   Origin at trunk body geometric center
#   X → forward (toward leg 1/2)
#   Y → left (toward legs 1,3,5)
#   Z → up
#
# IMPORTED BY:
#   master.py (all parameters)
#   initialize_lmm.py (via master.py arguments)
#
# ⚠️  ALL NUMPY ARRAYS ARE READ-ONLY.
#     If you need to modify a value, use .copy() first.
#     This prevents accidental corruption of global state.
# ══════════════════════════════════════════════════════════════

import numpy

# Export list — prevents accidental import of numpy via "from inputs import *"
__all__ = [
    'di', 'd_man', 'li', 'l_man', 'phi', 'phi_m',
    'r_l0_si_p0', 'r_l0_mb_p0',
    'vel_tb_max', 'gamma_r', 'gamma_l', 'eta_G_L0',
    'f_in', 'T_in', 'T_stroke',
    'Hmi1', 'del_h', 'hi3_dash', 'lt_time_ratio',
    'm_t_man', 'w', 'm_t', 'bounds',
    'r_G_tb_init', 'r_G_ee_init', 'x_si_li_init',
]


# ══════════════════════════════════════════════════════════════
# ROBOT BODY DIMENSIONS
# ══════════════════════════════════════════════════════════════

# ── Leg Link Lengths ─────────────────────────────────────────
# Each leg has 3 links: [L0 (hip), L1 (thigh), L2 (shin)]
#
#   Hip ──L0──┐
#             L1 (thigh)
#             │
#             L2 (shin)
#             │
#           [tip]

li = [0.095, 0.105, 0.100]    # [L0, L1, L2] in meters

# ── Leg Link Offsets (DH-parameter d) ────────────────────────
# Physical lateral offset at each joint.
#
# ⚠️  NUMERICAL REGULARIZATION:
#   di[0] = 1e-10 (NOT zero) to prevent division by zero in IK.
#   The hip offset is physically zero, but the IK formula
#   computes arctan(... / (di_sum + Ki2)). When Ki2 ≈ 0
#   (target directly above hip), di_sum = 0 causes NaN.
#   Using 1e-10 regularizes without affecting results
#   (error < 0.0000001 degrees).
#   DO NOT CHANGE TO [0, 0, 0] — IT WILL BREAK IK.

di = [1e-10, 0.0, 0.0]        # [d0, d1, d2] in meters


# ── Manipulator Link Lengths ─────────────────────────────────
# 3-DOF arm: [L0 (base vertical), L1 (upper arm), L2 (forearm)]
#
#   Base ──L0──┐ (vertical)
#              L1 (upper arm)
#              │
#              L2 (forearm)
#              │
#            [EE tip]

l_man = [0.0395, 0.250, 0.290]  # [L0, L1, L2] in meters

# ── Manipulator Link Offsets ─────────────────────────────────
# Same regularization hack as leg offsets.

d_man = [1e-10, 0.0, 0.0]       # [d0, d1, d2] in meters


# ── Mounting Angles ──────────────────────────────────────────
# phi   → leg hip joint mounting angle relative to body frame
#          0 means legs extend horizontally from hip
# phi_m → manipulator base mounting angle
#          π/2 means manipulator extends vertically from base

phi = 0.0                        # Leg mounting angle (rad)
phi_m = numpy.pi / 2.0           # Manipulator mounting angle (rad) — vertical


# ══════════════════════════════════════════════════════════════
# BODY FRAME GEOMETRY
# ══════════════════════════════════════════════════════════════

# ── Hip Joint Positions ──────────────────────────────────────
# Position of each leg's hip joint relative to trunk body center.
#
#   TOP VIEW (Z up, Y right on page):
#
#     Leg 1 (-X,+Y) ●─────────────● Leg 2 (+X,+Y)     ← Front
#                    │             │
#     Leg 3 (-X, 0) ●─────────────● Leg 4 (+X, 0)     ← Middle
#                    │             │
#     Leg 5 (-X,-Y) ●─────────────● Leg 6 (+X,-Y)     ← Rear
#
# All Z values = 0.0155m (hips slightly above body center)

r_l0_si_p0 = {
    1: numpy.array([-0.08,  0.205, 0.0155]),   # Left-Front
    2: numpy.array([ 0.08,  0.205, 0.0155]),   # Right-Front
    3: numpy.array([-0.08,  0.0,   0.0155]),   # Left-Middle
    4: numpy.array([ 0.08,  0.0,   0.0155]),   # Right-Middle
    5: numpy.array([-0.08, -0.205, 0.0155]),   # Left-Rear
    6: numpy.array([ 0.08, -0.205, 0.0155]),   # Right-Rear
}

# ── Manipulator Base Position ────────────────────────────────
# Position of manipulator base joint relative to trunk body center.
# Located at front-center, slightly above body frame.

r_l0_mb_p0 = numpy.array([0.0, 0.1025, 0.0425])  # [x, y, z] in meters


# ══════════════════════════════════════════════════════════════
# MOTION PARAMETERS
# ══════════════════════════════════════════════════════════════

# ── Trunk Body Velocity ──────────────────────────────────────
# Maximum velocity during gait locomotion (trapezoidal profile peak).

vel_tb_max = 0.04               # m/s (40 mm/s)


# ── Body Orientation Offsets ─────────────────────────────────
# Hip joint angular offsets for left/right leg mounting.
# Zero means symmetric mounting (no toe-in/toe-out).

gamma_r = 0.0                   # Right leg hip offset (rad)
gamma_l = 0.0                   # Left leg hip offset (rad)

# ── Body Frame Orientation ───────────────────────────────────
# Euler angles [roll, pitch, yaw] of body frame relative to global.
# [0, 0, 0] means body is level and aligned with global axes.
# inverse_kinematics.py computes rotation matrix from these values.

eta_G_L0 = [0.0, 0.0, 0.0]     # [roll, pitch, yaw] in radians


# ══════════════════════════════════════════════════════════════
# CONTROL TIMING
# ══════════════════════════════════════════════════════════════

f_in = 10.0                     # Control loop frequency (Hz)
T_in = 1.0 / f_in               # Control loop interval (s) = 0.1s
T_stroke = 1.0                  # Gait stroke duration (s)


# ══════════════════════════════════════════════════════════════
# GAIT PARAMETERS — LEG TIP TRAJECTORY
# ══════════════════════════════════════════════════════════════

# ── Swing Phase Heights ──────────────────────────────────────
#
#   SIDE VIEW (leg tip trajectory during swing):
#
#     ┌──────────────────────────┐ ← Hmi1 + del_h (max height)
#     │         ╭────────╮       │
#     │        ╱          ╲      │
#     │       ╱            ╲     │
#     ●──────╱              ╲────● ← ground level (hi3_dash)
#   lift-off                 touch-down
#     ├──────┤              ├────┤
#      lt_time_ratio    lt_time_ratio
#     (20% of stroke)  (20% of stroke)

Hmi1 = 0.015                    # Base swing height (m) — 15mm
del_h = 0.002                   # Additional height margin (m) — 2mm
                                 # Total max height = 17mm above ground
hi3_dash = 0.0                  # Leg tip Z at end of swing (m)
                                 # 0 = returns to same height as lift-off
lt_time_ratio = 0.2             # Fraction of stroke for lift/place (0.2 = 20%)
                                 # Middle 60% is at max height


# ══════════════════════════════════════════════════════════════
# MANIPULABILITY PARAMETERS
# ══════════════════════════════════════════════════════════════

# Threshold below which TB must move to restore manipulability.
# Manipulability is normalized: 0 = singular, 1 = optimal.

m_t_man = 0.25                  # Manipulability threshold for IK path check


# ══════════════════════════════════════════════════════════════
# REDUNDANCY RESOLUTION PARAMETERS
# ══════════════════════════════════════════════════════════════

# ── Objective Weights ────────────────────────────────────────
# w = [w_q1, w_q2, w_q3, w_xtb, w_ytb]
# Higher weight → optimizer penalizes deviation more.
# TB weights (20) >> joint weights (1) → prefers moving joints over body.

w = [1.0, 1.0, 1.0, 20.0, 20.0]

# ── Manipulability Threshold for Resolver ────────────────────
m_t = 0.25                      # Same as m_t_man (used inside optimizer constraint)

# ── Decision Variable Bounds ─────────────────────────────────
# [q1, q2, q3, x_tb, y_tb]
# Joint angles bounded to ±π/2 (physical servo limits)
# TB position unbounded (optimizer finds optimal displacement)

bounds = (
    (-1.5708, 1.5708),          # q1: manipulator base yaw (rad)
    (-1.5708, 1.5708),          # q2: manipulator shoulder pitch (rad)
    (-1.5708, 1.5708),          # q3: manipulator elbow pitch (rad)
    (-numpy.inf, numpy.inf),     # x_tb: trunk body X (m)
    (-numpy.inf, numpy.inf),     # y_tb: trunk body Y (m)
)


# ══════════════════════════════════════════════════════════════
# INITIAL CONDITIONS
# ══════════════════════════════════════════════════════════════

# ── Initial Trunk Body Position ──────────────────────────────
# Standing height = 0.125m (125mm) above ground.

r_G_tb_init = numpy.array([0.0, 0.0, 0.125])    # [x, y, z] in meters

# ── Initial End-Effector Position ────────────────────────────
# Arm extended forward and up from body.

r_G_ee_init = numpy.array([0.0, 0.45, 0.55])     # [x, y, z] in meters

# ── Initial Leg Tip Lateral Offset ───────────────────────────
# How far each leg tip starts from its hip joint (laterally).
# Used in initialize_lmm.py to compute initial standing footprint.
# Formula: tip_x = hip_x + ((-1)^leg) * x_si_li_init

x_si_li_init = 0.175            # meters (175mm lateral from hip)


# ══════════════════════════════════════════════════════════════
# MUTATION PROTECTION
# ══════════════════════════════════════════════════════════════
# FIX NEW-A: Make all numpy arrays read-only to prevent
# accidental mutation by importers. Use .copy() if you need
# to modify a value.

def _freeze_arrays():
    """Set all module-level numpy arrays to read-only."""
    import sys
    module = sys.modules[__name__]

    # Freeze standalone arrays
    for name in ['r_l0_mb_p0', 'r_G_tb_init', 'r_G_ee_init']:
        arr = getattr(module, name)
        arr.flags.writeable = False

    # Freeze dict of arrays
    for leg in r_l0_si_p0:
        r_l0_si_p0[leg].flags.writeable = False

_freeze_arrays()
