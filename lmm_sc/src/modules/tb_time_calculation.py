#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# TB (Trunk Body) Time Calculation
#
# Computes the acceleration/deceleration timing profile for
# trunk body motion during gait. The profile is trapezoidal:
#
#   [t_0, t_1] → Acceleration phase (cubic blend)
#   [t_1, t_2] → Constant velocity phase
#   [t_2, t_3] → Deceleration phase (symmetric to accel)
#
# Uses fsolve to find t_1 such that the total displacement
# matches the desired trunk body displacement.
#
# INPUT:
#   r_G_tb_i   → initial trunk body position [x, y, z]
#   r_G_tb_f   → final trunk body position [x, y, z]
#   theta_c    → crab angle (direction of motion)
#   vel_tb_max → maximum trunk body velocity (m/s)
#   T_stroke   → total stroke duration (s)
#   T_in       → control sampling interval (s)
#
# OUTPUT:
#   tb_time = [t_0, t_1, t_2, t_3] — all scalars (seconds)
# ══════════════════════════════════════════════════════════════

import numpy
import rospy
from scipy.optimize import fsolve


def tb_time_calculation(r_G_tb_i, r_G_tb_f, theta_c, vel_tb_max, T_stroke, T_in):
    """
    Compute trapezoidal velocity profile timing for trunk body motion.

    Returns [t_0, t_1, t_2, t_3] where:
        t_0 → start time (always 0)
        t_1 → end of acceleration phase
        t_2 → start of deceleration phase
        t_3 → end time (= T_stroke)

    Symmetric profile: accel duration = decel duration.
    """

    # ── TIME BOUNDARIES ──────────────────────────
    t_0 = 0.0
    t_3 = t_0 + T_stroke

    # ── VELOCITY VECTORS ─────────────────────────
    # FIX NEW-E: Initial velocity is zero — don't multiply 0 by trig
    # (original had 0*cos(theta_c) which is always 0)
    rdot_G_p0_o_i = numpy.array([0.0, 0.0, 0.0])

    rdot_G_p0_o_f = numpy.array([
        vel_tb_max * numpy.cos(theta_c),
        vel_tb_max * numpy.sin(theta_c),
        0.0
    ])

    # Velocity change during acceleration
    a_vel = rdot_G_p0_o_f - rdot_G_p0_o_i  # = rdot_G_p0_o_f since initial is zero

    # FIX NEW-D: Removed dead variable del_G_tb (was computed but never used)

    # ── CHOOSE DOMINANT AXIS ─────────────────────
    # FIX NEW-C: Use axis with largest displacement to avoid
    # trivially-satisfied equation in pure X or pure Y motion.
    dx = abs(r_G_tb_f[0] - r_G_tb_i[0])
    dy = abs(r_G_tb_f[1] - r_G_tb_i[1])
    axis = 0 if dx >= dy else 1

    # Edge case: no displacement at all (shouldn't happen, but guard)
    if dx < 1e-12 and dy < 1e-12:
        rospy.logwarn_throttle(5.0,
            "[tb_time] Zero displacement — using default timing profile")
        t_1_default = T_stroke * 0.2
        return [t_0, t_1_default, t_3 - t_1_default, t_3]

    # ── EQUATION TO SOLVE ────────────────────────
    # Find t1 (acceleration phase end time) such that the
    # position at t = T_in along the dominant axis matches
    # the desired displacement.
    #
    # Position formula during acceleration (cubic blend):
    #   r(t) = r_i + (v_i * del_a + a_vel * del_a^3 * (1 - del_a/2)) / D_del_a
    # where:
    #   del_a = (t - t_0) / (t_1 - t_0)    normalized time
    #   D_del_a = 1 / (t_1 - t_0)          time scale factor

    def equation(t1_val):
        # FIX NEW-B: Guard against division by zero
        if abs(t1_val) < 1e-12:
            return 1e6  # Large residual — push fsolve away from zero

        del_a = T_in / t1_val
        velocity_profile = (rdot_G_p0_o_i * del_a +
                           a_vel * (del_a**3 * (1.0 - del_a / 2.0)))

        # Extract dominant axis component
        v_component = velocity_profile[axis]

        # Position = initial + velocity_integral * time_scale
        position = r_G_tb_i[axis] + v_component * t1_val

        return r_G_tb_f[axis] - position

    # ── SOLVE ────────────────────────────────────
    t1_guess = T_stroke * 0.1  # 10% of stroke for acceleration

    # FIX NEW-A: Use full_output to check convergence
    sol, info, ier, msg = fsolve(equation, t1_guess, full_output=True)

    # FIX #11: Extract scalar from numpy array
    t1_solved = float(sol[0])

    # FIX NEW-A: Convergence check
    if ier != 1:
        rospy.logwarn_throttle(5.0,
            "[tb_time] fsolve did not converge (ier={}): {} — using default profile".format(ier, msg.strip()))
        t1_solved = T_stroke * 0.2  # Fallback: 20% accel

    # ── SANITY CHECKS ────────────────────────────
    # t1 must be positive and less than half the stroke
    # (otherwise there's no constant velocity phase)
    if t1_solved <= 0:
        rospy.logwarn_throttle(5.0,
            "[tb_time] t1={:.4f} is non-positive — clamping to default".format(t1_solved))
        t1_solved = T_stroke * 0.1

    if t1_solved > T_stroke / 2.0:
        rospy.logwarn_throttle(5.0,
            "[tb_time] t1={:.4f} exceeds T_stroke/2={:.4f} — clamping".format(
                t1_solved, T_stroke / 2.0))
        t1_solved = T_stroke / 2.0 - T_in  # Leave at least one step for cruise

    # ── BUILD OUTPUT ─────────────────────────────
    # All values are guaranteed to be Python floats (not numpy arrays)
    t_1 = float(t_0 + t1_solved)
    t_2 = float(t_3 - t1_solved)
    tb_time = [t_0, t_1, t_2, t_3]

    return tb_time
