#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# TB (Trunk Body) Trajectory Generator
#
# Generates position waypoints for trunk body motion using a
# trapezoidal velocity profile with cubic blend transitions.
#
# VELOCITY PROFILE:
#   [t_0, t_1] → Cubic acceleration from 0 to vel_tb_max
#   [t_1, t_2] → Constant velocity at vel_tb_max
#   [t_2, t_3] → Cubic deceleration from vel_tb_max to 0
#
# INPUT:
#   theta_c    → crab angle (direction of motion, radians)
#   vel_tb_max → maximum trunk body velocity (m/s)
#   tb_pos_i   → initial position [x, y, z] (numpy array)
#   tb_time    → [t_0, t_1, t_2, t_3] timing profile (from tb_time_calculation)
#   T_in       → sampling interval (seconds)
#
# OUTPUT:
#   List of numpy arrays [x, y, z] — one per timestep
#   Length = ceil(T_stroke / T_in)
#   Each element is a 3D position vector
#
# CALLED BY:
#   master.py → callback → gait planning branch
# ══════════════════════════════════════════════════════════════

import numpy
import rospy


def tb_trajectory(theta_c, vel_tb_max, tb_pos_i, tb_time, T_in):
    """
    Generate trunk body position waypoints along a trapezoidal velocity profile.

    Args:
        theta_c:    Crab angle (radians) — direction of motion
        vel_tb_max: Maximum trunk body velocity (m/s)
        tb_pos_i:   Initial position as numpy array [x, y, z]
        tb_time:    Timing profile [t_0, t_1, t_2, t_3] from tb_time_calculation
        T_in:       Sampling interval (seconds)

    Returns:
        List of numpy arrays, each [x, y, z] — position at each timestep
    """

    # ── VELOCITY VECTORS ─────────────────────────
    # FIX NEW-D: Initial velocity is zero — no need for 0*cos(theta_c)
    v_initial = numpy.array([0.0, 0.0, 0.0])

    v_final = numpy.array([
        vel_tb_max * numpy.cos(theta_c),
        vel_tb_max * numpy.sin(theta_c),
        0.0
    ])

    # Velocity change during acceleration (= v_final since v_initial = 0)
    a_vel = v_final - v_initial

    # ── TIMING ───────────────────────────────────
    t_0 = tb_time[0]
    t_1 = tb_time[1]
    t_2 = tb_time[2]
    t_3 = tb_time[3]

    # ── ZERO-DURATION GUARDS ─────────────────────
    # FIX NEW-A: Prevent division by zero if any phase has zero duration

    accel_duration = t_1 - t_0
    cruise_duration = t_2 - t_1
    decel_duration = t_3 - t_2

    if accel_duration < 1e-12:
        rospy.logwarn_throttle(5.0,
            "[tb_traj] Acceleration phase has zero duration — using minimum")
        accel_duration = T_in  # Minimum one timestep
        t_1 = t_0 + accel_duration

    if decel_duration < 1e-12:
        rospy.logwarn_throttle(5.0,
            "[tb_traj] Deceleration phase has zero duration — using minimum")
        decel_duration = T_in
        t_2 = t_3 - decel_duration

    # Time scale factors (reciprocal of phase duration)
    D_del_a = 1.0 / accel_duration
    D_del_d = 1.0 / decel_duration

    # ── PRECOMPUTE KEY POSITIONS ─────────────────
    # Position at end of acceleration (t = t_1):
    #   del_a = 1.0 (fully through acceleration phase)
    #   r(t1) = r_i + [v_i*1 + a_vel*(1^3 * (1-0.5))] * accel_duration
    #         = r_i + [0 + a_vel * 0.5] * accel_duration
    #         = r_i + 0.5 * v_final * accel_duration
    pos_at_t1 = tb_pos_i + (v_initial + a_vel * 0.5) * accel_duration

    # Position at end of cruise (t = t_2):
    #   r(t2) = r(t1) + v_final * cruise_duration
    pos_at_t2 = pos_at_t1 + v_final * cruise_duration

    # ── POSITION HELPER FUNCTIONS ────────────────
    # FIX NEW-C: Extracted from repeated inline formulas

    def position_accel(t):
        """Position during acceleration phase [t_0, t_1]."""
        delta = (t - t_0) * D_del_a  # Normalized time [0, 1]
        return tb_pos_i + (v_initial * delta + a_vel * (delta**3 * (1.0 - delta / 2.0))) / D_del_a

    def position_cruise(t):
        """Position during constant velocity phase [t_1, t_2]."""
        return pos_at_t1 + v_final * (t - t_1)

    def position_decel(t):
        """Position during deceleration phase [t_2, t_3]."""
        delta = (t - t_2) * D_del_d  # Normalized time [0, 1]
        return pos_at_t2 + (v_final * delta - a_vel * (delta**3 * (1.0 - delta / 2.0))) / D_del_d

    # ── GENERATE WAYPOINTS ───────────────────────

    N1 = int(numpy.ceil(t_3 / T_in))

    # FIX NEW-B: Ensure at least one waypoint
    if N1 < 1:
        rospy.logwarn_throttle(5.0,
            "[tb_traj] T_stroke ({:.4f}) < T_in ({:.4f}) — generating 1 waypoint at final position".format(
                t_3, T_in))
        N1 = 1

    waypoints = []

    for N in range(N1):
        t = (N + 1) * T_in

        # FIX #36: Clamp t to t_3 to handle ceil() overshoot
        # This eliminates the redundant else branch entirely
        t_clamped = min(t, t_3)

        if t_clamped <= t_1:
            # Acceleration phase
            waypoints.append(position_accel(t_clamped))

        elif t_clamped <= t_2:
            # Constant velocity phase
            waypoints.append(position_cruise(t_clamped))

        else:
            # Deceleration phase (includes t > t_3 via clamping)
            waypoints.append(position_decel(t_clamped))

    return waypoints
