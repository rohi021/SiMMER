#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# Leg Tip Trajectory — Local Frame
#
# Generates per-leg foot tip trajectories in the LOCAL frame
# (relative to each foot's starting position on the ground).
#
# TWO TYPES OF LEGS PER STROKE:
#   Support legs → stay at [0,0,0] (body moves over them)
#   Swing legs   → lift, move forward, set down
#
# SWING TRAJECTORY PROFILE:
#
#   X-axis (forward): Cubic S-curve over full stroke
#     x(δ) = swing_length * δ² * (3 - 2δ)    δ ∈ [0,1]
#     Smooth start + smooth stop, zero velocity at endpoints.
#
#   Y-axis: Always 0 (no lateral motion in local frame)
#
#   Z-axis (height): Three-phase trapezoidal profile
#     Phase 1 [t0→t1]: Lift   — cubic blend 0 → max_height
#     Phase 2 [t1→t2]: Cruise — constant max_height  
#     Phase 3 [t2→t3]: Place  — cubic blend max_height → hi3_dash
#
#   Side view:
#           ╭──────────────────╮ max_height = Hmi1 + del_h
#          ╱                    ╲
#         ╱                      ╲
#        ╱                        ╲
#     ●─╱                          ╲─●  hi3_dash
#     P  ├─ lift ─┤── cruise ──┤─ place ─┤  P'
#           20%        60%         20%
#
# OUTPUT:
#   Dict {leg_num: list of [x,y,z] waypoints} in LOCAL frame.
#   These are later transformed to global frame by
#   leg_tip_traj_local_to_global.py
#
# CALLED BY:
#   master.py → callback → gait planning branch
# ══════════════════════════════════════════════════════════════

import rospy
import numpy


class leg_tip_traj_local:
    """
    Generates local-frame foot tip trajectories for one gait stroke.

    Args:
        support_legs:  List of leg numbers that support (stay on ground)
        swing_length:  Total forward distance for swing legs (meters)
        theta_c:       Crab angle — direction of motion (radians)
        T_stroke:      Stroke duration (seconds)
        T_in:          Control loop interval (seconds)
        Hmi1:          Base swing height (meters)
        del_h:         Additional height margin (meters)
        hi3_dash:      Leg tip Z at end of swing (meters)
        lt_time_ratio: Fraction of stroke for lift/place phases (0-0.5)
    """

    def __init__(self, support_legs, swing_length, theta_c, T_stroke,
                 T_in, Hmi1, del_h, hi3_dash, lt_time_ratio):

        self.support_legs = support_legs
        self.swing_length = swing_length
        self.theta_c = theta_c
        self.T_in = T_in
        self.Hmi1 = Hmi1
        self.del_h = del_h
        self.hi3_dash = hi3_dash

        # Phase timing: [start, end_lift, start_place, end]
        self.lt_time = [
            0.0,
            T_stroke * lt_time_ratio,          # End of lift phase
            T_stroke * (1.0 - lt_time_ratio),  # Start of place phase
            T_stroke,                           # End of stroke
        ]

        # Total waypoints for this stroke
        self.N1 = int(numpy.ceil(T_stroke / T_in))

        # FIX NEW-A: Minimum 1 waypoint
        if self.N1 < 1:
            rospy.logwarn_throttle(5.0,
                "[leg_traj] T_stroke ({:.4f}) < T_in ({:.4f}) — using 1 waypoint".format(
                    T_stroke, T_in))
            self.N1 = 1

        # Crab angle rotation matrix (rotates local trajectory into crab direction)
        c = numpy.cos(theta_c)
        s = numpy.sin(theta_c)
        self.rot_crab = numpy.array([
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0]
        ])

    def _cubic_blend(self, delta):
        """
        Cubic S-curve interpolation.
        f(δ) = δ² * (3 - 2δ)

        Properties:
            f(0) = 0, f(1) = 1
            f'(0) = 0, f'(1) = 0 (zero velocity at endpoints)

        Args:
            delta: Normalized time ∈ [0, 1]

        Returns:
            Interpolation factor ∈ [0, 1]
        """
        return delta**2 * (3.0 - 2.0 * delta)

    def _support_trajectory(self):
        """
        Generate trajectory for a support leg.
        Support legs stay at origin (body moves over them).

        Returns:
            List of N1 waypoints, each [0.0, 0.0, 0.0]
        """
        return [[0.0, 0.0, 0.0] for _ in range(self.N1)]

    def _swing_trajectory(self):
        """
        Generate trajectory for a swing leg in local XZ plane.
        X: cubic S-curve over full stroke duration.
        Y: always 0 (lateral motion handled by crab rotation).
        Z: three-phase lift/cruise/place profile.

        Returns:
            List of N1 waypoints, each [x, y, z]
        """
        t0 = self.lt_time[0]
        t1 = self.lt_time[1]
        t2 = self.lt_time[2]
        t3 = self.lt_time[3]

        # Z-axis control points
        z_start = 0.0                          # Ground level at lift-off
        z_max = self.Hmi1 + self.del_h         # Maximum swing height
        z_end = self.hi3_dash                   # Ground level at touch-down

        # X-axis control points
        x_start = 0.0                          # Starting position
        x_end = self.swing_length              # Forward displacement

        # Height amplitudes for cubic blend
        a_z_lift = z_max - z_start             # Height gained during lift
        a_z_place = z_end - z_max              # Height change during place (negative)

        waypoints = []

        for N in range(self.N1):
            t = (N + 1) * self.T_in

            # FIX NEW-E: Clamp to stroke end (handles ceil overshoot)
            t = min(t, t3)

            # ── X-AXIS: Cubic S-curve over full stroke ──
            if t3 > 0:
                delta_x = t / t3               # Normalized time [0, 1]
            else:
                delta_x = 1.0
            x = x_start + (x_end - x_start) * self._cubic_blend(delta_x)

            # ── Y-AXIS: Always zero in local frame ──
            y = 0.0

            # ── Z-AXIS: Three-phase profile ──
            if t <= t1:
                # Phase 1: LIFT — cubic blend from ground to max height
                if (t1 - t0) > 1e-12:
                    delta_z = (t - t0) / (t1 - t0)
                else:
                    delta_z = 1.0
                z = z_start + a_z_lift * self._cubic_blend(delta_z)

            elif t <= t2:
                # Phase 2: CRUISE — constant max height
                z = z_max

            else:
                # Phase 3: PLACE — cubic blend from max height to ground
                #
                # NOTE ON REVERSED δ:
                #   delta_z = (t3 - t) / (t3 - t2) counts DOWN from 1→0
                #   At t=t2: δ=1 → z = z_end - a_z_place*(3-2) = z_end - a_z_place = z_max ✓
                #   At t=t3: δ=0 → z = z_end ✓
                #   This reversed convention avoids computing (1 - cubic_blend(forward_δ))
                if (t3 - t2) > 1e-12:
                    delta_z = (t3 - t) / (t3 - t2)
                else:
                    delta_z = 0.0
                z = z_end - a_z_place * self._cubic_blend(delta_z)

            waypoints.append([x, y, z])

        return waypoints

    def calculate_trajectory(self):
        """
        Generate trajectories for all 6 legs, rotated by crab angle.

        Support legs → zero trajectory (stay in place).
        Swing legs → swing trajectory rotated by theta_c.

        Returns:
            Dict {leg_number: list of [x,y,z] waypoints}
            All in local frame, rotated by crab angle.
        """
        leg_tip_trajectory_local = {}

        for i in range(6):
            leg = i + 1

            if leg in self.support_legs:
                traj = numpy.array(self._support_trajectory(), dtype=float)
            else:
                traj = numpy.array(self._swing_trajectory(), dtype=float)

            # Rotate trajectory by crab angle
            rot_traj = numpy.matmul(self.rot_crab, traj.T).T
            leg_tip_trajectory_local[leg] = rot_traj.tolist()

        return leg_tip_trajectory_local

    # FIX #34: Backward-compatible alias for old misspelled name
    # This allows any code we haven't updated yet to still work.
    # Remove this alias after confirming all callers are updated.
    calculate_trajectry = calculate_trajectory
