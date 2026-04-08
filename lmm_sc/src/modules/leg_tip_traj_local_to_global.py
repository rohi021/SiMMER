#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# Leg Tip Trajectory — Local to Global Frame Transformation
#
# PURPOSE:
#   Converts per-leg foot tip trajectories from LOCAL frame
#   (relative to foot starting position) to GLOBAL frame
#   (absolute world coordinates).
#
# TRANSFORMATION FORMULA:
#   r_global[i] = r_tb + r_tb_to_tip + r_local[i]
#
#   Where:
#     r_tb          = trunk body position in global frame (3D vector)
#     r_tb_to_tip   = TF transform: Trunk_Body → Leg_X_Tip
#                     (current foot position relative to body)
#     r_local[i]    = i-th waypoint in local trajectory
#                     (displacement from starting foot position)
#
# TF LOOKUP:
#   Uses TF2 buffer to get Trunk_Body → Leg_X_Tip at planning time.
#   This captures the CURRENT foot position as the trajectory start.
#   The local trajectory is then ADDED as displacement from there.
#
# INPUT:
#   tb_pos              → [x, y, z] trunk body position (numpy array)
#   leg_tip_traj_local  → dict {leg: list of [x,y,z]} from leg_tip_traj_local.py
#   tfBuffer            → tf2_ros.Buffer with populated TF tree
#
# OUTPUT:
#   dict {leg: list of [x,y,z]} in global frame
#   Each inner list is a Python list (not numpy array) for pop() in master.py
#
# CALLED BY:
#   master.py → callback → gait planning branch
#   Result stored in traj_r_G_leg_tip, popped one waypoint at a time
# ══════════════════════════════════════════════════════════════

import rospy
import tf2_ros
import numpy
import time


def traj_local_to_global(tb_pos, leg_tip_traj_local, tfBuffer):
    """
    Transform leg tip trajectories from local to global frame.

    Args:
        tb_pos:             Trunk body position [x,y,z] as numpy array
        leg_tip_traj_local: Dict {leg_num: list of [x,y,z] waypoints} in local frame
        tfBuffer:           tf2_ros.Buffer with TF tree

    Returns:
        Dict {leg_num: list of [x,y,z] waypoints} in global frame.
        Returns empty trajectories (all zeros) if TF lookup fails.
    """

    # FIX NEW-H: Defensive copy of trunk body position
    # Prevents corruption if caller modifies tb_pos during gait execution
    tb_pos_safe = numpy.array(tb_pos, dtype=float).copy()

    # ── VALIDATE INPUT SHAPES ────────────────────
    # FIX NEW-B: Verify all legs have same waypoint count

    leg_counts = {}
    for leg in range(1, 7):
        if leg in leg_tip_traj_local:
            leg_counts[leg] = len(leg_tip_traj_local[leg])
        else:
            rospy.logwarn_throttle(5.0,
                "[local_to_global] Leg {} missing from local trajectory!".format(leg))
            leg_counts[leg] = 0

    unique_counts = set(leg_counts.values()) - {0}
    if len(unique_counts) > 1:
        rospy.logwarn_throttle(5.0,
            "[local_to_global] Legs have different waypoint counts: {} — using minimum".format(
                leg_counts))

    if not unique_counts:
        rospy.logwarn_throttle(5.0,
            "[local_to_global] All legs have 0 waypoints — returning empty")
        return {leg: [] for leg in range(1, 7)}

    n = min(unique_counts)  # Use minimum count to prevent shape mismatch

    # ── TB POSITION MATRIX ───────────────────────
    # Repeat TB position for each waypoint: shape (n, 3)
    tb_pos_matrix = numpy.tile(tb_pos_safe, (n, 1))

    # ── TF LOOKUPS ───────────────────────────────
    # Get current foot tip positions relative to trunk body.
    # These are FIXED at planning time (current standing pose).

    # FIX NEW-A: Error handling for TF lookups
    lookup_start = time.time()

    tf_translations = {}
    leg_names = {
        1: 'Leg_1_Tip',
        2: 'Leg_2_Tip',
        3: 'Leg_3_Tip',
        4: 'Leg_4_Tip',
        5: 'Leg_5_Tip',
        6: 'Leg_6_Tip',
    }

    for leg, tip_frame in leg_names.items():
        try:
            tf_stamped = tfBuffer.lookup_transform(
                'Trunk_Body', tip_frame, rospy.Time(0))

            tf_translations[leg] = numpy.array([
                tf_stamped.transform.translation.x,
                tf_stamped.transform.translation.y,
                tf_stamped.transform.translation.z,
            ])

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0,
                "[local_to_global] TF lookup failed for '{}': {} — "
                "using zero offset (foot at body center)".format(tip_frame, e))
            tf_translations[leg] = numpy.array([0.0, 0.0, 0.0])

    # FIX NEW-D: Check lookup timing
    lookup_duration = time.time() - lookup_start
    if lookup_duration > 0.05:  # > 50ms for 6 lookups is unusually slow
        rospy.logwarn_throttle(5.0,
            "[local_to_global] TF lookups took {:.3f}s (expected <0.01s)".format(
                lookup_duration))

    # ── TRANSFORM TO GLOBAL ──────────────────────
    leg_tip_traj_global = {}

    for leg in range(1, 7):
        if leg_counts[leg] == 0:
            leg_tip_traj_global[leg] = []
            continue

        # Tile the TF offset: shape (n, 3)
        # FIX NEW-C: Renamed from tf_leg_to_tb (wrong direction)
        tf_tb_to_tip_matrix = numpy.tile(tf_translations[leg], (n, 1))

        # Convert local trajectory to numpy array: shape (n, 3)
        # Truncate to n if this leg has more waypoints
        local_traj = numpy.array(leg_tip_traj_local[leg][:n], dtype=float)

        # Global position = TB + TF_offset + local_displacement
        global_traj = tb_pos_matrix + tf_tb_to_tip_matrix + local_traj

        # Convert to list-of-lists for master.py's pop(0) usage
        # NOTE: This double conversion (numpy→list→numpy in IK) is intentional.
        # master.py uses list.pop(0) for O(1)-ish waypoint consumption.
        # IK auto-converts back to numpy for math operations.
        leg_tip_traj_global[leg] = global_traj.tolist()

    return leg_tip_traj_global
