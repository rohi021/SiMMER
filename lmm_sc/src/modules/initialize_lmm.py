#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# LMM Initialization — Standing Pose Calculator
#
# PURPOSE:
#   Computes the initial "home" standing pose for the hexapod
#   robot and manipulator arm. Called ONCE at startup.
#
# STANDING POSE GEOMETRY:
#
#   TOP VIEW (looking down, Z up):
#
#       Leg 1 ●───────────────● Leg 2
#       (-X)  │               │  (+X)
#             │   Trunk Body  │
#       Leg 3 ●───────────────● Leg 4
#             │               │
#       Leg 5 ●───────────────● Leg 6
#
#       ←─ x_si_li_init ─→←─ x_si_li_init ─→
#       (0.175m from hip)   (0.175m from hip)
#
#   SIDE VIEW:
#
#       ┌── Trunk Body ──┐  Z = 0.125m
#       │   [hip joint]  │  Z = 0.125 + 0.0155
#       │       │        │
#       │      leg       │
#       │       │        │
#       ●───── tip ──────●  Z = 0.0 (ground)
#
# FOOT TIP FORMULA:
#   For each leg i (1-6):
#     r_tip[i] = r_tb + r_hip[i] + [sign_x * x_offset, 0, -z_above_ground]
#
#   Where:
#     sign_x = -1 for left legs (1,3,5), +1 for right legs (2,4,6)
#     z_above_ground = r_tb[2] + r_hip[i][2]
#     This guarantees ALL feet are at Z = 0 (ground level)
#
# OUTPUT:
#   lmm_joint_angles:  dict of 21 joint angles (6 legs × 3 + manip × 3)
#   r_G_li_tip_init:   dict of 6 foot positions [x,y,z] in global frame
#
# CALLED BY:
#   master.py — once at startup, before control loop begins
# ══════════════════════════════════════════════════════════════

import rospy
import numpy
import inverse_kinematics


# Lateral sign map: which direction each leg extends from the hip
# Left legs (1,3,5) extend in -X; Right legs (2,4,6) extend in +X
# Explicit map is clearer than ((-1)**leg) and robust to renumbering
LEG_LATERAL_SIGN = {
    1: -1.0,   # Left-Front
    2: +1.0,   # Right-Front
    3: -1.0,   # Left-Middle
    4: +1.0,   # Right-Middle
    5: -1.0,   # Left-Rear
    6: +1.0,   # Right-Rear
}


def lmm_joint_angles_init(r_G_tb_init, r_G_ee_init, x_si_li_init,
                           eta_G_L0, r_l0_si_p0, li, di,
                           r_l0_mb_p0, l_man, d_man, phi, phi_m,
                           gamma_r, gamma_l):
    """
    Compute initial standing pose joint angles and foot positions.

    This function:
      1. Computes foot tip positions for a symmetric standing stance
      2. Calls the full LMM IK solver to find joint angles
      3. Validates the result

    Args:
        r_G_tb_init:   Initial trunk body position [x,y,z] (m)
        r_G_ee_init:   Initial end-effector position [x,y,z] (m)
        x_si_li_init:  Lateral foot offset from hip (m)
        eta_G_L0:      Body frame Euler angles [roll, pitch, yaw] (rad)
        r_l0_si_p0:    Dict {leg: hip position [x,y,z]} (m)
        li:            Leg link lengths [L0, L1, L2] (m)
        di:            Leg link offsets [d0, d1, d2] (m)
        r_l0_mb_p0:    Manipulator base position [x,y,z] (m)
        l_man:         Manipulator link lengths [L0, L1, L2] (m)
        d_man:         Manipulator link offsets [d0, d1, d2] (m)
        phi:           Leg mounting angle (rad)
        phi_m:         Manipulator mounting angle (rad)
        gamma_r:       Right leg hip offset angle (rad)
        gamma_l:       Left leg hip offset angle (rad)

    Returns:
        (lmm_joint_angles, r_G_li_tip_init):
            lmm_joint_angles: dict {joint_name: angle_rad} — 21 joints
            r_G_li_tip_init:  dict {leg_num: numpy.array([x,y,z])} — 6 foot positions

    Raises:
        RuntimeError: If IK solver fails to compute initial pose
    """

    rospy.loginfo("[init_lmm] Computing initial standing pose...")
    rospy.loginfo("[init_lmm]   TB position: [{:.4f}, {:.4f}, {:.4f}]".format(
        r_G_tb_init[0], r_G_tb_init[1], r_G_tb_init[2]))
    rospy.loginfo("[init_lmm]   EE position: [{:.4f}, {:.4f}, {:.4f}]".format(
        r_G_ee_init[0], r_G_ee_init[1], r_G_ee_init[2]))
    rospy.loginfo("[init_lmm]   Lateral offset: {:.4f}m".format(x_si_li_init))

    # ── MAXIMUM REACH CHECK ──────────────────────
    # FIX NEW-D: Validate that feet are within reachable workspace
    max_reach = li[0] + li[1] + li[2]
    min_reach = abs(li[1] - li[2])

    # ── COMPUTE FOOT TIP POSITIONS ───────────────
    r_G_li_tip_init = {}

    for i in range(6):
        leg = i + 1

        # FIX NEW-B: Explicit sign map instead of ((-1)**leg)
        sign_x = LEG_LATERAL_SIGN[leg]

        # Hip position in body frame
        hip_pos = r_l0_si_p0[leg]

        # FIX NEW-C: Z-coordinate derivation documented
        # We want feet on the ground (Z = 0 in global frame).
        # Foot Z in global = r_tb_z + hip_z + foot_offset_z
        # Setting foot Z = 0:
        #   0 = r_tb_z + hip_z + foot_offset_z
        #   foot_offset_z = -(r_tb_z + hip_z)
        foot_offset_z = -(r_G_tb_init[2] + hip_pos[2])

        # Foot tip position in global frame
        # FIX NEW-G: Return copies to prevent external mutation
        tip_position = (
            numpy.array(r_G_tb_init, dtype=float) +
            numpy.array(hip_pos, dtype=float) +
            numpy.array([sign_x * x_si_li_init, 0.0, foot_offset_z])
        )

        # FIX NEW-D: Reachability check
        # Vector from hip to foot (in global, before IK transforms)
        hip_to_foot = tip_position - (numpy.array(r_G_tb_init) + numpy.array(hip_pos))
        reach_distance = numpy.linalg.norm(hip_to_foot)

        if reach_distance > max_reach:
            rospy.logerr(
                "[init_lmm] Leg {} UNREACHABLE! Distance {:.4f}m > max reach {:.4f}m. "
                "Reduce x_si_li_init ({:.4f}m) or check body height ({:.4f}m).".format(
                    leg, reach_distance, max_reach, x_si_li_init, r_G_tb_init[2]))
        elif reach_distance < min_reach:
            rospy.logwarn(
                "[init_lmm] Leg {} near singularity! Distance {:.4f}m < min reach {:.4f}m.".format(
                    leg, reach_distance, min_reach))
        elif reach_distance > 0.9 * max_reach:
            rospy.logwarn(
                "[init_lmm] Leg {} near extension limit! Distance {:.4f}m = {:.0f}% of max.".format(
                    leg, reach_distance, 100.0 * reach_distance / max_reach))

        r_G_li_tip_init[leg] = tip_position.copy()

        rospy.loginfo("[init_lmm]   Leg {} tip: [{:.4f}, {:.4f}, {:.4f}] (reach: {:.4f}m / {:.4f}m)".format(
            leg, tip_position[0], tip_position[1], tip_position[2],
            reach_distance, max_reach))

    # ── VERIFY ALL FEET ON GROUND ────────────────
    for leg in range(1, 7):
        z = r_G_li_tip_init[leg][2]
        if abs(z) > 1e-6:
            rospy.logwarn("[init_lmm] Leg {} Z={:.6f} — should be 0 (ground)!".format(leg, z))

    # ── SOLVE IK FOR STANDING POSE ───────────────
    rospy.loginfo("[init_lmm] Solving IK for standing pose...")

    ik_lmm = inverse_kinematics.ik_lmm_solver(
        r_G_tb_init, eta_G_L0, r_G_ee_init, r_G_li_tip_init,
        r_l0_si_p0, li, di, r_l0_mb_p0, l_man, d_man,
        phi, phi_m, gamma_r, gamma_l
    )

    lmm_joint_angles_result = ik_lmm.solve_lmm()

    # FIX NEW-A: Validate IK result
    if lmm_joint_angles_result is None or len(lmm_joint_angles_result) == 0:
        error_msg = ("[init_lmm] FATAL: IK solver failed to compute initial standing pose! "
                     "Check: body height, lateral offset, link lengths, EE position.")
        rospy.logfatal(error_msg)
        raise RuntimeError(error_msg)

    # Verify all 21 joints are present
    expected_joints = []
    for leg in range(1, 7):
        for j in range(1, 4):
            expected_joints.append('joint_{}_{}'.format(leg, j))
    for j in range(1, 4):
        expected_joints.append('joint_m_{}'.format(j))

    missing = [j for j in expected_joints if j not in lmm_joint_angles_result]
    if missing:
        rospy.logwarn("[init_lmm] Missing joints in IK result: {}".format(missing))

    # Verify no NaN/Inf in joint angles
    for joint, angle in lmm_joint_angles_result.items():
        if not numpy.isfinite(angle):
            error_msg = "[init_lmm] FATAL: Joint '{}' has non-finite angle: {}".format(
                joint, angle)
            rospy.logfatal(error_msg)
            raise RuntimeError(error_msg)

    # ── LOG RESULT ───────────────────────────────
    rospy.loginfo("[init_lmm] ✅ Standing pose computed — {} joints:".format(
        len(lmm_joint_angles_result)))

    for joint in sorted(lmm_joint_angles_result.keys()):
        angle_deg = numpy.degrees(lmm_joint_angles_result[joint])
        rospy.loginfo("[init_lmm]   {}: {:.4f} rad ({:.1f}°)".format(
            joint, lmm_joint_angles_result[joint], angle_deg))

    return lmm_joint_angles_result, r_G_li_tip_init
