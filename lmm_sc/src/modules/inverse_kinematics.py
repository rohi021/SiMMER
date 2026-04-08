#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# Inverse Kinematics Solver for SiMMER Hexapod + Manipulator
#
# FORMULATION:
#   Uses geometric IK with Euler angle rotation matrices.
#   Each leg: 3-DOF (hip yaw, shoulder pitch, elbow pitch)
#   Manipulator: 3-DOF (base yaw, shoulder pitch, elbow pitch)
#
# CONVENTIONS:
#   r_G_tb     → trunk body position in global frame
#   r_G_ee     → end-effector position in global frame
#   r_G_li_tip → leg i tip position in global frame
#   eta_G_L0   → [roll, pitch, yaw] of body frame
#   A_G_L0     → rotation matrix from body to global (computed from eta)
#
# SAFETY:
#   All sqrt operations have domain checks.
#   Returns None + logs warning if IK is geometrically impossible.
#   Caches last valid solution as fallback.
# ══════════════════════════════════════════════════════════════

import rospy
import numpy


class ik_lmm_solver:

    # Class-level cache for last known good angles (shared across instances)
    _last_good_lmm_angles = None
    _last_good_leg_angles = None
    _last_good_man_angles = None

    def __init__(self, r_G_tb, eta_G_L0, r_G_ee, r_G_li_tip, r_l0_si_p0,
                 li, di, r_l0_mb_p0, l_man, d_man, phi, phi_m, gamma_r, gamma_l):

        # Store Euler angles
        self.c1 = numpy.cos(eta_G_L0[0])
        self.s1 = numpy.sin(eta_G_L0[0])
        self.c2 = numpy.cos(eta_G_L0[1])
        self.s2 = numpy.sin(eta_G_L0[1])
        self.c3 = numpy.cos(eta_G_L0[2])
        self.s3 = numpy.sin(eta_G_L0[2])

        # Store parameters
        self.r_G_tb = r_G_tb
        self.r_G_ee = r_G_ee
        self.r_G_li_tip = r_G_li_tip
        self.r_l0_si_p0 = r_l0_si_p0
        self.li = li
        self.di_sum = sum(di)
        self.r_l0_mb_p0 = r_l0_mb_p0
        self.l_man = l_man
        self.d_man_sum = sum(d_man)
        self.phi = phi
        self.phi_m = phi_m
        self.gamma_r = gamma_r
        self.gamma_l = gamma_l

        # FIX NEW-A: Compute A_G_L0 from actual eta_G_L0 (not hardcoded identity)
        # Rotation matrix: Rz(yaw) * Ry(pitch) * Rx(roll) — ZYX Euler convention
        self.A_G_L0 = numpy.array([
            [self.c2 * self.c3,
             self.s1 * self.s2 * self.c3 - self.c1 * self.s3,
             self.c1 * self.s2 * self.c3 + self.s1 * self.s3],
            [self.c2 * self.s3,
             self.s1 * self.s2 * self.s3 + self.c1 * self.c3,
             self.c1 * self.s2 * self.s3 - self.s1 * self.c3],
            [-self.s2,
             self.s1 * self.c2,
             self.c1 * self.c2]
        ])
        # NOTE: When eta_G_L0 = [0,0,0], this evaluates to numpy.eye(3)
        # which matches the original hardcoded behavior. No behavioral change
        # for current config, but now handles non-zero orientations correctly.

    def _safe_sqrt(self, value, context=""):
        """
        Safe square root with domain check.
        Returns None if value is negative (geometrically impossible).
        """
        if value < 0:
            # Small negative values from floating point are OK
            if value > -1e-10:
                return 0.0
            rospy.logwarn_throttle(2.0,
                "[IK] sqrt of negative value {:.6f} — {}".format(value, context))
            return None
        return numpy.sqrt(value)

    def _check_output(self, angles_dict, label=""):
        """
        Verify no nan or inf in output angles.
        Returns True if all values are finite.
        """
        for joint, angle in angles_dict.items():
            if not numpy.isfinite(angle):
                rospy.logwarn_throttle(2.0,
                    "[IK] Non-finite angle in {}: {} = {:.6f}".format(label, joint, angle))
                return False
        return True

    def solve_hexapod(self):
        """
        Solve IK for all 6 legs.
        Returns dict of joint angles, or None if any leg IK fails.
        Uses last known good angles as fallback.
        """
        leg_joint_angles = {}

        for i in range(6):
            leg = i + 1

            # Vector from hip to leg tip in body frame
            r_G_pi3_si = -(self.r_G_tb + numpy.matmul(self.A_G_L0, self.r_l0_si_p0[leg]) - self.r_G_li_tip[leg])
            ai = r_G_pi3_si[0]
            bi = r_G_pi3_si[1]
            ci = r_G_pi3_si[2]

            # Intermediate kinematic variables
            Ki1 = (ai * self.c2 * self.c3 +
                   bi * (self.c1 * self.s3 + self.s1 * self.s2 * self.c3) +
                   ci * (self.s1 * self.s3 - self.c1 * self.s2 * self.c3))
            Ki2 = (-ai * self.c2 * self.s3 +
                   bi * (self.c1 * self.c3 - self.s1 * self.s2 * self.s3) +
                   ci * (self.s1 * self.c3 + self.c1 * self.s2 * self.s3))
            Ki3 = (ai * self.s2 -
                   bi * self.s1 * self.c2 +
                   ci * self.c1 * self.c2)

            # ── DANGER 1: Ki4 — hip offset distance check ──
            Ki4_sq = Ki1**2 + Ki2**2 - self.di_sum**2
            Ki4 = self._safe_sqrt(Ki4_sq, "Leg {} Ki4: target inside hip radius".format(leg))
            if Ki4 is None:
                rospy.logwarn_throttle(2.0, "[IK] Leg {} UNREACHABLE — target inside hip joint radius".format(leg))
                return ik_lmm_solver._last_good_leg_angles

            # ── DANGER 2: Ki5 — reach check ──
            Ki5_num = ((Ki3 - self.li[0] * numpy.sin(self.phi))**2 +
                       (Ki4 - self.li[0] * numpy.cos(self.phi))**2 -
                       self.li[1]**2 - self.li[2]**2)
            Ki5_den = 2 * self.li[1] * self.li[2]

            if abs(Ki5_den) < 1e-12:
                rospy.logwarn_throttle(2.0, "[IK] Leg {} — zero link lengths".format(leg))
                return ik_lmm_solver._last_good_leg_angles

            Ki5 = Ki5_num / Ki5_den

            # ── DANGER 4: Ki5 range check for joint 3 arctan ──
            # sqrt((1-Ki5)/(1+Ki5)) requires -1 < Ki5 < 1
            if Ki5 <= -1.0 or Ki5 >= 1.0:
                # Allow minor floating-point overshoot
                if Ki5 < -1.0 and Ki5 > -1.0 - 1e-6:
                    Ki5 = -1.0 + 1e-10
                elif Ki5 > 1.0 and Ki5 < 1.0 + 1e-6:
                    Ki5 = 1.0 - 1e-10
                else:
                    rospy.logwarn_throttle(2.0,
                        "[IK] Leg {} UNREACHABLE — Ki5={:.4f} outside [-1,1] (target beyond reach)".format(leg, Ki5))
                    return ik_lmm_solver._last_good_leg_angles

            # ── DANGER 3: sqrt inside joint 2 arctan ──
            j2_sqrt_arg = ((Ki3 - self.li[0] * numpy.sin(self.phi))**2 +
                           (Ki4 - self.li[0] * numpy.cos(self.phi))**2 -
                           (self.li[1] + self.li[2] * Ki5)**2)
            j2_sqrt = self._safe_sqrt(j2_sqrt_arg, "Leg {} joint_2 inner sqrt".format(leg))
            if j2_sqrt is None:
                rospy.logwarn_throttle(2.0, "[IK] Leg {} UNREACHABLE — joint 2 geometry violated".format(leg))
                return ik_lmm_solver._last_good_leg_angles

            # ── DANGER 4: joint 3 sqrt ──
            j3_sqrt_arg = (1 - Ki5) / (1 + Ki5)
            j3_sqrt = self._safe_sqrt(j3_sqrt_arg, "Leg {} joint_3 elbow sqrt".format(leg))
            if j3_sqrt is None:
                rospy.logwarn_throttle(2.0, "[IK] Leg {} UNREACHABLE — elbow angle impossible".format(leg))
                return ik_lmm_solver._last_good_leg_angles

            # ── DANGER 5: joint 1 arctan divisor ──
            j1_divisor = self.di_sum + Ki2
            if abs(j1_divisor) < 1e-12:
                rospy.logwarn_throttle(2.0,
                    "[IK] Leg {} — joint 1 divisor near zero (target directly above hip)".format(leg))
                return ik_lmm_solver._last_good_leg_angles

            # Joint 2 arctan divisor
            j2_divisor = self.li[1] + self.li[2] * Ki5 + Ki4 - self.li[0] * numpy.cos(self.phi)
            if abs(j2_divisor) < 1e-12:
                rospy.logwarn_throttle(2.0, "[IK] Leg {} — joint 2 divisor near zero".format(leg))
                return ik_lmm_solver._last_good_leg_angles

            # ── COMPUTE JOINT ANGLES ──
            if leg in [1, 3, 5]:  # Left side
                leg_joint_angles['joint_' + str(leg) + '_1'] = (
                    self.gamma_l - (2 * numpy.arctan((Ki1 + Ki4) / j1_divisor)))
                leg_joint_angles['joint_' + str(leg) + '_2'] = (
                    -(self.phi - 2 * numpy.arctan(
                        ((Ki3 - self.li[0] * numpy.sin(self.phi)) + j2_sqrt) / j2_divisor)))
                leg_joint_angles['joint_' + str(leg) + '_3'] = (
                    -2 * numpy.arctan(j3_sqrt))

            elif leg in [2, 4, 6]:  # Right side
                leg_joint_angles['joint_' + str(leg) + '_1'] = (
                    self.gamma_r - (2 * numpy.arctan((Ki1 - Ki4) / j1_divisor)))
                leg_joint_angles['joint_' + str(leg) + '_2'] = (
                    (self.phi - 2 * numpy.arctan(
                        ((Ki3 - self.li[0] * numpy.sin(self.phi)) + j2_sqrt) / j2_divisor)))
                leg_joint_angles['joint_' + str(leg) + '_3'] = (
                    +2 * numpy.arctan(j3_sqrt))

        # FIX NEW-C: Validate output
        if not self._check_output(leg_joint_angles, "hexapod"):
            rospy.logwarn_throttle(2.0, "[IK] Hexapod output contains non-finite values — using fallback")
            return ik_lmm_solver._last_good_leg_angles

        # Cache as last known good
        ik_lmm_solver._last_good_leg_angles = dict(leg_joint_angles)

        return leg_joint_angles

    def solve_manipulator(self):
        """
        Solve IK for the 3-DOF manipulator arm.
        Returns dict of joint angles, or None if IK fails.
        Uses last known good angles as fallback.
        """
        man_joint_angles = {}

        r_G_ee_mb = -(self.r_G_tb + numpy.matmul(self.A_G_L0, self.r_l0_mb_p0) - self.r_G_ee)
        a = r_G_ee_mb[0]
        b = r_G_ee_mb[1]
        c = r_G_ee_mb[2]

        # Intermediate kinematic variables
        K1 = (a * self.c2 * self.c3 +
              b * (self.c1 * self.s3 + self.s1 * self.s2 * self.c3) +
              c * (self.s1 * self.s3 - self.c1 * self.s2 * self.c3))
        K2 = (-a * self.c2 * self.s3 +
              b * (self.c1 * self.c3 - self.s1 * self.s2 * self.s3) +
              c * (self.s1 * self.c3 + self.c1 * self.s2 * self.s3))
        K3 = (a * self.s2 -
              b * self.s1 * self.c2 +
              c * self.c1 * self.c2)

        # ── DANGER 1: K4 ──
        K4_sq = K1**2 + K2**2 - self.d_man_sum**2
        K4 = self._safe_sqrt(K4_sq, "Manipulator K4: target inside base radius")
        if K4 is None:
            rospy.logwarn_throttle(2.0, "[IK] Manipulator UNREACHABLE — target inside base joint radius")
            return ik_lmm_solver._last_good_man_angles

        # ── DANGER 2: K5 ──
        K5_num = ((K3 - self.l_man[0] * numpy.sin(self.phi_m))**2 +
                  (K4 - self.l_man[0] * numpy.cos(self.phi_m))**2 -
                  self.l_man[1]**2 - self.l_man[2]**2)
        K5_den = 2 * self.l_man[1] * self.l_man[2]

        if abs(K5_den) < 1e-12:
            rospy.logwarn_throttle(2.0, "[IK] Manipulator — zero link lengths")
            return ik_lmm_solver._last_good_man_angles

        K5 = K5_num / K5_den

        # ── DANGER 4: K5 range check ──
        if K5 <= -1.0 or K5 >= 1.0:
            if K5 < -1.0 and K5 > -1.0 - 1e-6:
                K5 = -1.0 + 1e-10
            elif K5 > 1.0 and K5 < 1.0 + 1e-6:
                K5 = 1.0 - 1e-10
            else:
                rospy.logwarn_throttle(2.0,
                    "[IK] Manipulator UNREACHABLE — K5={:.4f} outside [-1,1]".format(K5))
                return ik_lmm_solver._last_good_man_angles

        # ── DANGER 3: joint m_2 inner sqrt ──
        jm2_sqrt_arg = ((K3 - self.l_man[0] * numpy.sin(self.phi_m))**2 +
                        (K4 - self.l_man[0] * numpy.cos(self.phi_m))**2 -
                        (self.l_man[1] + self.l_man[2] * K5)**2)
        jm2_sqrt = self._safe_sqrt(jm2_sqrt_arg, "Manipulator joint_m_2 inner sqrt")
        if jm2_sqrt is None:
            rospy.logwarn_throttle(2.0, "[IK] Manipulator UNREACHABLE — joint m_2 geometry violated")
            return ik_lmm_solver._last_good_man_angles

        # ── DANGER 4: joint m_3 sqrt ──
        jm3_sqrt_arg = (1 - K5) / (1 + K5)
        jm3_sqrt = self._safe_sqrt(jm3_sqrt_arg, "Manipulator joint_m_3 elbow sqrt")
        if jm3_sqrt is None:
            rospy.logwarn_throttle(2.0, "[IK] Manipulator UNREACHABLE — elbow angle impossible")
            return ik_lmm_solver._last_good_man_angles

        # ── DANGER 5: joint m_1 divisor ──
        jm1_divisor = self.d_man_sum + K1
        if abs(jm1_divisor) < 1e-12:
            rospy.logwarn_throttle(2.0, "[IK] Manipulator — joint m_1 divisor near zero")
            return ik_lmm_solver._last_good_man_angles

        # joint m_2 divisor
        jm2_divisor = self.l_man[1] + self.l_man[2] * K5 + K4 - self.l_man[0] * numpy.cos(self.phi_m)
        if abs(jm2_divisor) < 1e-12:
            rospy.logwarn_throttle(2.0, "[IK] Manipulator — joint m_2 divisor near zero")
            return ik_lmm_solver._last_good_man_angles

        # ── COMPUTE JOINT ANGLES ──
        man_joint_angles['joint_m_1'] = -(2 * numpy.arctan((-K2 + K4) / jm1_divisor))
        man_joint_angles['joint_m_2'] = -(self.phi_m - 2 * numpy.arctan(
            ((K3 - self.l_man[0] * numpy.sin(self.phi_m)) + jm2_sqrt) / jm2_divisor))
        man_joint_angles['joint_m_3'] = -2 * numpy.arctan(jm3_sqrt)

        # FIX NEW-C: Validate output
        if not self._check_output(man_joint_angles, "manipulator"):
            rospy.logwarn_throttle(2.0, "[IK] Manipulator output contains non-finite values — using fallback")
            return ik_lmm_solver._last_good_man_angles

        # Cache as last known good
        ik_lmm_solver._last_good_man_angles = dict(man_joint_angles)

        return man_joint_angles

    def solve_lmm(self):
        """
        Solve full LMM IK (hexapod + manipulator).
        Returns dict of all 21 joint angles.
        Falls back to last known good if either sub-solver fails.
        """
        leg_joint_angles = self.solve_hexapod()
        man_joint_angles = self.solve_manipulator()

        # Handle failure cases
        if leg_joint_angles is None and man_joint_angles is None:
            rospy.logwarn_throttle(2.0, "[IK] BOTH hexapod and manipulator IK failed — using full fallback")
            if ik_lmm_solver._last_good_lmm_angles is not None:
                return dict(ik_lmm_solver._last_good_lmm_angles)
            else:
                rospy.logerr("[IK] No fallback available — first IK solve failed!")
                return {}

        if leg_joint_angles is None:
            rospy.logwarn_throttle(2.0, "[IK] Hexapod IK failed — using leg fallback only")
            if ik_lmm_solver._last_good_leg_angles is not None:
                leg_joint_angles = dict(ik_lmm_solver._last_good_leg_angles)
            else:
                rospy.logerr("[IK] No leg fallback available!")
                return dict(man_joint_angles) if man_joint_angles else {}

        if man_joint_angles is None:
            rospy.logwarn_throttle(2.0, "[IK] Manipulator IK failed — using manipulator fallback only")
            if ik_lmm_solver._last_good_man_angles is not None:
                man_joint_angles = dict(ik_lmm_solver._last_good_man_angles)
            else:
                rospy.logerr("[IK] No manipulator fallback available!")
                return dict(leg_joint_angles)

        # FIX #10: dict() creates a COPY, not a reference
        lmm_joint_angles = dict(leg_joint_angles)
        lmm_joint_angles.update(man_joint_angles)

        # Cache combined result
        ik_lmm_solver._last_good_lmm_angles = dict(lmm_joint_angles)

        return lmm_joint_angles
