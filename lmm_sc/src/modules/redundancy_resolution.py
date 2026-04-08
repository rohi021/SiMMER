#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# Redundancy Resolution for SiMMER Hexapod Manipulator
#
# When the manipulator arm approaches its workspace limit
# (manipulability drops below threshold), this module computes
# optimal trunk body displacement + joint angle adjustment
# to restore manipulability while minimizing deviation from
# current configuration.
#
# OPTIMIZATION PROBLEM:
#   Decision variables: x = [q1, q2, q3, x_tb, y_tb]
#     q1  → manipulator base yaw angle (rad)
#     q2  → manipulator shoulder pitch angle (rad)
#     q3  → manipulator elbow pitch angle (rad)
#     x_tb → trunk body X position (m)
#     y_tb → trunk body Y position (m)
#
#   Objective: minimize weighted squared deviation from current x_i
#     min Σ w[i] * (x[i] - x_i[i])²
#
#   Subject to:
#     EE position constraints (3 equality) — arm must reach target
#     Manipulability constraint (1 inequality) — must exceed threshold
#     Stroke length bounds (2 inequality) — TB displacement in [min, max]
#
# CALLED BY:
#   master.py → callback → Branch 2 (inputs nonzero + manipulability low)
#
# DEPENDENCIES:
#   scipy.optimize.minimize (SLSQP method)
# ══════════════════════════════════════════════════════════════

import rospy
import numpy

try:
    from scipy.optimize import minimize
except ImportError:
    rospy.logfatal("[redundancy] scipy not installed! Run: pip3 install scipy")
    raise


# ── STROKE LENGTH BOUNDS ─────────────────────────────
# These constrain how far the trunk body can move per optimization step.
# FIX NEW-E: Named constants with units documentation
MIN_STROKE_LENGTH = 9e-5     # 0.09 mm — minimum TB displacement (prevents zero-motion)
MAX_STROKE_LENGTH = 1.25e-3  # 1.25 mm — maximum TB displacement (legs can track)


class redundancy_resolver:
    """
    Resolves manipulator redundancy by optimizing trunk body position
    and joint angles to maintain end-effector manipulability.
    """

    def __init__(self, w, l_man, r_l0_mb_p0, m_t, r_G_tb, r_G_ee, x_i, bounds):
        """
        Args:
            w:          Weight vector [w_q1, w_q2, w_q3, w_xtb, w_ytb] for objective
            l_man:      Manipulator link lengths [l0, l1, l2] (m)
            r_l0_mb_p0: Vector from body frame to manipulator base [x, y, z] (m)
            m_t:        Manipulability threshold (dimensionless, typically 0.25)
            r_G_tb:     Current trunk body position [x, y, z] (m)
            r_G_ee:     Current end-effector position [x, y, z] (m)
            x_i:        Current configuration [q1, q2, q3, x_tb, y_tb]
            bounds:     Joint/position bounds for optimizer
        """
        # FIX NEW-G: Validate input dimensions
        assert len(w) == 5, "Weight vector must have 5 elements, got {}".format(len(w))
        assert len(x_i) == 5, "Initial config must have 5 elements, got {}".format(len(x_i))

        self.w = w
        self.l_man = l_man
        self.r_l0_mb_p0 = r_l0_mb_p0
        self.m_t = m_t
        self.r_G_tb = r_G_tb
        self.r_G_ee = r_G_ee
        self.x_i = x_i
        self.bounds = bounds

    def objective(self, x):
        """
        Weighted sum of squared deviations from initial configuration.
        Minimizing this keeps the robot close to its current pose.
        """
        return (self.w[0] * (x[0] - self.x_i[0])**2 +
                self.w[1] * (x[1] - self.x_i[1])**2 +
                self.w[2] * (x[2] - self.x_i[2])**2 +
                self.w[3] * (x[3] - self.x_i[3])**2 +
                self.w[4] * (x[4] - self.x_i[4])**2)

    def ee_x_constraint(self, x):
        """EE X-position equality constraint: FK_x(x) - target_x = 0"""
        return (x[3] +
                self.r_l0_mb_p0[0] +
                self.l_man[1] * numpy.sin(x[0]) * numpy.sin(x[1]) +
                self.l_man[2] * numpy.sin(x[0]) * numpy.sin(x[1] + x[2]) -
                self.r_G_ee[0])

    def ee_y_constraint(self, x):
        """EE Y-position equality constraint: FK_y(x) - target_y = 0"""
        return (x[4] +
                self.r_l0_mb_p0[1] -
                self.l_man[1] * numpy.sin(x[1]) * numpy.cos(x[0]) -
                self.l_man[2] * numpy.cos(x[0]) * numpy.sin(x[1] + x[2]) -
                self.r_G_ee[1])

    def ee_z_constraint(self, x):
        """EE Z-position equality constraint: FK_z(x) - target_z = 0"""
        return (self.r_G_tb[2] +
                self.r_l0_mb_p0[2] +
                self.l_man[0] +
                self.l_man[1] * numpy.cos(x[1]) +
                self.l_man[2] * numpy.cos(x[1] + x[2]) -
                self.r_G_ee[2])

    def ee_manipulability_constraint(self, x, m_t=None):
        """
        Manipulability inequality constraint: m(x) - m_threshold ≥ 0

        Manipulability = |det(J)| / normalization_factor
        where J is the 3×3 manipulator Jacobian and normalization
        makes it dimensionless (0 = singular, 1 = optimal).

        Args:
            x:   Configuration [q1, q2, q3, x_tb, y_tb]
            m_t: Manipulability threshold. None → use self.m_t

        Returns:
            manipulability - threshold (≥ 0 means constraint satisfied)
        """
        # FIX #12: Use "is None" instead of "is 0" (identity vs value)
        # Original: "if m_t is 0" — works by CPython luck for int 0,
        # fails silently for float 0.0, SyntaxWarning in Python 3.8+
        if m_t is None:
            m_t = self.m_t

        # Extract joint angles for readability
        q1 = x[0]
        q2 = x[1]
        q3 = x[2]
        l1 = self.l_man[1]
        l2 = self.l_man[2]

        # FIX NEW-D: Jacobian broken into readable components
        # Each element corresponds to d(FK_i)/d(q_j)
        #
        # Common sub-expressions:
        s1 = numpy.sin(q1)
        c1 = numpy.cos(q1)
        s2 = numpy.sin(q2)
        c2 = numpy.cos(q2)
        s23 = numpy.sin(q2 + q3)
        c23 = numpy.cos(q2 + q3)

        # Arm reach components
        reach_s = l1 * s2 + l2 * s23    # radial reach (sin component)
        reach_c = l1 * c2 + l2 * c23    # axial reach (cos component)

        # Jacobian: J[i][j] = d(position_i) / d(joint_j)
        #
        # position_x = x_tb + r_mb[0] + l1*s1*s2 + l2*s1*s23
        # position_y = y_tb + r_mb[1] - l1*s2*c1 - l2*c1*s23
        # position_z = z_tb + r_mb[2] + l0 + l1*c2 + l2*c23
        J = numpy.array([
            [ c1 * reach_s,        s1 * reach_c,        l2 * s1 * c23],   # d(x)/d(q1,q2,q3)
            [ s1 * reach_s,       -c1 * reach_c,       -l2 * c1 * c23],   # d(y)/d(q1,q2,q3)
            [ 0.0,                -reach_s,             -l2 * s23     ],   # d(z)/d(q1,q2,q3)
        ])

        # Normalized manipulability measure
        # Denominator = average_link * l1 * l2 (normalization constant)
        # FIX NEW-C: Guard against zero link lengths
        denom = ((l1 + l2) / 2.0) * l1 * l2
        if abs(denom) < 1e-12:
            rospy.logwarn_throttle(5.0,
                "[redundancy] Zero manipulability denominator — link lengths may be zero")
            return -m_t  # Maximally violated → forces gait planning

        manipulability = abs(numpy.linalg.det(J)) / denom

        return manipulability - m_t

    def stroke_length_constraint_1(self, x):
        """
        Minimum stroke length: ||Δ_tb|| ≥ MIN_STROKE_LENGTH
        Prevents zero-displacement solutions (robot must actually move).
        """
        return (numpy.hypot(x[3] - self.x_i[3], x[4] - self.x_i[4]) -
                MIN_STROKE_LENGTH)

    def stroke_length_constraint_2(self, x):
        """
        Maximum stroke length: ||Δ_tb|| ≤ MAX_STROKE_LENGTH
        Prevents too-large steps (legs must be able to track body motion).
        """
        return (MAX_STROKE_LENGTH -
                numpy.hypot(x[3] - self.x_i[3], x[4] - self.x_i[4]))

    def resolve(self):
        """
        Run SLSQP optimization to find optimal configuration that
        restores manipulability while minimizing pose change.

        Returns:
            x = [q1, q2, q3, x_tb, y_tb] — optimized configuration
            Falls back to x_i if optimization fails.
        """
        # FIX NEW-B: Copy initial guess to prevent reference mutation
        x_0 = list(self.x_i)

        # Build constraint list
        constraints = [
            {'type': 'eq',   'fun': self.ee_x_constraint},              # EE reaches target X
            {'type': 'eq',   'fun': self.ee_y_constraint},              # EE reaches target Y
            {'type': 'eq',   'fun': self.ee_z_constraint},              # EE reaches target Z
            {'type': 'ineq', 'fun': self.ee_manipulability_constraint}, # Manipulability ≥ threshold
            {'type': 'ineq', 'fun': self.stroke_length_constraint_1},   # Stroke ≥ minimum
            {'type': 'ineq', 'fun': self.stroke_length_constraint_2},   # Stroke ≤ maximum
        ]

        # Run optimizer
        solution = minimize(
            self.objective,
            x_0,
            method='SLSQP',
            bounds=self.bounds,
            constraints=constraints
        )

        # FIX NEW-A: Check convergence
        if not solution.success:
            rospy.logwarn_throttle(3.0,
                "[redundancy] SLSQP did not converge: '{}' — using initial config as fallback".format(
                    solution.message))

            # Validate that the failed solution at least satisfies EE constraints
            # If not, fall back completely to initial config
            ee_err = (abs(self.ee_x_constraint(solution.x)) +
                      abs(self.ee_y_constraint(solution.x)) +
                      abs(self.ee_z_constraint(solution.x)))

            if ee_err > 1e-3:  # 1mm total EE error tolerance
                rospy.logwarn_throttle(3.0,
                    "[redundancy] Failed solution violates EE constraints (err={:.4f}m) — "
                    "returning initial config".format(ee_err))
                return numpy.array(self.x_i)

            rospy.loginfo_throttle(3.0,
                "[redundancy] Failed solution satisfies EE constraints — using partial result")

        x = solution.x
        return x
