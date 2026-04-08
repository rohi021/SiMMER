#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# ROS node: lmm_sc_master
# Real-time motion planning for SiMMER hexapod + manipulator
#
# SUBSCRIBES: /lmm_incremental_inputs (Float32MultiArray)
#   data[0] → X increment (m)
#   data[1] → Y increment (m)
#   data[2] → Z increment (m)
#
# PUBLISHES: /lmm_joint_states (JointState)
#   21 joint angles for the full LMM (6 legs × 3 + manipulator × 3)
#
# PIPELINE:
#   joystick → joy_incremental → THIS NODE → motor_controller → servos
#
# STATE MACHINE:
#   1. TB in motion     → follow pre-planned trajectory
#   2. Inputs nonzero   → check manipulability → solve IK or plan gait
#   3. Inputs zero      → hold position (no computation)
# ══════════════════════════════════════════════════════════════

# ──────────────────────────────────────────────
# IMPORTS
# ──────────────────────────────────────────────
import rospy
import rospkg
import tf2_ros
import numpy
import time
import csv
import os
import threading
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Add modules path
import sys
rospack = rospkg.RosPack()
package_path = rospack.get_path('lmm_sc')
sys.path.insert(0, package_path + '/src/modules')

# Import modules
import inputs
import tb_time_calculation
import tb_trajectory
import leg_tip_traj_local
import leg_tip_traj_local_to_global
import redundancy_resolution
import inverse_kinematics
import initialize_lmm

# ──────────────────────────────────────────────
# INIT NODE
# ──────────────────────────────────────────────
rospy.init_node('lmm_sc_master')

# ──────────────────────────────────────────────
# CONSTANTS
# ──────────────────────────────────────────────

# Gait pattern — quadruped (support legs per stroke)
# FIX #23: Moved from inside callback to named constant
GAIT_SUPPORT_LEGS = [
    [2, 3, 4, 5],   # Stroke 1: legs 2,3,4,5 support, legs 1,6 swing
    [1, 3, 4, 6],   # Stroke 2: legs 1,3,4,6 support, legs 2,5 swing
    [1, 2, 5, 6],   # Stroke 3: legs 1,2,5,6 support, legs 3,4 swing
]
N_STROKES = len(GAIT_SUPPORT_LEGS)

# EE drift safety threshold (meters from init position)
# If EE drifts beyond this, something is wrong — warn operator
EE_DRIFT_WARN_THRESHOLD = 0.3  # 300mm — generous but catches runaway

# ──────────────────────────────────────────────
# TF SETUP — WITH ROBUST WAIT
# ──────────────────────────────────────────────
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# FIX #13: Replace hardcoded sleep(1) with active TF wait
# We need Trunk_Body → Leg tips to exist before proceeding
REQUIRED_FRAMES = [
    ('Trunk_Body', 'Leg_1_Tip'),
    ('Trunk_Body', 'Leg_2_Tip'),
    ('Trunk_Body', 'Leg_3_Tip'),
    ('Trunk_Body', 'Leg_4_Tip'),
    ('Trunk_Body', 'Leg_5_Tip'),
    ('Trunk_Body', 'Leg_6_Tip'),
]
TF_TIMEOUT = 10.0  # seconds — generous for slow Docker startup

rospy.loginfo("[master] Waiting for TF frames (timeout: {:.0f}s)...".format(TF_TIMEOUT))
for parent, child in REQUIRED_FRAMES:
    if not tfBuffer.can_transform(parent, child, rospy.Time(0), rospy.Duration(TF_TIMEOUT)):
        rospy.logfatal("[master] TF frame '{}' → '{}' not available after {:.0f}s! "
                       "Check: robot_state_publisher, tip_tf_broadcaster, URDF".format(
                           parent, child, TF_TIMEOUT))
        rospy.signal_shutdown("Missing TF frames")
        sys.exit(1)
    rospy.loginfo("[master]   ✓ {} → {}".format(parent, child))

rospy.loginfo("[master] All TF frames available")

# ──────────────────────────────────────────────
# GLOBAL STATE — WITH THREAD LOCK
# ──────────────────────────────────────────────

# FIX #16: Thread lock for all shared state
state_lock = threading.Lock()

# FIX NEW-C: .copy() to avoid mutating inputs module arrays
is_tb_in_motion = False
traj_r_G_tb = []
traj_r_G_leg_tip = {1: [], 2: [], 3: [], 4: [], 5: [], 6: []}
r_G_tb = inputs.r_G_tb_init.copy()
r_G_ee = inputs.r_G_ee_init.copy()
r_G_leg_tip = {}
lmm_joint_angles = {}
stroke_length = 0.0
time_instance = 0

# ──────────────────────────────────────────────
# CSV DATA RECORDING — OPEN ONCE
# ──────────────────────────────────────────────

# FIX #17: Open file once, not every callback
data_folder = package_path + '/data/'
if not os.path.exists(data_folder):
    os.makedirs(data_folder)
data_file_name = time.strftime('%Y_%m_%d__%H_%M_%S') + '.csv'
data_file_path = data_folder + data_file_name

csv_file_handle = open(data_file_path, 'w', newline='')
csv_writer = csv.writer(csv_file_handle)

# FIX #29: Write header row so columns are identifiable
csv_writer.writerow([
    'time_s',
    'ee_inc_x', 'ee_inc_y', 'ee_inc_z',
    'ee_x', 'ee_y', 'ee_z',
    'tb_x', 'tb_y', 'tb_z',
    'manipulability', 'stroke_length',
    'joint_1_1', 'joint_1_2', 'joint_1_3',
    'joint_2_1', 'joint_2_2', 'joint_2_3',
])
csv_file_handle.flush()

rospy.loginfo("[master] CSV recording to: {}".format(data_file_name))

# ──────────────────────────────────────────────
# PUBLISHER
# ──────────────────────────────────────────────
pub = rospy.Publisher('lmm_joint_states', JointState, queue_size=10)

# ──────────────────────────────────────────────
# INITIALIZE LMM (first IK solve for standing pose)
# ──────────────────────────────────────────────
rospy.loginfo("[master] Computing initial standing pose...")

lmm_joint_angles, r_G_leg_tip = initialize_lmm.lmm_joint_angles_init(
    inputs.r_G_tb_init, inputs.r_G_ee_init, inputs.x_si_li_init,
    inputs.eta_G_L0, inputs.r_l0_si_p0, inputs.li, inputs.di,
    inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man, inputs.phi,
    inputs.phi_m, inputs.gamma_r, inputs.gamma_l
)

# Publish initial standing pose
lmm_joint_states = JointState()
lmm_joint_states.header.stamp = rospy.Time.now()
for joint, angle in lmm_joint_angles.items():
    lmm_joint_states.name.append(joint)
    lmm_joint_states.position.append(angle)
pub.publish(lmm_joint_states)

rospy.loginfo("[master] Initial pose published ({} joints)".format(len(lmm_joint_angles)))

# ──────────────────────────────────────────────
# HELPER: Publish joint states
# ──────────────────────────────────────────────
def publish_joint_states(joint_angles):
    """Build and publish JointState message from angle dict."""
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    for joint, angle in joint_angles.items():
        msg.name.append(joint)
        msg.position.append(angle)
    pub.publish(msg)

# ──────────────────────────────────────────────
# CALLBACK — MAIN CONTROL LOOP
# ──────────────────────────────────────────────
def callback(lmm_inputs):
    start_time = time.time()

    global is_tb_in_motion
    global traj_r_G_tb
    global traj_r_G_leg_tip
    global r_G_tb
    global r_G_ee
    global r_G_leg_tip
    global lmm_joint_angles
    global stroke_length
    global time_instance

    ee_increments = numpy.array(lmm_inputs.data)
    time_instance += 1

    # FIX #16: Lock shared state during entire computation
    with state_lock:

        # ── STATE CHECK ──────────────────────────────

        if not traj_r_G_tb:
            is_tb_in_motion = False
        else:
            is_tb_in_motion = True

        # FIX #20: Use numpy.allclose instead of exact float == 0
        is_inputs_zero = numpy.allclose(ee_increments, 0, atol=1e-6)

        # ── BRANCH 1: TB IN MOTION — FOLLOW TRAJECTORY ──

        if is_tb_in_motion:

            # FIX NEW-D: Log state transition
            if len(traj_r_G_tb) == len(traj_r_G_leg_tip[1]):
                remaining = len(traj_r_G_tb)
                if remaining % 10 == 0:
                    rospy.loginfo_throttle(2.0, "[master] TB trajectory: {}/{} waypoints remaining".format(
                        remaining, remaining))

            r_G_tb = traj_r_G_tb.pop(0)
            r_G_ee = r_G_ee + ee_increments
            r_G_leg_tip = {}
            for i in range(6):
                leg = i + 1
                r_G_leg_tip[leg] = traj_r_G_leg_tip[leg].pop(0)

            ik_lmm = inverse_kinematics.ik_lmm_solver(
                r_G_tb, inputs.eta_G_L0, r_G_ee, r_G_leg_tip,
                inputs.r_l0_si_p0, inputs.li, inputs.di,
                inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man,
                inputs.phi, inputs.phi_m, inputs.gamma_r, inputs.gamma_l
            )
            lmm_joint_angles = ik_lmm.solve_lmm()
            publish_joint_states(lmm_joint_angles)

            # FIX #37: stroke_length stays from last gait plan (correct here)

        # ── BRANCH 2: INPUTS NONZERO — MANIPULABILITY CHECK ──

        elif not is_inputs_zero:

            r_G_ee = r_G_ee + ee_increments

            # FIX NEW-A: Drift safety check
            ee_drift = numpy.linalg.norm(r_G_ee - inputs.r_G_ee_init)
            if ee_drift > EE_DRIFT_WARN_THRESHOLD:
                rospy.logwarn_throttle(3.0,
                    "[master] EE drift {:.3f}m exceeds threshold {:.3f}m — check joystick!".format(
                        ee_drift, EE_DRIFT_WARN_THRESHOLD))

            x_i = [
                lmm_joint_angles['joint_m_1'],
                lmm_joint_angles['joint_m_2'],
                lmm_joint_angles['joint_m_3'],
                r_G_tb[0], r_G_tb[1]
            ]

            red_resolver = redundancy_resolution.redundancy_resolver(
                inputs.w, inputs.l_man, inputs.r_l0_mb_p0, inputs.m_t,
                r_G_tb, r_G_ee, x_i, inputs.bounds
            )

            ik_lmm = inverse_kinematics.ik_lmm_solver(
                r_G_tb, inputs.eta_G_L0, r_G_ee, r_G_leg_tip,
                inputs.r_l0_si_p0, inputs.li, inputs.di,
                inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man,
                inputs.phi, inputs.phi_m, inputs.gamma_r, inputs.gamma_l
            )
            man_joint_angles = ik_lmm.solve_manipulator()

            x_f = [
                man_joint_angles['joint_m_1'],
                man_joint_angles['joint_m_2'],
                man_joint_angles['joint_m_3'],
                r_G_tb[0], r_G_tb[1]
            ]

            if red_resolver.ee_manipulability_constraint(x_f, inputs.m_t_man) >= 0:
                # Manipulability OK — just solve IK
                lmm_joint_angles = ik_lmm.solve_lmm()
                publish_joint_states(lmm_joint_angles)

                # FIX #37: No active trajectory
                stroke_length = 0.0

            else:
                # Manipulability LOW — trigger redundancy resolution + gait planning
                rospy.loginfo("[master] Manipulability below threshold — planning gait")

                x_f = red_resolver.resolve()

                # TB positions for crab angle
                r_G_tb_i = r_G_tb.copy()
                r_G_tb_f = numpy.array([x_f[3], x_f[4], r_G_tb[2]])

                # Crab angle
                theta_c = numpy.arctan2(
                    (r_G_tb_f[1] - r_G_tb_i[1]),
                    (r_G_tb_f[0] - r_G_tb_i[0])
                )

                # TB time calculation
                tb_time = tb_time_calculation.tb_time_calculation(
                    r_G_tb_i, r_G_tb_f, theta_c,
                    inputs.vel_tb_max, inputs.T_stroke, inputs.T_in
                )

                # Gait planning
                tb_pos_i = r_G_tb.copy()
                leg_tip_trajectory_local = {1: [], 2: [], 3: [], 4: [], 5: [], 6: []}
                leg_tip_pos_f_local = {
                    1: numpy.array([0, 0, 0]),
                    2: numpy.array([0, 0, 0]),
                    3: numpy.array([0, 0, 0]),
                    4: numpy.array([0, 0, 0]),
                    5: numpy.array([0, 0, 0]),
                    6: numpy.array([0, 0, 0]),
                }

                for i in range(N_STROKES):
                    traj_r_G_tb_temp = tb_trajectory.tb_trajectory(
                        theta_c, inputs.vel_tb_max, tb_pos_i, tb_time, inputs.T_in
                    )
                    traj_r_G_tb.extend(traj_r_G_tb_temp)
                    n_traj_temp = len(traj_r_G_tb_temp)
                    tb_pos_f = traj_r_G_tb_temp[n_traj_temp - 1]

                    stroke_length = numpy.hypot(
                        (tb_pos_f[0] - tb_pos_i[0]),
                        (tb_pos_f[1] - tb_pos_i[1])
                    )
                    swing_length = N_STROKES * stroke_length

                    leg_tip_tr_local_temp = leg_tip_traj_local.leg_tip_traj_local(
                        GAIT_SUPPORT_LEGS[i], swing_length, theta_c,
                        inputs.T_stroke, inputs.T_in, inputs.Hmi1,
                        inputs.del_h, inputs.hi3_dash, inputs.lt_time_ratio
                    )
                    leg_tip_trajectory_local_temp = leg_tip_tr_local_temp.calculate_trajectry()

                    for j in range(6):
                        leg = j + 1
                        for k in range(n_traj_temp):
                            leg_tip_trajectory_local_temp[leg][k] += leg_tip_pos_f_local[leg]
                        leg_tip_trajectory_local[leg].extend(leg_tip_trajectory_local_temp[leg])

                    tb_pos_i = tb_pos_f
                    n_traj = len(leg_tip_trajectory_local[1])
                    for p in range(6):
                        leg = p + 1
                        leg_tip_pos_f_local[leg] = leg_tip_trajectory_local[leg][n_traj - 1]

                traj_r_G_leg_tip = leg_tip_traj_local_to_global.traj_local_to_global(
                    r_G_tb, leg_tip_trajectory_local, tfBuffer
                )

                rospy.loginfo("[master] Gait planned: {} total waypoints, stroke={:.4f}m".format(
                    len(traj_r_G_tb), stroke_length))

                # Pop first waypoint
                r_G_tb = traj_r_G_tb.pop(0)
                r_G_leg_tip = {}
                for i in range(6):
                    leg = i + 1
                    r_G_leg_tip[leg] = traj_r_G_leg_tip[leg].pop(0)

                ik_lmm = inverse_kinematics.ik_lmm_solver(
                    r_G_tb, inputs.eta_G_L0, r_G_ee, r_G_leg_tip,
                    inputs.r_l0_si_p0, inputs.li, inputs.di,
                    inputs.r_l0_mb_p0, inputs.l_man, inputs.d_man,
                    inputs.phi, inputs.phi_m, inputs.gamma_r, inputs.gamma_l
                )
                lmm_joint_angles = ik_lmm.solve_lmm()
                publish_joint_states(lmm_joint_angles)

        # ── BRANCH 3: INPUTS ZERO — HOLD POSITION ──

        else:
            # FIX #37: Explicitly zero when idle
            stroke_length = 0.0

    # ── END OF LOCK ──────────────────────────────

    # ── TIMING CHECK ─────────────────────────────
    end_time = time.time()
    loop_time = end_time - start_time
    if loop_time > inputs.T_in:
        rospy.logwarn("[master] Loop overrun: {:.4f}s (limit: {:.4f}s)".format(
            loop_time, inputs.T_in))

    # ── CSV RECORDING ────────────────────────────
    # FIX #17: Write to pre-opened file handle (no open/close per loop)
    try:
        x_man = [
            lmm_joint_angles['joint_m_1'],
            lmm_joint_angles['joint_m_2'],
            lmm_joint_angles['joint_m_3'],
            r_G_tb[0], r_G_tb[1]
        ]
        red_res_man = redundancy_resolution.redundancy_resolver(
            inputs.w, inputs.l_man, inputs.r_l0_mb_p0, inputs.m_t,
            r_G_tb, r_G_ee, x_man, inputs.bounds
        )
        manipulability = red_res_man.ee_manipulability_constraint(x_man, inputs.m_t_man) + inputs.m_t_man

        row = [
            time_instance * inputs.T_in,
            ee_increments[0], ee_increments[1], ee_increments[2],
            r_G_ee[0], r_G_ee[1], r_G_ee[2],
            r_G_tb[0], r_G_tb[1], r_G_tb[2],
            manipulability, stroke_length,
            lmm_joint_angles["joint_1_1"], lmm_joint_angles["joint_1_2"], lmm_joint_angles["joint_1_3"],
            lmm_joint_angles["joint_2_1"], lmm_joint_angles["joint_2_2"], lmm_joint_angles["joint_2_3"],
        ]
        csv_writer.writerow(row)
        csv_file_handle.flush()
    except (KeyError, ValueError) as e:
        rospy.logwarn_throttle(5.0, "[master] CSV write error: {}".format(e))


# ──────────────────────────────────────────────
# SHUTDOWN HANDLER
# ──────────────────────────────────────────────

# FIX NEW-B: Graceful shutdown — flush and close CSV
def shutdown_handler():
    rospy.loginfo("[master] Shutting down...")
    rospy.loginfo("[master] Final EE position: [{:.4f}, {:.4f}, {:.4f}]".format(
        r_G_ee[0], r_G_ee[1], r_G_ee[2]))
    rospy.loginfo("[master] Final TB position: [{:.4f}, {:.4f}, {:.4f}]".format(
        r_G_tb[0], r_G_tb[1], r_G_tb[2]))
    try:
        csv_file_handle.flush()
        csv_file_handle.close()
        rospy.loginfo("[master] CSV saved: {}".format(data_file_name))
    except Exception as e:
        rospy.logwarn("[master] CSV close error: {}".format(e))

rospy.on_shutdown(shutdown_handler)


# ──────────────────────────────────────────────
# FIX #32: Create subscriber BEFORE ready message
# ──────────────────────────────────────────────
rospy.Subscriber('lmm_incremental_inputs', Float32MultiArray, callback)

# Ready
rospy.loginfo("[master] ✅ Im Ready — listening on /lmm_incremental_inputs")
rospy.loginfo("[master] Initial TB: [{:.4f}, {:.4f}, {:.4f}]".format(
    r_G_tb[0], r_G_tb[1], r_G_tb[2]))
rospy.loginfo("[master] Initial EE: [{:.4f}, {:.4f}, {:.4f}]".format(
    r_G_ee[0], r_G_ee[1], r_G_ee[2]))

# Keep spinning
rospy.spin()
