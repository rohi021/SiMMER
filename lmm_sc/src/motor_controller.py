#!/usr/bin/env python3

# ROS node for controlling servo motors using I2CPWM board
#
# ARCHITECTURE NOTE:
#   This node converts joint angles → PWM values and publishes to /servos_absolute.
#   The i2cpwm_board node handles multi-board routing:
#     servo 1-16  → Board 1 (0x40)
#     servo 17-32 → Board 2 (0x41)
#   We do NOT need to do channel remapping (% 16) here — i2cpwm_board does it.
#
# CONFIG NOTE:
#   The 'board_id' field in joints.yaml is metadata only — this node does not use it.
#   Routing is determined solely by the 'servo' number passed to i2cpwm_board.

# Import required libraries
import rospy
import rospkg
import yaml
import time
from sensor_msgs.msg import JointState
from i2cpwm_board.msg import Servo, ServoArray

# ──────────────────────────────────────────────
# LOAD CONFIG
# ──────────────────────────────────────────────
rospack = rospkg.RosPack()
package_path = rospack.get_path('lmm_sc')

# FIX #1: file() does not exist in Python 3 — use open()
# FIX #2: yaml.load() without Loader is unsafe — use yaml.safe_load()
with open(package_path + '/config/joints.yaml', 'r') as joints_config_file:
    lmm_joints = yaml.safe_load(joints_config_file)['lmm_joints']

# ──────────────────────────────────────────────
# INIT NODE
# ──────────────────────────────────────────────
rospy.init_node('motor_controller')

# Publisher
pub = rospy.Publisher('servos_absolute', ServoArray, queue_size=10)

# FIX #14: Rate protection — limit I2C publish rate
MIN_PUBLISH_INTERVAL = 0.09  # ~10 Hz max (matches master.py f_in = 10)
last_publish_time = 0.0

# ──────────────────────────────────────────────
# CALLBACK — JOINT ANGLES → PWM
# ──────────────────────────────────────────────
def callback(joint_data):
    global last_publish_time

    # Rate guard — skip if called too fast (protects I2C bus)
    now = time.time()
    if (now - last_publish_time) < MIN_PUBLISH_INTERVAL:
        return
    last_publish_time = now

    servos = []

    for joint in range(len(joint_data.name)):
        joint_name = joint_data.name[joint]

        # Safety: skip joints not in config
        if joint_name not in lmm_joints:
            rospy.logwarn_throttle(5.0, "[motor_controller] Joint '{}' not in config — skipping".format(joint_name))
            continue

        jcfg = lmm_joints[joint_name]

        # Extract config values
        min_angle  = float(jcfg['min_angle'])
        max_angle  = float(jcfg['max_angle'])
        min_input  = float(jcfg['min_input'])
        max_input  = float(jcfg['max_input'])
        axis       = float(jcfg['axis'])
        servo_num  = int(jcfg['servo'])

        # Compute midpoints and ranges
        mean_angle = (max_angle + min_angle) / 2.0
        angle_range = max_angle - min_angle
        mean_input = (max_input + min_input) / 2.0
        input_range = max_input - min_input

        # Prevent division by zero (misconfigured joint)
        if abs(angle_range) < 1e-10:
            rospy.logerr_throttle(5.0, "[motor_controller] Joint '{}' has zero angle range — skipping".format(joint_name))
            continue

        # Convert angle → PWM
        servo_value = mean_input + axis * (input_range / angle_range) * (joint_data.position[joint] - mean_angle)

        # FIX #3: CLAMP to safe PWM range — prevents mechanical damage
        clamped = max(min_input, min(max_input, servo_value))

        if abs(servo_value - clamped) > 1.0:
            rospy.logwarn_throttle(2.0,
                "[motor_controller] CLAMPED '{}': raw={:.1f} → safe={:.1f} (limits: {:.0f}-{:.0f})".format(
                    joint_name, servo_value, clamped, min_input, max_input))

        # Build servo message (int PWM value)
        servo_msg = Servo()
        servo_msg.servo = servo_num
        servo_msg.value = int(round(clamped))
        servos.append(servo_msg)

    # Publish all servos at once
    if servos:
        msg = ServoArray()
        msg.servos = servos
        pub.publish(msg)


# ──────────────────────────────────────────────
# FIX #18: SAFE SHUTDOWN — park servos at neutral
# ──────────────────────────────────────────────
def shutdown_handler():
    rospy.loginfo("[motor_controller] Shutting down — parking all servos at neutral position")
    servos = []
    for joint_name, jcfg in lmm_joints.items():
        servo_msg = Servo()
        servo_msg.servo = int(jcfg['servo'])
        servo_msg.value = int(round((float(jcfg['max_input']) + float(jcfg['min_input'])) / 2.0))
        servos.append(servo_msg)
    msg = ServoArray()
    msg.servos = servos
    pub.publish(msg)
    rospy.sleep(0.5)  # Allow message to transmit before node dies

rospy.on_shutdown(shutdown_handler)


# ──────────────────────────────────────────────
# SUBSCRIBER + SPIN
# ──────────────────────────────────────────────
rospy.Subscriber('lmm_joint_states', JointState, callback)
rospy.loginfo("[motor_controller] Ready — listening on /lmm_joint_states")
rospy.spin()
