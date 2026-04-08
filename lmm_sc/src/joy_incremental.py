#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# ROS node: joy_incremental
# Converts joystick analog stick inputs to position increments
# for the LMM end-effector (manipulator arm tip).
#
# PUBLISHES:  /lmm_incremental_inputs  (Float32MultiArray)
#   data[0] → X increment (meters) — from Right Stick X
#   data[1] → Y increment (meters) — from Right Stick Y
#   data[2] → Z increment (meters) — from Left Stick Y
#
# SUBSCRIBES: /joy  (sensor_msgs/Joy)
#
# JOYSTICK: EvoFox Elite X2 — VERIFIED AXIS MAP:
#   axes[0] → Left Stick X   (not used)
#   axes[1] → Left Stick Y   ✅ mapped to Z
#   axes[2] → Right Stick X  ✅ mapped to X
#   axes[3] → Right Stick Y  ✅ mapped to Y
#   axes[4] → ❌ DEAD — stuck at -32767 — NEVER USE
#   axes[5] → ❌ DEAD — stuck at -32767 — NEVER USE
#   axes[6] → D-Pad X
#   axes[7] → D-Pad Y
#
# INCREMENT MATH (per publish cycle at 10 Hz):
#   increment = (1/divider) * round(multiplier * stick_value) * 10^-3
#   At full deflection: (1/4) * round(5.0 * 1.0) * 0.001 = 0.00125 m
#   At 10 Hz → max speed = 0.0125 m/s = 12.5 mm/s
#   round() quantizes to ~1mm steps — intentional coarse quantization
# ══════════════════════════════════════════════════════════════

import rospy
import threading
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

# ──────────────────────────────────────────────
# INIT NODE
# ──────────────────────────────────────────────
rospy.init_node('joy_incremental')

# ──────────────────────────────────────────────
# PARAMETERS
# ──────────────────────────────────────────────
# Multipliers: scale stick deflection (±1.0) to increment magnitude
#   Negative = invert axis direction to match robot frame
#   [X, Y, Z] — units: dimensionless scaling factor
multipliers = [-5.0, 5.0, 5.0]

# Divider: additional scaling divisor for finer control
#   Higher = slower/more precise end-effector movement
divider = 4.0

# Publish rate (Hz) — matches master.py callback rate
f_in = 10

# Deadzone: ignore stick values below this threshold
#   Prevents drift when sticks are released (analog sticks never rest at exactly 0)
#   0.05 = 5% of full deflection — tuned for EvoFox Elite X2
DEADZONE = 0.05

# Axis indices — EvoFox Elite X2 verified mapping
AXIS_X = 2    # Right Stick X → end-effector X
AXIS_Y = 3    # Right Stick Y → end-effector Y
AXIS_Z = 1    # Left Stick Y  → end-effector Z

# Minimum required axes count from joystick
MIN_AXES_REQUIRED = 4  # Need at least axes[0] through axes[3]

# ──────────────────────────────────────────────
# STATE (thread-safe)
# ──────────────────────────────────────────────
# FIX NEW-C: Atomic swap instead of element-by-element mutation
joy_lock = threading.Lock()
joy_input = [0.0, 0.0, 0.0]
joy_received = False  # For startup confirmation log

# ──────────────────────────────────────────────
# DEADZONE FILTER
# ──────────────────────────────────────────────
def apply_deadzone(value, deadzone):
    """Return 0.0 if stick is within deadzone, else return original value."""
    if abs(value) < deadzone:
        return 0.0
    return value

# ──────────────────────────────────────────────
# CALLBACK — /joy → joy_input
# ──────────────────────────────────────────────
def callback(data):
    global joy_input, joy_received

    # FIX NEW-B: Bounds check — different controllers may have fewer axes
    if len(data.axes) < MIN_AXES_REQUIRED:
        rospy.logwarn_throttle(5.0,
            "[joy_incremental] Joystick has only {} axes, need {}. Check controller!".format(
                len(data.axes), MIN_AXES_REQUIRED))
        return

    # FIX #7: Read from VERIFIED working axes (NOT axes[4] which is DEAD)
    # FIX NEW-A: Apply deadzone to filter stick drift
    new_input = [
        apply_deadzone(data.axes[AXIS_X], DEADZONE),  # Right Stick X → X
        apply_deadzone(data.axes[AXIS_Y], DEADZONE),  # Right Stick Y → Y
        apply_deadzone(data.axes[AXIS_Z], DEADZONE),  # Left Stick Y  → Z
    ]

    # FIX NEW-C: Atomic swap — main loop always reads a consistent triplet
    with joy_lock:
        joy_input[:] = new_input

    # FIX NEW-D: Log first joystick message received (once)
    if not joy_received:
        joy_received = True
        rospy.loginfo("[joy_incremental] First joystick message received — axes count: {}".format(len(data.axes)))

# ──────────────────────────────────────────────
# SUBSCRIBER
# ──────────────────────────────────────────────
rospy.Subscriber('joy', Joy, callback)

# ──────────────────────────────────────────────
# PUBLISHER LOOP
# ──────────────────────────────────────────────
pub = rospy.Publisher('lmm_incremental_inputs', Float32MultiArray, queue_size=10)
rate = rospy.Rate(f_in)

rospy.loginfo("[joy_incremental] Ready — waiting for /joy messages")
rospy.loginfo("[joy_incremental] Axis map: X=axes[{}], Y=axes[{}], Z=axes[{}] | Deadzone={:.2f}".format(
    AXIS_X, AXIS_Y, AXIS_Z, DEADZONE))

while not rospy.is_shutdown():
    # FIX NEW-C: Read atomically
    with joy_lock:
        local_input = list(joy_input)

    # Compute increments (preserving original quantization math exactly)
    lmm_input = [
        (1.0 / divider) * round(multipliers[0] * local_input[0]) * 1e-3,
        (1.0 / divider) * round(multipliers[1] * local_input[1]) * 1e-3,
        (1.0 / divider) * round(multipliers[2] * local_input[2]) * 1e-3,
    ]

    # Publish
    msg = Float32MultiArray()
    msg.data = lmm_input
    pub.publish(msg)
    rate.sleep()
