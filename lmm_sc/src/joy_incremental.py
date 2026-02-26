#!/usr/bin/env python

# ROS node for converting joystick inputs to position increments
#
# Joystick Input:
#   This node subscribes to the /joy topic published by the ROS joy_node
#   (http://wiki.ros.org/joy). The joy_node reads from a Linux joystick
#   device (default /dev/input/js0) using the standard Linux joystick API.
#
#   Supported connection types (all are handled transparently by the OS):
#     - Wired USB controllers (e.g. Xbox/PS controller via USB cable)
#     - Wireless USB dongle controllers (e.g. Logitech with USB receiver)
#     - Bluetooth controllers (paired via the OS Bluetooth stack)
#
#   The connection type (wired/wireless) does not affect this code — the
#   Linux kernel presents all joystick devices uniformly as /dev/input/js*.
#   To change the device, set the 'dev' parameter on the joy_node in the
#   launch file (e.g. /dev/input/js1 for a second controller).
#
#   Axis mapping (assumes XInput-compatible controller layout):
#     axes[1] — Left stick Y   → controls Z increments (forward/back)
#     axes[3] — Right stick X  → controls X increments (left/right)
#     axes[4] — Right stick Y  → controls Y increments (up/down)
#
#   Compatible controllers include any XInput gamepad, for example:
#     - Xbox 360 / Xbox One / Xbox Series controllers
#     - EvoFox Elite X2 (2.4GHz USB dongle or wired USB-C)
#     - Logitech F710 / F310
#     - Sony DualShock 4 / DualSense (via ds4drv or Linux hid driver)
#   On Linux, XInput controllers typically require the 'xpad' kernel module.

# Import required libraries
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

# Initialize the node
rospy.init_node('joy_incremental')

# Set the parameters
multipliers = [-5.0, 5.0, 5.0]
divider = 4.0
f_in = 10		# Publishing frequency in Hertz

# Initialize the input vectors
joy_input = [0, 0, 0]
lmm_input = [0, 0, 0]

# Define the publisher
pub = rospy.Publisher('lmm_incremental_inputs', Float32MultiArray, queue_size = 10)
rate = rospy.Rate(f_in)

# Define callback to update the input vector everytime a msg is received
def callback(data):
	global joy_input
	joy_input[0] = data.axes[3]
	joy_input[1] = data.axes[4]
	joy_input[2] = data.axes[1]

# Define the subscriber
rospy.Subscriber('joy', Joy, callback)

# Publisher
while not rospy.is_shutdown():
	lmm_input[0] = (1/divider)*round(multipliers[0]*joy_input[0])*10**-3
	lmm_input[1] = (1/divider)*round(multipliers[1]*joy_input[1])*10**-3
	lmm_input[2] = (1/divider)*round(multipliers[2]*joy_input[2])*10**-3
	lmm_input_data = Float32MultiArray()
	lmm_input_data.data = lmm_input
	pub.publish(lmm_input_data)
	rate.sleep()
