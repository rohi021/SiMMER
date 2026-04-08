#!/usr/bin/env python3

# ══════════════════════════════════════════════════════════════
# TF Broadcaster: Leg Tips + End-Effector Tip
#
# PURPOSE:
#   Publishes static transforms from the last link of each leg
#   (and the gripper) to the physical foot tip (and EE tip).
#   These frames are NOT in the URDF — they represent the
#   final segment that connects the last joint to ground contact.
#
# FRAMES PUBLISHED:
#   Leg_1_Link_3 → Leg_1_Tip   (-0.1m in X — left leg shin)
#   Leg_2_Link_3 → Leg_2_Tip   (+0.1m in X — right leg shin)
#   Leg_3_Link_3 → Leg_3_Tip   (-0.1m in X — left leg shin)
#   Leg_4_Link_3 → Leg_4_Tip   (+0.1m in X — right leg shin)
#   Leg_5_Link_3 → Leg_5_Tip   (-0.1m in X — left leg shin)
#   Leg_6_Link_3 → Leg_6_Tip   (+0.1m in X — right leg shin)
#   Gripper      → EE_Tip      (+0.09m in Z — gripper length)
#
# OFFSET ORIGINS:
#   Leg tips: ±0.1m = li[2] from inputs.py (shin link length)
#     Left legs (1,3,5): -X (tip extends outward from body)
#     Right legs (2,4,6): +X (tip extends outward from body)
#     Sign depends on URDF Link_3 frame orientation.
#
#   EE tip: 0.09m = gripper physical length (NOT from l_man)
#     Extends in +Z from Gripper frame.
#
# ⚠️  If leg link lengths change in inputs.py (li[2]),
#     update LEG_TIP_OFFSET below to match!
#
# ⚠️  If URDF link frame names change, update PARENT frames below!
#
# CONSUMED BY:
#   leg_tip_traj_local_to_global.py (TF lookups)
#   master.py (TF wait at startup)
#
# BROADCAST TYPE:
#   Static — these transforms never change at runtime.
#   Published once via StaticTransformBroadcaster.
# ══════════════════════════════════════════════════════════════

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

# ──────────────────────────────────────────────
# INIT NODE
# ──────────────────────────────────────────────
rospy.init_node('tip_tf_broadcaster')

# ──────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────

# Leg tip offset from Link_3 frame (meters)
# This equals the shin link length: li[2] = 0.100m in inputs.py
# Left legs: -X direction (outward from body)
# Right legs: +X direction (outward from body)
LEG_TIP_OFFSET = 0.1    # meters — MUST match inputs.py li[2]

# EE tip offset from Gripper frame (meters)
# This is the gripper physical length (separate from manipulator links)
EE_TIP_OFFSET = 0.09    # meters

# Transform definitions: (parent_frame, child_frame, [x, y, z])
# Rotation is identity [0, 0, 0, 1] for all (no rotation at tips)
TIP_TRANSFORMS = [
    ('Leg_1_Link_3', 'Leg_1_Tip', [-LEG_TIP_OFFSET, 0.0, 0.0]),   # Left-Front
    ('Leg_2_Link_3', 'Leg_2_Tip', [ LEG_TIP_OFFSET, 0.0, 0.0]),   # Right-Front
    ('Leg_3_Link_3', 'Leg_3_Tip', [-LEG_TIP_OFFSET, 0.0, 0.0]),   # Left-Middle
    ('Leg_4_Link_3', 'Leg_4_Tip', [ LEG_TIP_OFFSET, 0.0, 0.0]),   # Right-Middle
    ('Leg_5_Link_3', 'Leg_5_Tip', [-LEG_TIP_OFFSET, 0.0, 0.0]),   # Left-Rear
    ('Leg_6_Link_3', 'Leg_6_Tip', [ LEG_TIP_OFFSET, 0.0, 0.0]),   # Right-Rear
    ('Gripper',      'EE_Tip',    [0.0, 0.0, EE_TIP_OFFSET]),      # End-Effector
]

# ──────────────────────────────────────────────
# BUILD TRANSFORM MESSAGES
# ──────────────────────────────────────────────

def build_transform(parent, child, translation):
    """
    Create a TransformStamped message with identity rotation.

    Args:
        parent:      Parent frame name (must exist in URDF TF tree)
        child:       Child frame name (created by this node)
        translation: [x, y, z] offset in meters

    Returns:
        TransformStamped message
    """
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0
    return t

# ──────────────────────────────────────────────
# WAIT FOR PARENT FRAMES
# ──────────────────────────────────────────────
# FIX NEW-E: Verify parent frames exist before publishing.
# If a parent frame is missing (URDF not loaded, typo), the
# child frame floats disconnected from the TF tree.

rospy.loginfo("[tip_tf] Waiting for parent frames from URDF...")

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

FRAME_TIMEOUT = 10.0  # seconds

# We need to check that at least one parent exists
# (they all come from the same URDF, so if one works, all should)
test_parent = TIP_TRANSFORMS[0][0]  # 'Leg_1_Link_3'
if not tfBuffer.can_transform('Trunk_Body', test_parent, rospy.Time(0), rospy.Duration(FRAME_TIMEOUT)):
    rospy.logwarn("[tip_tf] Parent frame '{}' not available after {:.0f}s — "
                  "publishing anyway (frames may be disconnected)".format(
                      test_parent, FRAME_TIMEOUT))
else:
    rospy.loginfo("[tip_tf]   ✓ Parent frame '{}' found".format(test_parent))

# ──────────────────────────────────────────────
# PUBLISH STATIC TRANSFORMS
# ──────────────────────────────────────────────
# FIX NEW-A, NEW-B: Use StaticTransformBroadcaster
# These offsets never change → publish ONCE, TF2 caches forever.
# Eliminates 10Hz publishing overhead.

static_broadcaster = tf2_ros.StaticTransformBroadcaster()

transforms = []
for parent, child, trans in TIP_TRANSFORMS:
    tf_msg = build_transform(parent, child, trans)
    transforms.append(tf_msg)
    rospy.loginfo("[tip_tf]   Publishing: {} → {} [{:.3f}, {:.3f}, {:.3f}]".format(
        parent, child, trans[0], trans[1], trans[2]))

static_broadcaster.sendTransform(transforms)

rospy.loginfo("[tip_tf] ✅ {} static transforms published".format(len(transforms)))
rospy.loginfo("[tip_tf] Leg tip offset: {:.3f}m (should match inputs.py li[2])".format(LEG_TIP_OFFSET))
rospy.loginfo("[tip_tf] EE tip offset: {:.3f}m (gripper length)".format(EE_TIP_OFFSET))

# ──────────────────────────────────────────────
# KEEP NODE ALIVE
# ──────────────────────────────────────────────
# StaticTransformBroadcaster latches the message,
# but the node must stay alive for the ROS lifecycle.

rospy.spin()
