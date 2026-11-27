"""Publish recorded body-frame relationships with transient-local durability."""
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster
import numpy as np


def publish_rig(node, edges):
    broadcaster = StaticTransformBroadcaster(node)
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for edge in edges:
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = edge['parent']
        tf.child_frame_id = edge['child']
        m = np.asarray(edge['matrix'])
        t, q = tf.transform.translation, tf.transform.rotation
        t.x, t.y, t.z = m[:3, 3].tolist()
        q.x, q.y, q.z, q.w = Rotation.from_matrix(m[:3, :3]).as_quat().tolist()
        transforms.append(tf)
    broadcaster.sendTransform(transforms)
    return broadcaster
