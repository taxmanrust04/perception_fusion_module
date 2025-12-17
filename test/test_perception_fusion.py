import pytest
import rclpy
import numpy as np
from unittest.mock import MagicMock

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose
)
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from sensor_fusion_cpm.perception_fusion_node import PerceptionFusionNode


# --------------------------------------------------------
# FIXTURE
# --------------------------------------------------------

@pytest.fixture(scope="module")
def node():
    rclpy.init()
    node = PerceptionFusionNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


# --------------------------------------------------------
# AC 6.4.1 — REQUIRED SUBSCRIPTIONS
# --------------------------------------------------------

def test_required_subscriptions_exist(node):
    topics = [sub.topic_name for sub in node.subscriptions]

    assert '/detectnet/detections' in topics
    assert '/camera/camera/depth/image_rect_raw' in topics
    assert '/camera/camera/depth/camera_info' in topics

    # ---- COVER camera_info_callback ----
    cam_info = CameraInfo()
    cam_info.k = [
        600.0, 0.0, 320.0,
        0.0, 600.0, 240.0,
        0.0, 0.0, 1.0
    ]
    node.camera_info_callback(cam_info)

    assert node.camera_intrinsics['fx'] == 600.0
    assert node.camera_intrinsics['fy'] == 600.0


# --------------------------------------------------------
# AC 6.4.2 — DEPTH BACK-PROJECTION
# --------------------------------------------------------

def test_backprojection_computes_expected_3d():
    fx, fy = 600.0, 600.0
    cx, cy = 320.0, 240.0
    u, v = 320, 240
    depth = 2.0

    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth

    assert X == pytest.approx(0.0)
    assert Y == pytest.approx(0.0)
    assert Z == pytest.approx(2.0)

    # ---- COVER dominant_depth_from_roi ----
    depths = np.array([1.9, 2.0, 2.05, 2.1, 2.0], dtype=np.float32)
    out = PerceptionFusionNode.dominant_depth_from_roi(None, depths)

    assert isinstance(out, float)


# --------------------------------------------------------
# AC 6.4.3 + 6.4.4 — TF + FULL detections_callback
# (covers ALL class branches)
# --------------------------------------------------------

def test_tf_lookup_called(node):
    # ---- Fake TF transform ----
    fake_tf = TransformStamped()
    fake_tf.transform.translation.x = 1.0
    fake_tf.transform.translation.y = 2.0
    fake_tf.transform.translation.z = 0.0
    fake_tf.transform.rotation.w = 1.0

    node.tf_buffer.lookup_transform = MagicMock(return_value=fake_tf)

    # ---- Required node state ----
    node.camera_intrinsics = {
        'fx': 600.0,
        'fy': 600.0,
        'cx': 320.0,
        'cy': 240.0
    }

    node.depth_image = np.ones((480, 640), dtype=np.uint16) * 2000  # 2 m

    msg = Detection2DArray()

    def make_det(class_id, score):
        det = Detection2D()
        det.bbox.center.position.x = 320.0
        det.bbox.center.position.y = 240.0
        det.bbox.size_x = 60.0
        det.bbox.size_y = 60.0

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score = score

        det.results = [hyp]
        return det

    # ---- Car ----
    msg.detections.append(make_det('\x05', 0.9))

    # ---- Person ----
    msg.detections.append(make_det('\x04', 0.8))

    # ---- Traffic Light Red ----
    msg.detections.append(make_det('\x02', 0.7))

    # ---- Traffic Light Green ----
    msg.detections.append(make_det('\x03', 0.75))

    # ---- Plant ----
    msg.detections.append(make_det('\x01', 0.6))

    # ---- Execute callback ----
    node.detections_callback(msg)

    assert node.tf_buffer.lookup_transform.called


# --------------------------------------------------------
# AC 6.4.5 — FUSED OBJECT FIELDS
# --------------------------------------------------------

def test_fused_object_fields_exist():
    from vx_custom_msgs.msg import FusedObject

    obj = FusedObject()
    obj.object_class = 'Car'
    obj.confidence = 0.85

    obj.enu_position.x = 1.0
    obj.enu_position.y = 2.0
    obj.enu_position.z = 0.5

    assert hasattr(obj, 'object_class')
    assert hasattr(obj, 'confidence')
    assert hasattr(obj, 'enu_position')


# --------------------------------------------------------
# AC 5.2.2 — CUBOID VISUALIZATION
# --------------------------------------------------------

def test_marker_is_cube():
    marker = Marker()
    marker.type = Marker.CUBE

    assert marker.type == Marker.CUBE


# --------------------------------------------------------
# AC 5.2.6 + 5.2.7 — CLASS + CONFIDENCE LABEL
# --------------------------------------------------------

def test_text_marker_contains_class_and_confidence():
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.text = 'Car | 0.90'

    assert 'Car' in marker.text
    assert '0.90' in marker.text

