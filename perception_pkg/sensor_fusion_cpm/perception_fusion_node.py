#!/usr/bin/env python3
"""
Perception Fusion Node

Fuses 2D object detections, depth camera data, and TF-based localization
to generate globally referenced 3D object information and RViz markers.
"""

# pylint: disable=too-many-instance-attributes

import time
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
from vx_custom_msgs.msg import FusedObject, FusedObjects

import tf2_ros
from tf2_geometry_msgs import do_transform_point


class PerceptionFusionNode(Node):
    """
    ROS 2 node that performs perception fusion by combining:
    - 2D object detections
    - Depth camera measurements
    - TF-based localization

    Publishes fused object data and RViz visualization markers.
    """

    def __init__(self):
        super().__init__('perception_fusion_node')

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.depth_image = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.label_map = {
            '1': 'Plant',
            '2': 'Traffic_light_red',
            '3': 'Traffic_light_green',
            '4': 'Person',
            '5': 'Car',
            '6': 'House'
        }

        self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detections_callback,
            10
        )

        self.pub_fused = self.create_publisher(
            FusedObjects, '/fused_objects_out', 10
        )
        self.pub_marker = self.create_publisher(
            MarkerArray, '/visualization_marker', 10
        )

    # -------------------------------------------------

    def camera_info_callback(self, msg):
        """Store camera intrinsic parameters."""
        k = msg.k
        self.camera_intrinsics = {
            'fx': float(k[0]),
            'fy': float(k[4]),
            'cx': float(k[2]),
            'cy': float(k[5]),
        }

    def depth_callback(self, msg):
        """Store the latest depth image."""
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    def dominant_depth_from_roi(self, depths):
        """Estimate dominant depth value from a depth ROI."""
        hist, bins = np.histogram(depths, bins=40, range=(0.1, 8.0))
        peak = np.argmax(hist)
        z_min, z_max = bins[peak], bins[peak + 1]
        vals = depths[(depths >= z_min) & (depths <= z_max)]
        return float(np.median(vals if vals.size > 10 else depths))

    # -------------------------------------------------

    # pylint: disable=too-many-locals, too-many-statements
    def detections_callback(self, msg):
        """
        Fuse detections with depth and TF transforms to produce
        3D object positions and RViz visualization markers.
        """

        if self.camera_intrinsics is None or self.depth_image is None:
            return

        fused = FusedObjects()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.station_id = 6
        fused.timestamp = float(time.time())

        marker_array = MarkerArray()
        marker_id = 0

        for det in msg.detections:

            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)
            w = int(det.bbox.size_x / 2)
            h = int(det.bbox.size_y / 2)

            roi = self.depth_image[
                max(0, v - h):min(v + h, self.depth_image.shape[0]),
                max(0, u - w):min(u + w, self.depth_image.shape[1])
            ].astype(np.float32) / 1000.0

            roi = roi[(roi > 0.1) & (roi < 8.0)]
            if roi.size < 30:
                continue

            depth = self.dominant_depth_from_roi(roi)

            fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
            cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']

            pt_cam = PointStamped()
            pt_cam.header.frame_id = 'camera_depth_optical_frame'
            pt_cam.point.x = (u - cx) * depth / fx
            pt_cam.point.y = (v - cy) * depth / fy
            pt_cam.point.z = depth

            try:
                tf_cam_to_base = self.tf_buffer.lookup_transform(
                    '6/base_link',
                    'camera_depth_optical_frame',
                    rclpy.time.Time()
                )
                pt_base = do_transform_point(pt_cam, tf_cam_to_base)

                tf_base_to_map = self.tf_buffer.lookup_transform(
                    'map',
                    '6/base_link',
                    rclpy.time.Time()
                )
                pt_map = do_transform_point(pt_base, tf_base_to_map)

            except Exception:  # pylint: disable=broad-except
                continue

            obj_class = 'unknown'
            conf = 0.0

            if det.results:
                res = det.results[0].hypothesis
                conf = float(res.score)
                raw_id = res.class_id
                if isinstance(raw_id, str) and len(raw_id) == 1:
                    raw_id = str(ord(raw_id))
                obj_class = self.label_map.get(raw_id, raw_id)

            obj = FusedObject()
            obj.object_class = obj_class
            obj.confidence = conf
            obj.base_link_position.x = pt_base.point.x
            obj.base_link_position.y = pt_base.point.y
            obj.base_link_position.z = pt_base.point.z
            obj.enu_position.x = pt_map.point.x
            obj.enu_position.y = pt_map.point.y
            obj.enu_position.z = pt_map.point.z
            fused.objects.append(obj)

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = fused.header.stamp
            marker.ns = obj_class
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = pt_map.point.x
            marker.pose.position.y = pt_map.point.y
            marker.color.a = max(0.3, min(1.0, conf))

            if obj_class == 'Car':
                marker.scale.x, marker.scale.y, marker.scale.z = 0.3, 0.8, 0.32
                marker.pose.position.z = 0.16
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            elif obj_class == 'Person':
                marker.scale.x, marker.scale.y, marker.scale.z = 0.15, 0.15, 0.38
                marker.pose.position.z = 0.19
                marker.color.r, marker.color.g, marker.color.b = 0.6, 0.35, 0.15
            elif obj_class == 'Traffic_light_red':
                marker.scale.x, marker.scale.y, marker.scale.z = 0.10, 0.10, 0.60
                marker.pose.position.z = 0.30
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            elif obj_class == 'Traffic_light_green':
                marker.scale.x, marker.scale.y, marker.scale.z = 0.10, 0.10, 0.60
                marker.pose.position.z = 0.30
                marker.color.r, marker.color.g, marker.color.b = 0.4, 1.0, 0.4
            elif obj_class == 'Plant':
                marker.scale.x, marker.scale.y, marker.scale.z = 0.28, 0.28, 0.45
                marker.pose.position.z = 0.225
                marker.color.r, marker.color.g, marker.color.b = 0.0, 0.45, 0.0
            else:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                marker.pose.position.z = 0.1
                marker.color.r = marker.color.g = marker.color.b = 0.7

            marker.lifetime.sec = 5
            marker_array.markers.append(marker)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = fused.header.stamp
            label.ns = obj_class
            label.id = marker_id + 1000
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pt_map.point.x
            label.pose.position.y = pt_map.point.y
            label.pose.position.z = (
                marker.pose.position.z + marker.scale.z + 0.08
            )
            label.scale.z = 0.18
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'{obj_class} | {conf:.2f}'
            label.lifetime.sec = 5

            marker_array.markers.append(label)
            marker_id += 1

        self.pub_fused.publish(fused)
        self.pub_marker.publish(marker_array)


def main(args=None):
    """Entry point for the perception fusion node."""
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
