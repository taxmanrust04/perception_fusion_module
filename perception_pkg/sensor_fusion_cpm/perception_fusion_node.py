#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
from vx_custom_msgs.msg import FusedObject, FusedObjects

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point


class PerceptionFusionNode(Node):

    def __init__(self):
        super().__init__("perception_fusion_node")

        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.depth_image = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.label_map = {
            "1": "Plant",
            "2": "Traffic_light_red",
            "3": "Traffic_light_green",
            "4": "Person",
            "5": "Car",
            "6": "House"
        }

        self.create_subscription(
            CameraInfo,
            "/camera/camera/depth/camera_info",
            self.camera_info_callback,
            10
        )

        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_callback,
            10
        )

        self.create_subscription(
            Detection2DArray,
            "/detectnet/detections",
            self.detections_callback,
            10
        )

        self.pub_fused = self.create_publisher(
            FusedObjects,
            "/fused_objects_out",
            10
        )

        self.pub_marker = self.create_publisher(
            MarkerArray,
            "/visualization_marker",
            10
        )

        self.get_logger().info("Perception Fusion Node started (TF + 3D RViz markers).")

    # -------------------------------------------------------------

    def camera_info_callback(self, msg: CameraInfo):
        k = msg.k
        self.camera_intrinsics = {
            "fx": k[0],
            "fy": k[4],
            "cx": k[2],
            "cy": k[5]
        }

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    # -------------------------------------------------------------

    def detections_callback(self, msg: Detection2DArray):

        if self.camera_intrinsics is None or self.depth_image is None:
            return

        fused = FusedObjects()
        fused.header = Header()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.station_id = 6
        fused.timestamp = time.time()

        marker_array = MarkerArray()

        # Clear old markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for i, det in enumerate(msg.detections):

            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            try:
                raw_depth = float(self.depth_image[v, u])
                depth = raw_depth / 1000.0 if raw_depth > 10 else raw_depth
            except:
                continue

            Xc, Yc, Zc = self.pixel_to_camera(u, v, depth)

            pt_cam = PointStamped()
            pt_cam.header.frame_id = "camera_depth_optical_frame"
            pt_cam.header.stamp = self.get_clock().now().to_msg()
            pt_cam.point.x = Xc
            pt_cam.point.y = Yc
            pt_cam.point.z = Zc

            try:
                T = self.tf_buffer.lookup_transform(
                    "map",
                    "camera_depth_optical_frame",
                    rclpy.time.Time()
                )
                pt_map = do_transform_point(pt_cam, T)
            except TransformException:
                continue

            obj_class = "unknown"
            confidence = 0.0

            if det.results:
                res = det.results[0].hypothesis
                confidence = float(res.score)
                try:
                    idx = int.from_bytes(res.class_id.encode(), "big")
                    obj_class = self.label_map.get(str(idx), "unknown")
                except:
                    obj_class = res.class_id

            obj = FusedObject()
            obj.object_id = i + 1
            obj.object_class = obj_class
            obj.confidence = confidence
            obj.enu_position.x = pt_map.point.x
            obj.enu_position.y = pt_map.point.y
            obj.enu_position.z = pt_map.point.z
            fused.objects.append(obj)

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detected_objects"
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = pt_map.point.x
            marker.pose.position.y = pt_map.point.y
            marker.pose.orientation.w = 1.0
            marker.color.a = 0.9

            if obj_class == "Car":
                marker.type = Marker.CUBE
                marker.scale.x = 0.8
                marker.scale.y = 0.3
                marker.scale.z = 0.32
                marker.pose.position.z = pt_map.point.z + 0.16
                marker.color.b = 1.0

            elif obj_class == "Person":
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.07
                marker.scale.y = 0.07
                marker.scale.z = 0.30
                marker.pose.position.z = pt_map.point.z + 0.15
                marker.color.r = 1.0

            elif obj_class in ["Traffic_light_red", "Traffic_light_green"]:
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.52
                marker.pose.position.z = pt_map.point.z + 0.26
                marker.color.g = 1.0

            elif obj_class == "Plant":
                marker.type = Marker.CUBE
                marker.scale.x = 0.23
                marker.scale.y = 0.23
                marker.scale.z = 0.38
                marker.pose.position.z = pt_map.point.z + 0.19
                marker.color.r = 0.4
                marker.color.g = 0.25
                marker.color.b = 0.1

            else:
                continue

            marker_array.markers.append(marker)

        self.pub_fused.publish(fused)
        self.pub_marker.publish(marker_array)

    # -------------------------------------------------------------

    def pixel_to_camera(self, u, v, depth):
        fx = self.camera_intrinsics["fx"]
        fy = self.camera_intrinsics["fy"]
        cx = self.camera_intrinsics["cx"]
        cy = self.camera_intrinsics["cy"]

        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        return X, Y, Z


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
