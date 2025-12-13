#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

from vx_custom_msgs.msg import FusedObject, FusedObjects


class PerceptionFusionNode(Node):

    def __init__(self):
        super().__init__("perception_fusion_node")

        self.bridge = CvBridge()

        self.camera_intrinsics = None
        self.depth_image = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Marker cleanup tracker
        self.prev_marker_count = 0

        # Label mapping
        self.label_map = {
            "1": "Plant",
            "2": "Traffic_light_red",
            "3": "Traffic_light_green",
            "4": "Person",
            "5": "Car",
            "6": "House"
        }

        # Cuboid sizes
        self.sizes = {
            "Car": (0.33, 0.83, 0.25),
            "Plant": (0.23, 0.23, 0.38),
            "Person": (0.06, 0.06, 0.34),
            "Traffic_light_red": (0.05, 0.05, 0.52),
            "Traffic_light_green": (0.05, 0.05, 0.52),
            "House": (0.4, 0.4, 0.4)
        }

        self.create_subscription(CameraInfo,
                                 "/camera/camera/depth/camera_info",
                                 self.camera_info_callback, 10)

        self.create_subscription(Image,
                                 "/camera/camera/depth/image_rect_raw",
                                 self.depth_callback, 10)

        self.create_subscription(Detection2DArray,
                                 "/detectnet/detections",
                                 self.detections_callback, 10)

        self.pub_fused = self.create_publisher(FusedObjects,
                                               "/fused_objects_out",
                                               10)

        self.pub_markers = self.create_publisher(MarkerArray,
                                                 "/detected_markers",
                                                 10)

        self.get_logger().info("Fusion + RViz cleanup node started.")


    def camera_info_callback(self, msg):
        k = msg.k
        self.camera_intrinsics = {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]}


    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")


    def detections_callback(self, msg):
        if self.camera_intrinsics is None or self.depth_image is None:
            return

        fused = FusedObjects()
        fused.header = Header()
        fused.header.stamp = self.get_clock().now().to_msg()
        fused.station_id = 6
        fused.timestamp = time.time()

        marker_array = MarkerArray()
        marker_id = 0

        for det in msg.detections:

            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            try:
                raw = float(self.depth_image[v, u])
                depth = raw / 1000.0 if raw > 10 else raw
            except:
                continue

            Xc, Yc, Zc = self.pixel_to_camera(u, v, depth)

            pt_cam = PointStamped()
            pt_cam.header.frame_id = "camera_depth_optical_frame"
            pt_cam.header.stamp = fused.header.stamp
            pt_cam.point.x = Xc
            pt_cam.point.y = Yc
            pt_cam.point.z = Zc

            try:
                T_c2b = self.tf_buffer.lookup_transform("6/base_link",
                                                        "camera_depth_optical_frame",
                                                        rclpy.time.Time())
                pt_base = do_transform_point(pt_cam, T_c2b)
            except TransformException:
                continue

            try:
                T_b2m = self.tf_buffer.lookup_transform("map",
                                                        "6/base_link",
                                                        rclpy.time.Time())
                pt_map = do_transform_point(pt_base, T_b2m)
            except TransformException:
                continue

            # Class decoding
            obj_class = "unknown"
            conf = 0.0
            if det.results:
                h = det.results[0].hypothesis
                conf = float(h.score)
                try:
                    idx = int.from_bytes(h.class_id.encode(), "big")
                    obj_class = self.label_map.get(str(idx), f"class_{idx}")
                except:
                    obj_class = h.class_id

            obj = FusedObject()
            obj.object_id = marker_id + 1
            obj.object_class = obj_class
            obj.confidence = conf

            obj.base_link_position.x = pt_base.point.x
            obj.base_link_position.y = pt_base.point.y
            obj.base_link_position.z = pt_base.point.z

            obj.enu_position.x = pt_map.point.x
            obj.enu_position.y = pt_map.point.y
            obj.enu_position.z = pt_map.point.z

            fused.objects.append(obj)

            # RViz marker
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = fused.header.stamp
            m.ns = "detections"
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.pose.position.x = pt_map.point.x
            m.pose.position.y = pt_map.point.y
            m.pose.position.z = pt_map.point.z
            m.pose.orientation.w = 1.0

            size = self.sizes.get(obj_class, (0.2, 0.2, 0.2))
            m.scale.x, m.scale.y, m.scale.z = size

            m.color = ColorRGBA(r=0.1, g=0.7, b=1.0, a=0.85)

            marker_array.markers.append(m)
            marker_id += 1

        # ---- CLEANUP OLD MARKERS ----
        for old_id in range(marker_id, self.prev_marker_count):
            del_marker = Marker()
            del_marker.header.frame_id = "map"
            del_marker.header.stamp = fused.header.stamp
            del_marker.ns = "detections"
            del_marker.id = old_id
            del_marker.action = Marker.DELETE
            marker_array.markers.append(del_marker)

        self.prev_marker_count = marker_id

        # Publish both topics
        self.pub_fused.publish(fused)
        self.pub_markers.publish(marker_array)


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

