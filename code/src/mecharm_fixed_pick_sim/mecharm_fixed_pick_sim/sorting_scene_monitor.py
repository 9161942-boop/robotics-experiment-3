#!/usr/bin/env python3
"""Publish a deterministic object list for the desktop sorting experiment.

The first sorting milestone uses Gazebo model states as the ground-truth detector.
The topic and message format stay stable when a camera-based detector is added.
"""

import json

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class SortingSceneMonitor(Node):
    def __init__(self):
        super().__init__("sorting_scene_monitor")
        self.publisher = self.create_publisher(String, "sorting_scene/objects", 10)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.detection_publisher = self.create_publisher(
            Detection2DArray, "sorting_scene/detections", latched_qos
        )
        self.subscription = self.create_subscription(
            ModelStates, "/gazebo/model_states", self._on_model_states, 10
        )
        self.get_logger().info("sorting scene monitor ready")

    def _on_model_states(self, message):
        objects = []
        for index, name in enumerate(message.name):
            if not name.startswith("object_"):
                continue
            parts = name.split("_")
            category = parts[1] if len(parts) > 1 else "unknown"
            pose = message.pose[index]
            objects.append(
                {
                    "name": name,
                    "class": category,
                    "position": {
                        "x": round(pose.position.x, 5),
                        "y": round(pose.position.y, 5),
                        "z": round(pose.position.z, 5),
                    },
                }
            )
        objects.sort(key=lambda item: item["name"])
        output = String()
        output.data = json.dumps(
            {"frame": "world", "count": len(objects), "objects": objects},
            ensure_ascii=True,
            separators=(",", ":"),
        )
        self.publisher.publish(output)

        # The overhead camera is fixed at 640x480.  This deterministic
        # projection keeps the detector contract identical to a later RGB
        # detector while the simulation uses Gazebo state as ground truth.
        stamp = self.get_clock().now().to_msg()
        detections = Detection2DArray()
        detections.header = Header(stamp=stamp, frame_id="sorting_camera_link")
        for item in objects:
            position = item["position"]
            detection = Detection2D()
            detection.header = detections.header
            detection.id = item["name"]
            detection.bbox.center.position.x = 320.0 + (position["y"] - 0.18) * 900.0
            detection.bbox.center.position.y = 240.0 - (position["x"] - 0.30) * 900.0
            detection.bbox.size_x = 34.0
            detection.bbox.size_y = 34.0
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = item["class"]
            result.hypothesis.score = 0.99
            detection.results.append(result)
            detections.detections.append(detection)
        self.detection_publisher.publish(detections)


def main(args=None):
    rclpy.init(args=args)
    node = SortingSceneMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
