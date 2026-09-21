#!/usr/bin/env python3
"""Grid-based sorting state machine for the desktop sorting experiment.

This first controller validates the recognition, grid assignment, destination
selection, exception handling, and logging contract. Motion execution is kept
behind the ``execute_motion`` parameter until six fixed-point trajectories have
been calibrated against the sorting grid.
"""

import csv
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


class SortingTask(Node):
    def __init__(self):
        super().__init__("sorting_task")
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("min_confidence", 0.80)
        self.declare_parameter("grid_tolerance_px", 48.0)
        self.declare_parameter("log_file", "")
        self.execute_motion = bool(self.get_parameter("execute_motion").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.grid_tolerance_px = float(self.get_parameter("grid_tolerance_px").value)
        configured_log = str(self.get_parameter("log_file").value).strip()
        log_path = Path(configured_log).expanduser() if configured_log else Path.home() / ".ros" / "mecharm_sorting_task.csv"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = log_path.open("a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.log_file)
        if log_path.stat().st_size == 0:
            self.writer.writerow(["wall_time", "event", "object_id", "class", "grid", "destination", "status", "message"])
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, "sorting_task/state", latched_qos)
        self.result_pub = self.create_publisher(String, "sorting_task/result", latched_qos)
        self.subscription = self.create_subscription(
            Detection2DArray, "/sorting_scene/detections", self._on_detections, 10
        )
        self.last_signature = None
        self.processed = set()
        # In execution mode the objects intentionally leave the pickup grid.
        # Lock the first complete plan so those post-release poses do not
        # create a second, empty plan while the executor is still running.
        self.plan_locked = False
        self.get_logger().info(f"sorting task ready (execute_motion={self.execute_motion})")

    def _log(self, event, object_id="", category="", grid="", destination="", status="", message=""):
        self.writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), event, object_id, category, grid, destination, status, message])
        self.log_file.flush()

    def _grid_for(self, x, y):
        # The camera is rotated so world-y is image-x and world-x is image-y.
        # With A=(0.30, 0.18), the pickup layout appears as three image columns
        # and three image rows in the 640 x 480 image.
        columns = [212.0, 266.0, 320.0]
        rows = [186.0, 240.0, 294.0]
        col = min(range(len(columns)), key=lambda i: abs(x - columns[i]))
        row = min(range(3), key=lambda i: abs(y - rows[i]))
        distance = math.hypot(x - columns[col], y - rows[row])
        if distance > self.grid_tolerance_px:
            return None, distance
        return f"r{row + 1}c{col + 1}", distance

    def _on_detections(self, message):
        if self.execute_motion and self.plan_locked:
            return
        signature = tuple(sorted((d.id, round(d.bbox.center.position.x, 1), round(d.bbox.center.position.y, 1)) for d in message.detections))
        if signature == self.last_signature:
            return
        self.last_signature = signature
        self._publish_state("detecting")
        occupied = set()
        plan = []
        for detection in message.detections:
            object_id = detection.id or "anonymous"
            if not detection.results:
                self._log("detect", object_id, status="skipped", message="no classification result")
                continue
            hypothesis = max(detection.results, key=lambda result: result.hypothesis.score)
            category = hypothesis.hypothesis.class_id.strip().lower()
            confidence = float(hypothesis.hypothesis.score)
            grid, distance = self._grid_for(detection.bbox.center.position.x, detection.bbox.center.position.y)
            if confidence < self.min_confidence:
                self._log("detect", object_id, category, status="skipped", message=f"confidence={confidence:.3f}")
                continue
            if category not in {"red", "blue", "green"}:
                self._log("detect", object_id, category, status="skipped", message="unknown class")
                continue
            if grid is None:
                self._log("detect", object_id, category, status="skipped", message=f"outside grid distance={distance:.1f}px")
                continue
            if grid in occupied:
                self._log("detect", object_id, category, grid, status="skipped", message="duplicate grid assignment")
                continue
            occupied.add(grid)
            destination = f"{category}_sort_bin"
            status = "planned" if not self.execute_motion else "queued"
            self._log("plan", object_id, category, grid, destination, status=status, message=f"confidence={confidence:.3f}")
            plan.append({"object_id": object_id, "class": category, "grid": grid, "destination": destination, "status": status})
        result = {"status": "planned", "count": len(plan), "execute_motion": self.execute_motion, "plan": plan}
        self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False, separators=(",", ":"))))
        self._publish_state(f"plan_ready:{len(plan)}")
        if self.execute_motion and plan:
            self.plan_locked = True

    def _publish_state(self, state):
        self.state_pub.publish(String(data=state))

    def destroy_node(self):
        if getattr(self, "log_file", None) is not None:
            self.log_file.close()
            self.log_file = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SortingTask()
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
