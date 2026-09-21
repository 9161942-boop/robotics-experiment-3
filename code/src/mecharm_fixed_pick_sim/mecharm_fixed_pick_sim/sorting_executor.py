#!/usr/bin/env python3
"""Execute the validated fixed-point sorting trajectory for each plan item.

The scene planner remains independent from motion execution.  This executor
uses the already calibrated single-point trajectory as a stable baseline and
attaches only the selected Gazebo model to the gripper during transport.  The
execution result is reported per object so a later six-point IK replacement can
keep the same planning and logging interfaces.
"""

import csv
import json
import math
import time
from pathlib import Path

import rclpy
from gazebo_msgs.msg import LinkStates, ModelStates
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = [
    "joint1_to_base", "joint2_to_joint1", "joint3_to_joint2",
    "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5",
]


class SortingExecutor(Node):
    def __init__(self):
        super().__init__("sorting_executor")
        self.declare_parameter("execute_motion", False)
        self.declare_parameter("step_duration_sec", 1.2)
        self.declare_parameter("log_file", "")
        self.execute_motion = bool(self.get_parameter("execute_motion").value)
        self.step_duration = max(0.2, float(self.get_parameter("step_duration_sec").value))

        self.arm_pub = self.create_publisher(JointTrajectory, "full_arm_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, "full_gripper_controller/joint_trajectory", 10)
        self.state_pub = self.create_publisher(String, "sorting_task/execution_state", 10)
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.result_pub = self.create_publisher(String, "sorting_task/execution_result", result_qos)
        plan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/sorting_task/result", self._on_plan, plan_qos)
        self.create_subscription(LinkStates, "/gazebo/link_states", self._on_link_states, 10)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._on_model_states, 10)
        self.entity_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        # These are the user-validated A-to-B teaching poses from
        # fixed_point_experiment.yaml / manual_animation.json.  The previous
        # sorting version accidentally used the separate official-model poses,
        # which follow a different visual-frame calibration.
        self.poses = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "pick_above": [0.52, 0.63, -0.58, 0.0, 0.0, 0.0],
            "pick_down": [0.53, 1.65, -1.31, 0.0, -0.09, 0.0],
            "pick_grasp": [0.54, 1.86, -1.22, 0.0, -0.62, 0.0],
            "lift": [0.53, 0.0, 0.0, 0.0, 0.0, 0.0],
            "place_above": [-0.48, 1.20, 0.23, 0.0, -1.41, 0.0],
            "place_down": [-0.48, 1.20, 0.23, 0.0, -1.41, 0.0],
            "lift_clear": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        self.gripper_open = 0.12
        self.gripper_closed = -0.55
        self.canonical_pick = (0.30, 0.18, 0.0275)
        # Separate drop cells make all six final objects visible in Gazebo.
        self.destination_cells = {
            # The destination objects rest on the table-level pads: pad top
            # 0.001 m plus half of the 0.055 m cylinder height.
            "red": [(0.115, -0.15, 0.0285), (0.14, -0.12, 0.0285), (0.165, -0.09, 0.0285)],
            "blue": [(0.235, -0.15, 0.0285), (0.26, -0.12, 0.0285), (0.285, -0.09, 0.0285)],
            "green": [(0.34, -0.02, 0.0285), (0.38, 0.02, 0.0285)],
        }
        self.destination_indices = {"red": 0, "blue": 0, "green": 0}

        configured_log = str(self.get_parameter("log_file").value).strip()
        log_path = Path(configured_log).expanduser() if configured_log else Path.home() / ".ros" / "mecharm_sorting_execution.csv"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = log_path.open("a", newline="", encoding="utf-8")
        self.writer = csv.writer(self.log_file)
        if log_path.stat().st_size == 0:
            self.writer.writerow(["wall_time", "event", "object_id", "class", "grid", "destination", "status", "message"])

        self.plan = []
        self.results = []
        self.object_poses = {}
        self.gripper_pose = None
        self.index = -1
        self.phase = "disabled" if not self.execute_motion else "waiting_plan"
        self.phase_started = time.monotonic()
        self.attached = False
        self.attach_offset = (0.0, 0.0, -0.05)
        self.service_future = None
        self.service_kind = ""
        self.next_attachment_update = 0.0
        self.release_check_at = 0.0
        self.publish_state(self.phase)
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(f"sorting executor ready (execute_motion={self.execute_motion})")

    def _log(self, event, item=None, status="", message=""):
        item = item or {}
        self.writer.writerow([
            time.strftime("%Y-%m-%d %H:%M:%S"), event, item.get("object_id", ""),
            item.get("class", ""), item.get("grid", ""), item.get("destination", ""), status, message,
        ])
        self.log_file.flush()

    def publish_state(self, state):
        self.phase = state
        self.phase_started = time.monotonic()
        self.state_pub.publish(String(data=state))

    def _on_plan(self, message):
        if not self.execute_motion or self.plan or self.phase not in {"waiting_plan", "disabled"}:
            return
        try:
            payload = json.loads(message.data)
            plan = payload.get("plan", [])
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid sorting plan: {exc}")
            self.publish_state("failed:invalid_plan")
            return
        if not plan:
            self.publish_state("failed:empty_plan")
            return
        self.plan = plan
        self.index = -1
        self._start_next_object()

    def _on_link_states(self, message):
        try:
            index = message.name.index("mecharm_270_full::gripper_base")
        except ValueError:
            return
        self.gripper_pose = message.pose[index]

    def _on_model_states(self, message):
        for name, pose in zip(message.name, message.pose):
            if name.startswith("object_"):
                self.object_poses[name] = pose

    def _duration_msg(self, seconds):
        from builtin_interfaces.msg import Duration
        return Duration(sec=int(seconds), nanosec=int((seconds - int(seconds)) * 1e9))

    def _send_pose(self, name, gripper):
        arm = JointTrajectory(joint_names=ARM_JOINTS)
        arm.points = [JointTrajectoryPoint(positions=list(self.poses[name]), time_from_start=self._duration_msg(self.step_duration))]
        self.arm_pub.publish(arm)
        grip = JointTrajectory(joint_names=["gripper_controller"])
        grip.points = [JointTrajectoryPoint(positions=[gripper], time_from_start=self._duration_msg(self.step_duration))]
        self.gripper_pub.publish(grip)
        self._log("motion_command", self.current_item(), message=f"pose={name},gripper={gripper:.3f}")

    def current_item(self):
        return self.plan[self.index] if 0 <= self.index < len(self.plan) else {}

    def destination_for(self, item):
        category = item.get("class", "blue")
        cells = self.destination_cells.get(category, self.destination_cells["blue"])
        key = category if category in self.destination_indices else "blue"
        cell_index = self.destination_indices[key]
        return cells[min(cell_index, len(cells) - 1)]

    def _request_entity(self, name, xyz, kind):
        if self.service_future is not None or not self.entity_client.service_is_ready():
            return False
        request = SetEntityState.Request()
        request.state.name = name
        request.state.pose.position.x, request.state.pose.position.y, request.state.pose.position.z = map(float, xyz)
        request.state.pose.orientation.w = 1.0
        request.state.twist.linear.x = request.state.twist.linear.y = request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = request.state.twist.angular.y = request.state.twist.angular.z = 0.0
        request.state.reference_frame = "world"
        self.service_future = self.entity_client.call_async(request)
        self.service_kind = kind
        return True

    def _start_next_object(self):
        self.index += 1
        self.attached = False
        if self.index >= len(self.plan):
            self._finish()
            return
        item = self.current_item()
        self.publish_state(f"reset:{item['object_id']}")
        if not self._request_entity(item["object_id"], self.canonical_pick, "reset"):
            self._log("exception", item, "waiting", "set_entity_state unavailable")

    def _finish(self):
        passed = sum(result["status"] == "passed" for result in self.results)
        status = "passed" if passed >= 5 else "failed"
        payload = {"status": status, "success_count": passed, "count": len(self.results), "results": self.results}
        self.result_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self.publish_state(f"task_{status}")
        self._log("task_result", status=status, message=json.dumps(payload, separators=(",", ":")))
        self.get_logger().info(json.dumps(payload, ensure_ascii=False))
        self.execute_motion = False

    def _service_done(self):
        if self.service_future is None:
            return False
        if not self.service_future.done():
            return True
        future, kind = self.service_future, self.service_kind
        self.service_future = None
        self.service_kind = ""
        try:
            response = future.result()
        except Exception as exc:
            self._fail_current(f"service exception: {exc}")
            return True
        if response is None or not response.success:
            self._fail_current(getattr(response, "status_message", "set_entity_state rejected") if response else "no response")
            return True
        if kind == "reset":
            self._send_pose("pick_above", self.gripper_open)
            self.publish_state("move_pick_above")
        elif kind == "release":
            self._send_pose("lift_clear", self.gripper_open)
            self.release_check_at = time.monotonic() + self.step_duration
            self.publish_state("verify_release")
        return True

    def _fail_current(self, message):
        item = self.current_item()
        self._log("exception", item, "failed", message)
        self.results.append({"object_id": item.get("object_id", ""), "status": "failed", "message": message})
        self.publish_state(f"failed:{item.get('object_id', 'unknown')}")
        self._start_next_object()

    def _complete_phase(self):
        item = self.current_item()
        if self.phase == "move_pick_above":
            self._send_pose("pick_down", self.gripper_open)
            self.publish_state("move_pick_down")
        elif self.phase == "move_pick_down":
            self._send_pose("pick_grasp", self.gripper_closed)
            self.publish_state("close_gripper")
        elif self.phase == "close_gripper":
            if self.gripper_pose is not None:
                self.attach_offset = (
                    self.canonical_pick[0] - self.gripper_pose.position.x,
                    self.canonical_pick[1] - self.gripper_pose.position.y,
                    self.canonical_pick[2] - self.gripper_pose.position.z,
                )
            self.attached = True
            self._log("grasp_attached", item, "passed", "selected object attached to gripper pose")
            self._send_pose("lift", self.gripper_closed)
            self.publish_state("lift_object")
        elif self.phase == "lift_object":
            self._send_pose("place_above", self.gripper_closed)
            self.publish_state("move_place_above")
        elif self.phase == "move_place_above":
            self._send_pose("place_down", self.gripper_closed)
            self.publish_state("move_place_down")
        elif self.phase == "move_place_down":
            self.attached = False
            destination = self.destination_for(item)
            if self._request_entity(item["object_id"], destination, "release"):
                self._log("release", item, "passed", f"destination={destination}")
                self.publish_state("release")
        elif self.phase == "lift_clear":
            category = item.get("class", "blue")
            key = category if category in self.destination_indices else "blue"
            self.destination_indices[key] += 1
            self.results.append({"object_id": item["object_id"], "class": category, "grid": item.get("grid", ""), "destination": item.get("destination", ""), "status": "passed"})
            self._log("object_complete", item, "passed", "pick-place sequence complete")
            self._start_next_object()

    def _update_attachment(self):
        if not self.attached or self.gripper_pose is None or self.service_future is not None:
            return
        now = time.monotonic()
        if now < self.next_attachment_update:
            return
        self.next_attachment_update = now + 0.06
        item = self.current_item()
        pose = self.gripper_pose
        xyz = (pose.position.x + self.attach_offset[0], pose.position.y + self.attach_offset[1], pose.position.z + self.attach_offset[2])
        self._request_entity(item["object_id"], xyz, "attach")

    def _tick(self):
        if not self.execute_motion:
            return
        if self._service_done():
            self._update_attachment()
            return
        if self.phase == "reset" and self.service_future is None:
            if not self._request_entity(self.current_item()["object_id"], self.canonical_pick, "reset"):
                return
        if self.phase == "release" and self.service_future is None:
            item = self.current_item()
            destination = self.destination_for(item)
            if not self._request_entity(item["object_id"], destination, "release"):
                return
        self._update_attachment()
        if self.phase == "verify_release":
            if time.monotonic() < self.release_check_at:
                return
            item = self.current_item()
            pose = self.object_poses.get(item.get("object_id"))
            destination = self.destination_for(item)
            error = math.hypot(pose.position.x - destination[0], pose.position.y - destination[1]) if pose else float("inf")
            if pose is None or error > 0.07 or pose.position.z > 0.20:
                self._fail_current(f"release pose outside destination: xy_error={error:.4f}")
                return
            self._send_pose("lift_clear", self.gripper_open)
            self.publish_state("lift_clear")
            return
        if self.phase in {"waiting_plan", "reset", "release", "disabled"}:
            return
        if time.monotonic() - self.phase_started >= self.step_duration:
            self._complete_phase()

    def destroy_node(self):
        if getattr(self, "log_file", None) is not None:
            self.log_file.close()
            self.log_file = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SortingExecutor()
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
