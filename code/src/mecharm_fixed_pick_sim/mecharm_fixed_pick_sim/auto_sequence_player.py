import json
import math
import time
from pathlib import Path

from gazebo_msgs.msg import LinkStates, ModelStates
from gazebo_msgs.srv import SetEntityState
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = [
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
]
GRIPPER_JOINT = "gripper_controller"

OBJECT_MODELS = (
    "object_red_1", "object_blue_1", "object_red_3", "object_blue_2",
    "object_green_1", "object_green_2",
)
GREEN_LOCKED_WAYPOINTS = {
    "above_green_1", "pick_4", "above_green_2", "pick_5", "release_4", "release_5",
}
# The recorded sorting order is positional, not simply color-based:
# top-left, top-right, bottom-left, bottom-right.
PICK_OBJECT_BY_STEP = {
    "pick_1": "object_red_1",
    "pick": "object_blue_1",
    "pick_3": "object_red_3",
    "pick_2": "object_blue_2",
    "pick_4": "object_green_1",
    "pick_5": "object_green_2",
}
INITIAL_OBJECT_POSES = {
    "object_red_1": (0.24, 0.12),
    "object_blue_1": (0.30, 0.12),
    "object_red_3": (0.24, 0.18),
    "object_blue_2": (0.30, 0.18),
    "object_green_1": (0.24, 0.24),
    "object_green_2": (0.30, 0.24),
}
GRASP_REFERENCE_OFFSET = (0.0, 0.06, -0.02)
def _is_pick_step(name):
    name = name.strip().lower().replace("-", "_")
    return (
        name in {"pick", "grasp", "grasp_object"}
        or (name.startswith("pick_") and name[len("pick_") :].isdigit())
        or (name.startswith("grasp_") and name[len("grasp_") :].isdigit())
    )


def _is_release_step(name):
    name = name.strip().lower().replace("-", "_")
    return (
        name in {"release", "release_object", "open_gripper"}
        or (name.startswith("release_") and name[len("release_") :].isdigit())
        or (
            name.startswith("open_gripper_")
            and name[len("open_gripper_") :].isdigit()
        )
    )


class AutoSequencePlayer(Node):
    def __init__(self):
        super().__init__("mecharm_auto_sequence_player")
        default_file = Path.cwd() / "manual_animation.json"
        self.declare_parameter("animation_file", str(default_file))
        self.declare_parameter("start_index", 0)
        self.declare_parameter("repeat_count", 1)
        self.declare_parameter("settle_extra_sec", 0.25)
        self.declare_parameter("controller_timeout_sec", 30.0)
        self.declare_parameter("vacuum_timeout_sec", 5.0)
        self.declare_parameter("visual_attach_enabled", True)
        self.declare_parameter("attach_update_hz", 20.0)
        self.declare_parameter("vacuum_enabled", False)
        self.declare_parameter("release_fall_duration_sec", 0.8)
        # The second green grid cell is at the edge of the zero-roll/yaw
        # posture's reachable envelope; allow a small calibration margin.
        self.declare_parameter("grasp_position_tolerance", 0.05)
        self.declare_parameter("reset_objects_on_start", True)

        self.arm_pub = self.create_publisher(
            JointTrajectory, "full_arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "full_gripper_controller/joint_trajectory", 10
        )
        self.pad_effort_pub = self.create_publisher(
            Float64MultiArray,
            "contact_pad_effort_controller/commands",
            10,
        )
        self.vacuum_client = self.create_client(SetBool, "/mecharm_grasp/switch")
        self.visual_attach_enabled = bool(self.get_parameter("visual_attach_enabled").value)
        self.attach_update_period = 1.0 / max(1.0, float(self.get_parameter("attach_update_hz").value))
        self.entity_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.create_subscription(ModelStates, "/gazebo/model_states", self._on_model_states, 10)
        self.create_subscription(LinkStates, "/gazebo/link_states", self._on_link_states, 10)
        self.model_poses = {}
        self.gripper_pose = None
        self.attached_object = None
        self.attach_offset = None
        self.last_attach_update = 0.0
        self.entity_future = None
        self.entity_future_kind = ""
        self.used_objects = set()
        self.object_rest_z = None
        self.release_animation = None

    @staticmethod
    def _duration(seconds):
        from builtin_interfaces.msg import Duration

        seconds = max(0.1, float(seconds))
        msg = Duration()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - msg.sec) * 1e9)
        return msg

    def _wait_for_controllers(self):
        timeout = float(self.get_parameter("controller_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.arm_pub.get_subscription_count() > 0
                and self.gripper_pub.get_subscription_count() > 0
            ):
                self.get_logger().info("arm and gripper controllers are ready")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            "controllers were not ready; start Gazebo with the full arm/gripper controllers"
        )
        return False

    def _reset_objects_on_start(self):
        if not bool(self.get_parameter("reset_objects_on_start").value):
            return True
        timeout = float(self.get_parameter("controller_timeout_sec").value)
        if not self.entity_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warning(
                "Gazebo set_entity_state is unavailable; keeping current object poses"
            )
            return False
        reset_z = self.object_rest_z if self.object_rest_z is not None else 0.0275
        for name, (x, y) in INITIAL_OBJECT_POSES.items():
            request = SetEntityState.Request()
            request.state.name = name
            request.state.pose.position.x = x
            request.state.pose.position.y = y
            request.state.pose.position.z = reset_z
            request.state.pose.orientation.w = 1.0
            request.state.reference_frame = "world"
            future = self.entity_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None or not future.result().success:
                self.get_logger().warning(f"failed to reset {name} to pickup grid")
                continue
            self.model_poses[name] = request.state.pose
        self.used_objects.clear()
        self.attached_object = None
        self.attach_offset = None
        self.release_animation = None
        self.get_logger().info("sorting objects reset to pickup grid")
        return True

    def _on_model_states(self, message):
        for name, pose in zip(message.name, message.pose):
            if name in OBJECT_MODELS:
                self.model_poses[name] = pose
                if self.object_rest_z is None:
                    self.object_rest_z = float(pose.position.z)
            elif name == "mecharm_270_full":
                # ModelStates has the arm root only; link pose is supplied by
                # LinkStates, handled by _on_link_states below.
                continue

    def _on_link_states(self, message):
        try:
            index = message.name.index("mecharm_270_full::gripper_base")
        except ValueError:
            return
        self.gripper_pose = message.pose[index]

    @staticmethod
    def _rotate_vector(q, vector):
        x, y, z = vector
        tx = 2.0 * (q.y * z - q.z * y)
        ty = 2.0 * (q.z * x - q.x * z)
        tz = 2.0 * (q.x * y - q.y * x)
        return (
            x + q.w * tx + (q.y * tz - q.z * ty),
            y + q.w * ty + (q.z * tx - q.x * tz),
            z + q.w * tz + (q.x * ty - q.y * tx),
        )

    def _poll_entity_future(self):
        if self.entity_future is None or not self.entity_future.done():
            return
        future = self.entity_future
        kind = self.entity_future_kind
        self.entity_future = None
        self.entity_future_kind = ""
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Gazebo object pose update failed ({kind}): {exc}")
            return
        if response is None or not response.success:
            self.get_logger().warning(f"Gazebo object pose update rejected ({kind})")

    def _request_entity_pose(self, pose, kind):
        if self.entity_future is not None or not self.attached_object:
            return
        if not self.entity_client.service_is_ready():
            return
        request = SetEntityState.Request()
        request.state.name = self.attached_object
        request.state.pose = pose
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        request.state.reference_frame = "world"
        self.entity_future = self.entity_client.call_async(request)
        self.entity_future_kind = kind

    def _start_visual_attachment(self, waypoint_name=""):
        if not self.visual_attach_enabled or self.attached_object or self.gripper_pose is None:
            return
        candidates = [
            (name, pose) for name, pose in self.model_poses.items()
            if name not in self.used_objects
        ]
        if not candidates:
            self.get_logger().warning("no unused sorting object is visible in Gazebo")
            return
        normalized = waypoint_name.strip().lower().replace("-", "_")
        expected_object = PICK_OBJECT_BY_STEP.get(normalized)
        if expected_object:
            exact_candidate = [item for item in candidates if item[0] == expected_object]
            if exact_candidate:
                candidates = exact_candidate
            else:
                self.get_logger().warning(
                    f"expected object {expected_object} is unavailable at {waypoint_name}"
                )
        gx, gy, gz = (self.gripper_pose.position.x, self.gripper_pose.position.y, self.gripper_pose.position.z)
        # The cylinder is held between the two pads, not at the gripper_base
        # origin.  Select and validate candidates against the pad-center
        # reference point so a bad waypoint cannot cause an airborne grasp.
        rx, ry, rz = self._rotate_vector(self.gripper_pose.orientation, GRASP_REFERENCE_OFFSET)
        grasp_point = (gx + rx, gy + ry, gz + rz)
        name, object_pose = min(
            candidates,
            key=lambda item: math.dist((item[1].position.x, item[1].position.y, item[1].position.z), grasp_point),
        )
        distance = math.dist(
            (object_pose.position.x, object_pose.position.y, object_pose.position.z),
            grasp_point,
        )
        tolerance = max(0.0, float(self.get_parameter("grasp_position_tolerance").value))
        if distance > tolerance:
            self.get_logger().warning(
                f"grasp rejected at {waypoint_name}: nearest object {name} "
                f"is {distance:.3f} m from gripper pad center "
                f"(tolerance={tolerance:.3f} m)"
            )
            return
        q = self.gripper_pose.orientation
        inverse_q = type(q)(x=-q.x, y=-q.y, z=-q.z, w=q.w)
        world_offset = (
            object_pose.position.x - gx,
            object_pose.position.y - gy,
            object_pose.position.z - gz,
        )
        self.attach_offset = self._rotate_vector(inverse_q, world_offset)
        self.attached_object = name
        self.get_logger().info(f"visual grasp attached: {name} (pad error={distance:.3f} m)")

    def _update_visual_attachment(self):
        self._poll_entity_future()
        if self.release_animation is not None:
            return
        if not self.attached_object or self.gripper_pose is None or self.attach_offset is None:
            return
        now = time.monotonic()
        if now - self.last_attach_update < self.attach_update_period:
            return
        self.last_attach_update = now
        q = self.gripper_pose.orientation
        dx, dy, dz = self._rotate_vector(q, self.attach_offset)
        pose = type(self.gripper_pose)()
        pose.position.x = self.gripper_pose.position.x + dx
        pose.position.y = self.gripper_pose.position.y + dy
        pose.position.z = self.gripper_pose.position.z + dz
        # Keep cylinders upright while they are logically attached.  Letting
        # them inherit the gripper orientation makes them lie sideways during
        # transport and causes violent collisions with the table/other parts.
        pose.orientation.w = 1.0
        self._request_entity_pose(pose, "attach")

    def _release_visual_attachment(self, waypoint_name=""):
        if not self.attached_object:
            return
        name = self.attached_object
        if self.gripper_pose is None or self.attach_offset is None:
            self.used_objects.add(name)
            self.attached_object = None
            self.attach_offset = None
            self.get_logger().warning(f"release animation skipped: no gripper pose for {name}")
            return

        q = self.gripper_pose.orientation
        dx, dy, dz = self._rotate_vector(q, self.attach_offset)
        start = (
            self.gripper_pose.position.x + dx,
            self.gripper_pose.position.y + dy,
            self.gripper_pose.position.z + dz,
        )
        target_z = self.object_rest_z if self.object_rest_z is not None else start[2]
        fall_target_z = min(float(target_z), start[2])
        self.release_animation = {
            "name": name,
            "start": start,
            "target_z": fall_target_z,
            "started_at": time.monotonic(),
            "duration": max(0.1, float(self.get_parameter("release_fall_duration_sec").value)),
            "final_sent": False,
        }
        self.get_logger().info(
            f"vertical release animation started: {name} "
            f"({start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f}) -> z={fall_target_z:.3f}"
        )

    def _update_release_animation(self):
        animation = self.release_animation
        if animation is None or self.attached_object is None:
            return
        self._poll_entity_future()
        if self.entity_future is not None or not self.entity_client.service_is_ready():
            return

        elapsed = time.monotonic() - animation["started_at"]
        fraction = min(1.0, elapsed / animation["duration"])
        sx, sy, sz = animation["start"]
        pose = type(self.gripper_pose)()
        pose.position.x = sx
        pose.position.y = sy
        pose.position.z = sz + (animation["target_z"] - sz) * fraction
        pose.orientation.w = 1.0
        if fraction < 1.0:
            self._request_entity_pose(pose, "release_fall")
            return
        if not animation["final_sent"]:
            self._request_entity_pose(pose, "release_fall")
            animation["final_sent"] = True
            return
        name = self.attached_object
        self.used_objects.add(name)
        self.attached_object = None
        self.attach_offset = None
        self.release_animation = None
        self.get_logger().info(f"vertical release animation completed: {name}")

    def _send_waypoint(self, point, waypoint_name="", lock_green_axes=False):
        joints = [float(value) for value in point["joints"]]
        if len(joints) != len(ARM_JOINTS):
            raise ValueError("each waypoint must contain six arm joint values")
        # Once the green cycle starts, hold the two symmetry-axis joints at
        # zero for every command through its final home pose.  This includes
        # shared names such as "First" and "initial", so no intermediate
        # transition can reintroduce a rotation.
        if lock_green_axes or waypoint_name.strip().lower() in GREEN_LOCKED_WAYPOINTS:
            joints[3] = 0.0
            joints[5] = 0.0
        duration = max(0.1, float(point.get("duration", 1.0)))
        gripper = float(point.get("gripper", 0.12))
        pad_effort = float(point.get("pad_effort", point.get("pad_size_mm", -4.0)))

        arm = JointTrajectory(joint_names=ARM_JOINTS)
        arm.points = [
            JointTrajectoryPoint(
                positions=joints,
                time_from_start=self._duration(duration),
            )
        ]
        self.arm_pub.publish(arm)

        # The normal sorting controller owns only the command joint; the
        # remaining finger joints are mimic joints in this mode.
        grip = JointTrajectory(joint_names=[GRIPPER_JOINT])
        grip.points = [
            JointTrajectoryPoint(
                positions=[gripper],
                time_from_start=self._duration(duration),
            )
        ]
        self.gripper_pub.publish(grip)

        if self.pad_effort_pub.get_subscription_count() > 0:
            self.pad_effort_pub.publish(
                Float64MultiArray(data=[pad_effort, pad_effort])
            )
        return duration

    def _set_vacuum(self, enabled):
        timeout = float(self.get_parameter("vacuum_timeout_sec").value)
        if not self.vacuum_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warning(
                "vacuum service is unavailable; continuing without grasp switch"
            )
            return False
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.vacuum_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            self.get_logger().warning("vacuum service did not answer")
            return False
        response = future.result()
        if not response.success:
            self.get_logger().warning(f"vacuum switch rejected: {response.message}")
        return bool(response.success)

    def play(self):
        path = Path(str(self.get_parameter("animation_file").value)).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        data = json.loads(path.read_text(encoding="utf-8"))
        waypoints = data.get("waypoints", {})
        sequence = data.get("sequence", [])
        if not sequence:
            raise ValueError(f"sequence is empty: {path}")

        start_index = max(0, int(self.get_parameter("start_index").value))
        repeat_count = max(1, int(self.get_parameter("repeat_count").value))
        extra = max(0.0, float(self.get_parameter("settle_extra_sec").value))
        for name in sequence:
            if name not in waypoints:
                raise KeyError(f"sequence point is missing from waypoints: {name}")

        self._reset_objects_on_start()

        self.get_logger().info(
            f"playing {len(sequence)} waypoints from {path} "
            f"(start={start_index}, repeats={repeat_count})"
        )
        for repeat in range(repeat_count):
            green_cycle_active = False
            for index, name in enumerate(sequence[start_index:], start=start_index):
                if not rclpy.ok():
                    return
                if name.strip().lower() == "above_green_1":
                    green_cycle_active = True
                duration = self._send_waypoint(
                    waypoints[name], name, lock_green_axes=green_cycle_active
                )
                self.get_logger().info(
                    f"[{index + 1}/{len(sequence)}] {name} ({duration:.2f}s)"
                )
                deadline = time.monotonic() + duration + extra
                while rclpy.ok() and time.monotonic() < deadline:
                    self._update_visual_attachment()
                    self._update_release_animation()
                    rclpy.spin_once(self, timeout_sec=0.05)
                vacuum_enabled = bool(self.get_parameter("vacuum_enabled").value)
                if _is_pick_step(name):
                    if vacuum_enabled:
                        self._set_vacuum(True)
                    self._start_visual_attachment(name)
                    self.get_logger().info(f"grasp enabled at {name}")
                elif _is_release_step(name):
                    if vacuum_enabled:
                        self._set_vacuum(False)
                    self._release_visual_attachment(name)
                    self.get_logger().info(f"grasp released at {name}")
                self._update_visual_attachment()
                self._update_release_animation()
        self.get_logger().info("automatic sequence playback completed")


def main(args=None):
    rclpy.init(args=args)
    node = AutoSequencePlayer()
    try:
        if node._wait_for_controllers():
            node.play()
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        node.get_logger().error(f"automatic playback failed: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
