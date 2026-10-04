#!/usr/bin/env python3
"""
Headless, CPU-only kinematic simulator for a 6-DOF arm.

Publishes:
  /joint_states          (sensor_msgs/JointState) at ~50 Hz
  /arm/cmd_traj          (trajectory_msgs/JointTrajectoryPoint list) command echo
  /clock                 optional sim time when use_sim_time is on

Consumes:
  ~/arm/command          (std_msgs/Float64MultiArray) target joint angles

There is no physics engine here on purpose: without a GPU, Gazebo's render
loop would dominate the budget and the visual result in Foxglove is identical.
TF is produced by robot_state_publisher from the URDF + /joint_states.
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

"""
Named joint poses.

A pose is a mapping from joint *name* to a target value in radians (metres
for prismatic joints). Resolving by name instead of by index means the same
preset works for the Franka FR3, the UR5 and the demo arm, whatever order the
URDF happens to list the joints in.

Values below were chosen to resemble the standard marketing photo of the FR3:
shoulder rotated out to one side, elbow folded, wrist held level so the
gripper faces down/forward. They are deliberately conservative -- well inside
the FR3's +-2.9rad arm range -- because these are *visual* poses, not a
trajectory, and an unreachable target would silently clip against the limit.
"""

# Presets for the 7-DoF Franka FR3 arm (joints 1-7), plus finger joints.
# The finger joints are prismatic (0.0 = closed, 0.04 = fully open), so they
# stay at 0.0 here: the photo shows the gripper closed on nothing.
FR3_PHOTO_POSE = {
    # j1  base rotation ~ -35deg: swings the whole arm to the robot's left,
    #     which is what produces the diagonal silhouette in the photo.
    "fr3_joint1": -0.60,
    # j2  shoulder pitch ~ +95deg: raises the upper arm away from the base.
    "fr3_joint2": 1.65,
    # j3  elbow ~ -95deg: folds the forearm back on itself, forming the
    #     characteristic zig-zag of the photo.
    "fr3_joint3": -1.65,
    # j4  forearm roll ~ 0: keeps the elbow plane upright.
    "fr3_joint4": 0.0,
    # j5  wrist pitch ~ -70deg: bends the wrist so the gripper points forward
    #     and slightly down rather than straight ahead.
    "fr3_joint5": -1.20,
    # j6  wrist roll ~ 0: keeps the gripper jaws square.
    "fr3_joint6": 0.0,
    # j7  wrist roll ~ 0.
    "fr3_joint7": 0.0,
    "fr3_finger_joint1": 0.0,
    "fr3_finger_joint2": 0.0,
}

# The FR3 xacro names its joints "fr3_joint1".."fr3_joint7", but some
# distributions use the bare "panda_joint*" convention. Accept both so the
# preset survives a package rename.
_PANDA_ALIASES = {
    f"panda_joint{i + 1}": v
    for i, v in enumerate(
        [-0.60, 1.65, -1.65, 0.0, -1.20, 0.0, 0.0])
}

PRESETS = {
    "photo": dict(FR3_PHOTO_POSE, **_PANDA_ALIASES),
    # Arms-up "ready" pose: useful as a recognisable default in Foxglove.
    "home": {
        "fr3_joint1": 0.0, "fr3_joint2": 0.60, "fr3_joint3": 0.0,
        "fr3_joint4": -0.785, "fr3_joint5": 0.785, "fr3_joint6": 0.0,
        "fr3_joint7": 0.785,
        "panda_joint1": 0.0, "panda_joint2": 0.60, "panda_joint3": 0.0,
        "panda_joint4": -0.785, "panda_joint5": 0.785, "panda_joint6": 0.0,
        "panda_joint7": 0.785,
        "fr3_finger_joint1": 0.0, "fr3_finger_joint2": 0.0,
    },
}


def resolve(preset_name, joint_names, urdf_root=None):
    """Expand a named preset into a full, ordered list of joint targets.

    Joints the preset does not mention fall back to 0.0. When ``urdf_root`` is
    given, every value is clamped into the joint's own <limit> so a bad preset
    can never command an angle the hardware cannot reach.

    Returns (values, report) where report lists one line per joint, for
    logging.
    """
    if preset_name not in PRESETS:
        raise KeyError(
            f"unknown pose '{preset_name}'; try: "
            + ", ".join(sorted(PRESETS)))
    preset = PRESETS[preset_name]

    limits = {}
    if urdf_root is not None:
        for j in urdf_root.findall("joint"):
            lim = j.find("limit")
            if lim is None:
                continue
            try:
                limits[j.get("name")] = (
                    float(lim.get("lower", "-inf")),
                    float(lim.get("upper", "inf")),
                )
            except (TypeError, ValueError):
                continue

    values, report = [], []
    for name in joint_names:
        val = float(preset.get(name, 0.0))
        note = ""
        if name in limits:
            lo, hi = limits[name]
            if val < lo or val > hi:
                clamped = max(lo, min(hi, val))
                note = f"  (clamped {val:.3f} -> {clamped:.3f})"
                val = clamped
        values.append(val)
        report.append(f"    {name:<28} {val:+.3f}{note}")
    return values, report


class ArmSim(Node):
    def __init__(self, joint_names=None, urdf_root=None):
        super().__init__("arm_sim")

        # Joint names come from the URDF when one is supplied, so this node
        # can drive whatever robot is loaded (demo arm, Franka FR3, UR5, ...).
        self.joint_names = list(joint_names) if joint_names else [
            "shoulder_pan_joint",
            "elbow_pitch_joint",
            "forearm_roll_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
            "gripper_joint",
        ]

        # Start in a named pose when one was requested, else all-zeros.
        # NOTE: an all-zero pose is NOT neutral for a Franka -- it leaves the
        # arm pointing straight ahead -- so prefer a named preset here.
        self.urdf_root = urdf_root
        self.preset_name = self.declare_parameter("pose", "home").value
        self.pos, self.report = self._start_pose(self.preset_name, urdf_root)
        self.target = list(self.pos)
        self.vel = [0.0] * len(self.joint_names)

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.state_pub = self.create_publisher(
            Float64MultiArray, "~/joint_positions", 10
        )
        self.create_subscription(
            Float64MultiArray, "~/arm/command", self._on_cmd, 10
        )
        # Named poses: /arm_sim/pose  std_msgs/String  -> "photo" | "home"
        self.create_subscription(
            String, "~/pose", self._on_pose, 10
        )

        self.rate_hz = 50.0
        self.t = 0.0
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"arm_sim up: {len(self.joint_names)} joints @ {self.rate_hz} Hz "
            f"(pose '{self.preset_name}')"
        )

    def _start_pose(self, name, urdf_root):
        """Resolve the initial pose, falling back to zeros if unavailable."""
        try:
            return resolve(name, self.joint_names, urdf_root)
        except KeyError as exc:
            self.get_logger().warn(f"{exc}; starting at zeros")
        return [0.0] * len(self.joint_names), []

    def _on_pose(self, msg):
        """Apply a named preset to every joint at once."""
        name = (msg.data or "").strip()
        try:
            values, report = resolve(name, self.joint_names, self.urdf_root)
        except KeyError as exc:
            self.get_logger().warn(str(exc))
            return
        self.target = list(values)
        self.get_logger().info(f"pose '{name}':\n" + "\n".join(report))

    def _on_cmd(self, msg: Float64MultiArray) -> None:
        n = min(len(msg.data), len(self.target))
        for i in range(n):
            self.target[i] = float(msg.data[i])
        self.get_logger().info(f"target <- {self.target}")

    def _tick(self) -> None:
        dt = 1.0 / self.rate_hz
        self.t += dt

        # Critically-damped-ish approach to the target for smooth motion.
        for i in range(len(self.pos)):
            err = self.target[i] - self.pos[i]
            self.vel[i] += (err * 60.0 - self.vel[i] * 12.0) * dt
            self.pos[i] += self.vel[i] * dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.joint_names)
        js.position = [float(p) for p in self.pos]
        js.velocity = list(self.vel)
        self.joint_pub.publish(js)

        arr = Float64MultiArray()
        arr.data = [float(p) for p in self.pos]
        self.state_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)

    # Read the joint names from the robot description, when one is available.
    urdf_path = os.environ.get("ARMWEB_URDF", "/ws/assets/urdf/robot.urdf")
    joint_names = None
    urdf_root = None
    if urdf_path and os.path.isfile(urdf_path):
        try:
            import xml.etree.ElementTree as ET
            urdf_root = ET.parse(urdf_path).getroot()
            joint_names = [j.get("name") for j in urdf_root.findall("joint")]
            print(f"arm_sim: {len(joint_names)} joints from "
                  f"{os.path.basename(urdf_path)}")
        except Exception as exc:
            print(f"arm_sim: could not read {urdf_path}: {exc}")

    node = ArmSim(joint_names, urdf_root)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()