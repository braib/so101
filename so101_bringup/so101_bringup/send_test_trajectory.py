#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectorySender(Node):
    def __init__(self):
        super().__init__("trajectory_sender")

        # MUST match your YAML + URDF joint names exactly
        self.joints = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper"
        ]

        # Publisher to your JointTrajectoryController
        self.pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",   # topic name
            10
        )

        # Timer to send trajectory periodically
        self.timer = self.create_timer(5.0, self.send_trajectory)
        self.toggle = False

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = self.joints

        point = JointTrajectoryPoint()

        if self.toggle:
            point.positions = [0.0, -0.5, 1.0, -0.5, 0.0, 0.3]
        else:
            point.positions = [0.4, -0.2, 0.6,  0.2, -0.5, 0.0]

        point.time_from_start.sec = 3
        traj.points.append(point)

        self.get_logger().info(f"Publishing trajectory: {point.positions}")
        self.pub.publish(traj)

        self.toggle = not self.toggle


def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
