#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmTestPublisher(Node):
    def __init__(self):
        super().__init__('arm_test_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.send_command)
        self.sent_once = False

    def send_command(self):
        if self.sent_once:
            return

        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.2, -0.5, 0.8, -0.3, 0.0, 0.02]
        point.time_from_start.sec = 2
        traj.points.append(point)

        self.publisher_.publish(traj)
        self.get_logger().info('Sent test trajectory!')
        self.sent_once = True


def main(args=None):
    rclpy.init(args=args)
    node = ArmTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
