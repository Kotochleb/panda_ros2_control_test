#!/usr/bin/python3

import itertools

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        self._goals = (
            [0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
            [1.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
            [-1.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
        )
        self._goal_cycle = itertools.cycle(self._goals)

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        # -------------------------------
        #   Timers
        # -------------------------------

        # check motor state at 0.5 Hz
        self.timer = self.create_timer(4.0, self._control_loop_cb)
        self.get_logger().info("Node started.")
        self._control_loop_cb()

    def _control_loop_cb(self, *args) -> None:
        jt = JointTrajectory()
        jt.points = [
            JointTrajectoryPoint(positions=[j]) for j in next(self._goal_cycle)
        ]
        self._trajectory_pub.publish(jt)
        self.get_logger().info("New goal published.")


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except:
        pass