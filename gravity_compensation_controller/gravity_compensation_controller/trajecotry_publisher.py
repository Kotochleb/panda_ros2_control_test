#!/usr/bin/python3

import itertools

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        self._goals = (            [0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
            [-0.4777696933746337, -0.6226304121455534, -0.3633688343365987, -2.1049883240685117, -0.5680811673667695, 2.397627968284819, 1.3426912733722065, 0.039659056812524796, 0.039659056812524796],
            [-0.378257365716153, 0.18240021813752358, -0.7467764909643876, -2.212217476329841, -0.022873654963241685, 1.8137042983770357, 0.8074611779857013, 0.039659056812524796, 0.039659056812524796],
            [0.3532727640725483, 0.04823498766152486, 0.02204937033883499, -2.5337728387757243, 0.1532298997299845, 2.834343907367076, 0.8131928997048073, 0.03965872526168823, 0.03965872526168823],
            [-0.01786361394448845, -0.16626295289533113, -0.139923212867239, -1.1854629752189312, -0.12228255026408665, 1.1932953288546613, 0.6572838811890955, 0.039659056812524796, 0.039659056812524796],
            [0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
            [-1.4741228487282467, -0.3116446433248576, -0.14510691334168213, -1.9493691723606785, -0.060910826997624505, 1.7138484372145555, 0.7317148729968402, 0.03965872526168823, 0.03965872526168823],
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