#!/usr/bin/python3

import itertools

import rospy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPublisher:
    def __init__(self, name: str) -> None:
        self._node_name = name
        rospy.init_node(self._node_name, anonymous=False)

        self._goals = (
            [0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
            [1.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
            [-1.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397],
        )
        self._goal_cycle = itertools.cycle(self._goals)

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._trajectory_pub = rospy.Publisher(
            "/joint_trajectory", JointTrajectory, queue_size=10
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        # check motor state at 0.5 Hz
        self._control_loop = rospy.Timer(rospy.Duration(4.0), self._control_loop_cb)

        rospy.loginfo(f"[{rospy.get_name()}] Node started")

    def _control_loop_cb(self, *args) -> None:
        jt = JointTrajectory()
        jt.points = [
            JointTrajectoryPoint(positions=[j]) for j in next(self._goal_cycle)
        ]
        rospy.loginfo(f"[{rospy.get_name()}] New pose published")
        self._trajectory_pub.publish(jt)


def main():
    trajectory_publisher = TrajectoryPublisher("trajectory_publisher")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
