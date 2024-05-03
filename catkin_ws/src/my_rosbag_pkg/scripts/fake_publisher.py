#! /usr/bin/env python3

import rospy
from numpy import random
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState


def fake_publisher():
    rospy.init_node("fake_publisher")
    rospy.loginfo("Node fake_publisher has been started. ")

    pub1 = rospy.Publisher("/Roboter_Pose", PoseStamped, queue_size=10)
    pub2 = rospy.Publisher("/Joint_States", JointState, queue_size=10)
    pub3 = rospy.Publisher("/Twist", TwistStamped, queue_size=10)

    rate = rospy.Rate(1)

    msg1 = PoseStamped()
    msg2 = JointState()
    msg3 = TwistStamped()
    while not rospy.is_shutdown():
        msg1.pose.position.x += 1
        pub1.publish(msg1)
        # generate a 1-D Array containing 4 random integers from 0 to 20
        msg2.velocity = random.randint(20, size=(4))
        msg2.effort = random.randint(10, size=(6))
        pub2.publish(msg2)
        msg3.twist.angular.x += 1
        pub3.publish(msg3)

        rate.sleep()


if __name__ == "__main__":
    try:
        print(__file__)
        fake_publisher()
    except rospy.ROSInitException:
        pass
