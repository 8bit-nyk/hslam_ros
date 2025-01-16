#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

def publish_fake_pose():
    pub = rospy.Publisher('/simple_pose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('fake_pose_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    x, y, theta = 0.0, 0.0, 0.0  # Initial pose
    velocity = 0.1  # Linear velocity (m/s)
    angular_velocity = 0.05  # Angular velocity (rad/s)

    while not rospy.is_shutdown():
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"

        # Update position and orientation
        x += velocity * np.cos(theta) * 0.1  # Assuming a rate of 10 Hz, hence dt = 0.1
        y += velocity * np.sin(theta) * 0.1
        theta += angular_velocity * 0.1

        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion for orientation
        q = np.array([0, 0, np.sin(theta / 2), np.cos(theta / 2)])  # [x, y, z, w]
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        # Add covariance (keep it small but non-zero for testing)
        pose_msg.pose.covariance = [1, 0, 0, 0, 0, 0,
                                    0, 0.2, 0, 0, 0, 0,
                                    0, 0, 1e-9, 0, 0, 0,
                                    0, 0, 0, 1e-9, 0, 0,
                                    0, 0, 0, 0, 1e-9, 0,
                                    0, 0, 0, 0, 0, 1e-9]

        pub.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_pose()
    except rospy.ROSInterruptException:
        pass
