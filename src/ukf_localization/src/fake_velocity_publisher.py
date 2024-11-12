#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np

def publish_fake_velocity():
    pub = rospy.Publisher('/simple_velocity', TwistWithCovarianceStamped, queue_size=10)
    rospy.init_node('fake_velocity_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "base_link"

        # Simulate velocity as a sine wave to create more realistic variation
        current_time = rospy.Time.now().to_sec()
        linear_velocity = 0.5 * (1 + np.sin(0.1 * current_time))  # Varies between 0 and 1 m/s
        angular_velocity = 0.1 * np.cos(0.1 * current_time)  # Varies between -0.1 and 0.1 rad/s

        twist_msg.twist.twist.linear.x = linear_velocity
        twist_msg.twist.twist.linear.y = 0.0
        twist_msg.twist.twist.linear.z = 0.0

        twist_msg.twist.twist.angular.z = angular_velocity

        # Add covariance (keep it non-zero)
        twist_msg.twist.covariance = [1, 0, 0, 0, 0, 0,
                                      0, 1, 0, 0, 0, 0,
                                      0, 0, 1e-9, 0, 0, 0,
                                      0, 0, 0, 0.01, 0, 0,
                                      0, 0, 0, 0, 1e-9, 0,
                                      0, 0, 0, 0, 0, 1e-9]

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fake_velocity()
    except rospy.ROSInterruptException:
        pass
