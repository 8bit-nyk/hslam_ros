#!/usr/bin/env python3

import rospy
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion
from geometry_msgs.msg import TwistWithCovariance
import tf
import math

# Function to publish GPS data as Odometry
def publish_gps_data(file_path):
    # Initialize publisher and node
    odom_pub = rospy.Publisher('gps/odom', Odometry, queue_size=10)
    rospy.init_node('gps_publisher_node', anonymous=True)
    rate = rospy.Rate(10)  # Publish at 10 Hz

    # Open the CSV file
    with open(file_path, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        next(csv_reader)  # Skip header

        for row in csv_reader:
            if rospy.is_shutdown():
                break

            # Extract position and orientation data from the last seven columns
            timestamp_ms = int(row[-7])
            x = float(row[-6])
            y = float(row[-5])
            z = float(row[-4])
            heading = float(row[-3])
            pitch = float(row[-2])
            roll = float(row[-1])

            # Convert roll, pitch, yaw (heading) to quaternion
            quaternion = tf.transformations.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))

            # Create an Odometry message
            odom_msg = Odometry()
           
            secs = timestamp_ms // 1_000_000
            nsecs = timestamp_ms % 1_000_000
            odom_msg.header.stamp = rospy.Time(secs, nsecs)
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

            # Set the position
            odom_msg.pose.pose.position = Point(x, y, z)

            # Set the orientation
            odom_msg.pose.pose.orientation = Quaternion(*quaternion)

            # Velocity is unknown, set to zero
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0

            # Publish the message
            rospy.loginfo(f"Publishing Odometry: Position ({x}, {y}, {z}), Orientation ({quaternion})")
            odom_pub.publish(odom_msg)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('gps_publisher_node', anonymous=True)
        
        # Get the GPS file path from the parameter server
        file_path = rospy.get_param('~gps_file_path', '/media/sf_datasets/ficosa_for_hslam/ficosa_may1/gps_data.csv')
        
        publish_gps_data(file_path)
    except rospy.ROSInterruptException:
        pass
