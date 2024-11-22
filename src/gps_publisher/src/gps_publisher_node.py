#!/usr/bin/env python3

import rospy
import csv
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, Imu
import tf

def publish_gps_data(file_path):
    # Initialize publishers
    pose_pub = rospy.Publisher('/gps/pose', PoseWithCovarianceStamped, queue_size=10)
    path_pub = rospy.Publisher('/gps/path', Path, queue_size=10)
    navsat_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
    imu_pub = rospy.Publisher('/gps/imu', Imu, queue_size=10)
    
    rospy.init_node('gps_publisher_node', anonymous=True)
    rate = rospy.Rate(20)  # Publish at 10 Hz

    # Create a Path message
    path_msg = Path()
    path_msg.header.frame_id = "map"

    with open(file_path, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        next(csv_reader)  # Skip header

        for row in csv_reader:
            if rospy.is_shutdown():
                break

            # Extract data from the CSV
            latitude = float(row[4])  # Adjust based on column indices
            longitude = float(row[5])
            altitude = float(row[6])
            heading = float(row[-3])  # Earth-referenced heading
            pitch = float(row[-2])
            roll = float(row[-1])
            x, y, z = float(row[-6]), float(row[-5]), float(row[-4])  # XYZ positions

            # Convert heading to quaternion
            quaternion = tf.transformations.quaternion_from_euler(
                math.radians(roll), math.radians(pitch), math.radians(heading)
            )

            # Publish NavSatFix message
            navsat_msg = NavSatFix()
            navsat_msg.header.stamp = rospy.Time.now()
            navsat_msg.header.frame_id = "map"
            navsat_msg.latitude = latitude
            navsat_msg.longitude = longitude
            navsat_msg.altitude = altitude
            navsat_msg.position_covariance = [1e-9] * 9
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            navsat_pub.publish(navsat_msg)

            # Publish Imu message with heading
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"
            imu_msg.orientation = Quaternion(*quaternion)
            imu_msg.orientation_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
            imu_pub.publish(imu_msg)

            # Publish PoseWithCovarianceStamped message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose.position = Point(x, y, z)
            pose_msg.pose.pose.orientation = Quaternion(*quaternion)
            pose_msg.pose.covariance = [0.5, 0, 0, 0, 0, 0,
                                        0, 0.5, 0, 0, 0, 0,
                                        0, 0, 1e-9, 0, 0, 0,
                                        0, 0, 0, 1e-9, 0, 0,
                                        0, 0, 0, 0, 1e-9, 0,
                                        0, 0, 0, 0, 0, 1e-9]
            pose_pub.publish(pose_msg)

            # Update Path message
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp = pose_msg.header.stamp
            pose_stamped_msg.header.frame_id = "map"
            pose_stamped_msg.pose = pose_msg.pose.pose
            path_msg.poses.append(pose_stamped_msg)
            path_pub.publish(path_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('gps_publisher_node', anonymous=True)
        file_path = rospy.get_param('~gps_file_path', '/media/sf_datasets/ficosa_for_hslam/ficosa_may1/gps_data.csv')
        publish_gps_data(file_path)
    except rospy.ROSInterruptException:
        pass
