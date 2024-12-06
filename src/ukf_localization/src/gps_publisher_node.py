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
    rate = rospy.Rate(30)  # Publish at 10 Hz

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

            x, y, z = float(row[-6]), float(row[-5]), float(row[-4])  # XYZ positions
            roll = float(row[-3]) # heading in file
            pitch = float(row[-2])  # pitch in file
            yaw = float(row[-1])  #Yaw ,roll in file

            # # Convert from from X-forward Y-right Z-down to Y-forward X-right z-up which should give ENU
            # transformed_roll = pitch  # New roll = old pitch
            # transformed_pitch = -roll  # New pitch = -old roll
            # transformed_yaw = yaw - 90  # Shift yaw for ENU frame (Y-forward)

            # if transformed_yaw < 0:
            #     transformed_yaw += 360  # Wrap around to 0-360°

            # # Convert transformed Euler angles to quaternion
            # quaternion = tf.transformations.quaternion_from_euler(
            #     math.radians(transformed_roll),
            #     math.radians(transformed_pitch),
            #     math.radians(transformed_yaw)
            #     )
            # Correct yaw for ENU frame
            # corrected_yaw = yaw - 90  # Shift from North-referenced to East-referenced
            # if corrected_yaw < 0:
            #     corrected_yaw += 360  # Wrap around to 0-360° if negative

            # Convert corrected yaw, pitch, and roll to quaternion
            quaternion = tf.transformations.quaternion_from_euler(
                math.radians(roll), math.radians(pitch), math.radians(yaw)#corrected_yaw
            )
       
            # Publish NavSatFix message
            navsat_msg = NavSatFix()
            navsat_msg.header.stamp = rospy.Time.now()
            navsat_msg.header.frame_id = "map"
            navsat_msg.latitude = latitude
            navsat_msg.longitude = longitude
            navsat_msg.altitude = altitude
            navsat_msg.position_covariance = [0.02, 0, 0,
                                            0, 0.02, 0,
                                            0, 0, 0.02]
            # navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            navsat_pub.publish(navsat_msg)

            # Publish Imu message with yaw
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "base_link"
            # imu_msg.child_frame_id = "base_link"
            imu_msg.orientation = Quaternion(*quaternion)
            imu_msg.orientation_covariance = [0.03, 0, 0,
                                            0, 0.03, 0, 
                                            0, 0, 0.1] # From datasheet of IMU xnav650
            imu_pub.publish(imu_msg)

            # Publish PoseWithCovarianceStamped message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose.position = Point(x, y, z)
            pose_msg.pose.pose.orientation = Quaternion(*quaternion)
            pose_msg.pose.covariance = [0.2, 0, 0, 0, 0, 0,
                                        0, 0.2, 0, 0, 0, 0,
                                        0, 0, 0.2, 0, 0, 0,
                                        0, 0, 0, 0.5, 0, 0,
                                        0, 0, 0, 0, 0.5, 0,
                                        0, 0, 0, 0, 0, 0.5]
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
