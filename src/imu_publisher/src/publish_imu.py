#!/usr/bin/env python3

import rospy
import csv
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import os

def publish_imu_data():
    rospy.init_node('imu_publisher_node', anonymous=True)
    imu_pub = rospy.Publisher('/imu0', Imu, queue_size=10)

    # Load the CSV file
    imu_data_file_path = os.path.expanduser('~/hslam_ros_ws/res/imu0/data.csv')
    imu_data_file = rospy.get_param('~imu_data_file', imu_data_file_path)
    publish_rate = rospy.get_param('~publish_rate', 200)  # Default to 200 Hz

    rospy.loginfo(f"Starting IMU publisher node with data file: {imu_data_file}")
    rospy.loginfo(f"Publishing rate set to {publish_rate} Hz")

    try:
        with open(imu_data_file, 'r') as csvfile:
            imu_data = csv.reader(csvfile, delimiter=',')
            next(imu_data)  # Skip the header row

            rate = rospy.Rate(publish_rate)  # Set the publishing rate
            for row in imu_data:
                if rospy.is_shutdown():
                    break

                imu_msg = Imu()

                # Assuming the CSV columns are: timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
                timestamp_ns = int(row[0])
                secs = timestamp_ns // 1_000_000_000
                nsecs = timestamp_ns % 1_000_000_000
                imu_msg.header.stamp = rospy.Time(secs, nsecs)
                imu_msg.header.frame_id = "imu_link"

                # Assigning acceleration (in m/s^2)
                imu_msg.linear_acceleration.x = float(row[1])
                imu_msg.linear_acceleration.y = float(row[2])
                imu_msg.linear_acceleration.z = float(row[3])

                # Assigning angular velocity (in rad/s)
                imu_msg.angular_velocity.x = float(row[4])
                imu_msg.angular_velocity.y = float(row[5])
                imu_msg.angular_velocity.z = float(row[6])

                # IMU does not provide orientation, so keep orientation zero (or apply some initial orientation)
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation.w = 1.0

                # Publish the message
                imu_pub.publish(imu_msg)
                rospy.loginfo(f"Published IMU data for timestamp: {timestamp_ns}")

                rate.sleep()

        rospy.loginfo("IMU data publishing completed.")

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("IMU publishing interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        rospy.loginfo("Shutting down IMU publisher node.")

if __name__ == '__main__':
    """
    IMU Publisher Node

    This node publishes IMU data from a CSV file to a specified ROS topic.

    Parameters:
    - imu_data_file: Path to the CSV file containing IMU data
    - publish_rate: Rate at which IMU data is published (Hz)

    Usage:
    $ rosrun your_package imu_publisher_node.py _imu_data_file:=/path/to/data.csv _publish_rate:=200
    """
    publish_imu_data()
