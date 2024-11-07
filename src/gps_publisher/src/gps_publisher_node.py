#!/usr/bin/env python3

import rospy
import csv
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf
import math

# Function to publish GPS data as PoseStamped
def publish_gps_data(file_path):
    # Initialize publisher and node
    pose_pub = rospy.Publisher('gps/pose', PoseStamped, queue_size=10)
    rospy.init_node('gps_publisher_node', anonymous=True)
    rate = rospy.Rate(50)  # Publish at 10 Hz

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
            # print(f"EXTRACTED VALUES:: Timestamp: {timestamp_ms}, Position: ({x}, {y}, {z}), Orientation: ({heading}, {pitch}, {roll})")

            # Convert roll, pitch, yaw (heading) to quaternion
            quaternion = tf.transformations.quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))

            # Create a PoseStamped message
            pose_msg = PoseStamped()
           
            secs = timestamp_ms // 1_000_000
            nsecs = timestamp_ms % 1_000_000
            pose_msg.header.stamp = rospy.Time(secs, nsecs)
            # pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"

            # Set the position
            pose_msg.pose.position = Point(x, y, z)

            # Set the orientation
            pose_msg.pose.orientation = Quaternion(*quaternion)

            # Publish the message
            rospy.loginfo(f"Publishing PoseStamped: Position ({x}, {y}, {z}), Orientation ({quaternion})")
            pose_pub.publish(pose_msg)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('gps_publisher_node', anonymous=True)
        
        # Get the GPS file path from the parameter server
        file_path = rospy.get_param('~gps_file_path', '/media/sf_datasets/ficosa_for_hslam/ficosa_may1/gps_data.csv')
        
        publish_gps_data(file_path)
    except rospy.ROSInterruptException:
        pass
