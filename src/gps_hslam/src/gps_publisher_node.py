#!/usr/bin/env python3

import rospy
import csv
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
import tf

class GPSPublisherNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('gps_publisher_node', anonymous=False)

        # Retrieve ROS parameters
        self.gps_file_path = rospy.get_param('~gps_file_path', '/media/sf_datasets/ficosa_for_hslam/ficosa_may1/gps_data.csv')
        self.hslam_pose_topic = rospy.get_param('~hslam_pose_topic', '/hslam/pose')
        self.gps_pose_topic = rospy.get_param('~gps_pose_topic', '/gps/pose')
        self.gps_path_topic = rospy.get_param('~gps_path_topic', '/gps/path')

        # Initialize publishers
        self.pose_pub = rospy.Publisher(self.gps_pose_topic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(self.gps_path_topic, Path, queue_size=10)

        # Initialize subscriber
        rospy.Subscriber(self.hslam_pose_topic, PoseStamped, self.hslam_callback)

        # Initialize GPS data
        self.gps_data = self.load_gps_data(self.gps_file_path)
        self.current_index = 0
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

    def load_gps_data(self, file_path):
        gps_data = []
        with open(file_path, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            next(csv_reader)  # Skip header
            for row in csv_reader:
                timestamp = int(row[-7])
                x, y, z = float(row[-6]), float(row[-5]), float(row[-4])
                roll = float(row[-3])
                pitch = float(row[-2])
                yaw = float(row[-1])
                gps_data.append((timestamp, x, y, z, roll, pitch, yaw))
        return gps_data

    def find_closest_gps_entry(self, hslam_timestamp):
        tolerance = rospy.Duration(0.1)  # 100 ms tolerance
        for entry in self.gps_data[self.current_index:]:
            gps_timestamp = rospy.Time(entry[0] // 1_000_000, (entry[0] % 1_000_000) * 1_000)
            if abs(hslam_timestamp - gps_timestamp) <= tolerance:
                return entry
        return None

    def hslam_callback(self, hslam_pose):
        hslam_timestamp = hslam_pose.header.stamp

        closest_gps_entry = self.find_closest_gps_entry(hslam_timestamp)
        if closest_gps_entry is None:
            rospy.logwarn("No matching GPS entry found for timestamp: %s", hslam_timestamp.to_sec())
            return

        timestamp, x, y, z, roll, pitch, yaw = closest_gps_entry
        secs = timestamp // 1_000_000
        nsecs = (timestamp % 1_000_000) * 1_000

        quaternion = tf.transformations.quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time(secs, nsecs)
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position = Point(x, y, z)
        pose_msg.pose.orientation = Quaternion(*quaternion)

        self.pose_pub.publish(pose_msg)

        self.path_msg.poses.append(pose_msg)
        self.path_pub.publish(self.path_msg)

    def spin(self):
        rospy.loginfo("GPS Publisher Node started.")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GPSPublisherNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

