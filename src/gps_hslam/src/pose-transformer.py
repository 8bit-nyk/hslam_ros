#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from collections import deque

class GPSTransformationNode:
    def __init__(self):
        rospy.init_node('gps_transformer_node', log_level=rospy.DEBUG) 
        # Parameters
        self.gps_pose_topic = rospy.get_param("~gps_pose_topic", "/gps/pose")
        self.hslam_pose_topic = rospy.get_param("~hslam_pose_topic", "/hslam/pose")
        self.transformed_gps_pose_topic = rospy.get_param("~transformed_gps_pose_topic", "/transformed_gps_pose")
        self.gps_frame = rospy.get_param("~gps_frame", "map")
        self.slam_frame = rospy.get_param("~slam_frame", "odom")
        self.scale_tolerance = rospy.get_param("~scale_tolerance", 0.01)

        # Publishers
        self.transformed_gps_pose_pub = rospy.Publisher(self.transformed_gps_pose_topic, PoseStamped, queue_size=10)

        # Subscribers for Synchronization
        self.gps_sub = Subscriber(self.gps_pose_topic, PoseStamped)
        self.hslam_sub = Subscriber(self.hslam_pose_topic, PoseStamped)
        self.sync = ApproximateTimeSynchronizer([self.gps_sub, self.hslam_sub], queue_size=100, slop=0.1)
        self.sync.registerCallback(self.sync_callback)

        # Subscriber for all GPS transformations
        self.gps_transform_sub = rospy.Subscriber(self.gps_pose_topic, PoseStamped, self.gps_callback)
        self.hslam_pose_sub = rospy.Subscriber(self.hslam_pose_topic, PoseStamped, self.hslam_callback)

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Internal State
        self.estimated_scale = 1.0
        self.scale_stabilized = False
        self.stable_scale_values = []
        self.stable_scale_values = deque(maxlen=20)  # Sliding window
        self.stabilization_count = 0  # Counter for consecutive stable readings
        self.required_stabilization_count = 10  # Number of stable readings to trigger flag
        self.hslam_pose = None


    def sync_callback(self, gps_pose, hslam_pose):
        rospy.logwarn("Received synchronized GPS and HSLAM poses.")
        # rospy.logwarn(f"GPS Pose: {gps_pose.pose.orientation}")
        # rospy.logwarn(f"HSLAM Pose: {hslam_pose.pose.orientation}")
        if not self.scale_stabilized:
            self.estimate_scale(gps_pose, hslam_pose)
        #self.transform_and_publish(gps_pose, hslam_pose)
        else:
            return


    def hslam_callback(self, hslam_pose):
        self.hslam_pose = hslam_pose

    def gps_callback(self, gps_pose):
        if self.hslam_pose is None:
            rospy.logwarn("HSLAM pose not yet received. Skipping transformation.")
            return

        self.transform_and_publish(gps_pose, self.hslam_pose)

    def estimate_scale(self, gps_pose, hslam_pose):
        try:
            gps_position = np.array([
                gps_pose.pose.position.x,
                gps_pose.pose.position.y,
                gps_pose.pose.position.z
            ])

            hslam_position = np.array([
                hslam_pose.pose.position.x,
                hslam_pose.pose.position.y,
                hslam_pose.pose.position.z
            ])

            hslam_distance = np.linalg.norm(hslam_position)
            gps_distance = np.linalg.norm(gps_position)

            if hslam_distance > 1:  # Avoid division by zero
                scale = gps_distance / hslam_distance
                self.stable_scale_values.append(scale)

                rospy.logwarn(f"Scale calculated: {scale}")
                
                if len(self.stable_scale_values) == self.stable_scale_values.maxlen:
                    # Calculate stabilization metrics
                    scale_variation = np.std(self.stable_scale_values)
                    avg_scale = np.mean(self.stable_scale_values)
                    
                    rospy.logwarn(f"Current average scale: {avg_scale}")
                    rospy.logwarn(f"Scale variation: {scale_variation}")

                    if scale_variation < self.scale_tolerance:
                        self.stabilization_count += 1
                        rospy.logwarn(f"Stabilization count: {self.stabilization_count}")
                        
                        if self.stabilization_count >= self.required_stabilization_count:
                            self.scale_stabilized = True
                            self.estimated_scale = avg_scale
                            rospy.logwarn(f"Scale stabilized at {self.estimated_scale}. Stopping estimation.")
                    else:
                        # Reset if variation exceeds tolerance
                        rospy.logwarn("Scale variation too high, resetting stabilization.")
                        self.stabilization_count = 0
                else:
                    rospy.logwarn("Insufficient data for stabilization check.")
            else:
                rospy.logwarn("HSLAM distance is too small to estimate scale.")

        except Exception as e:
            rospy.logerr(f"Error estimating scale: {e}")

    def transform_and_publish(self, gps_pose, hslam_pose):
        rospy.logwarn(f"Scale from GPS Transformer: {self.estimated_scale}")
        if not self.scale_stabilized:
            rospy.logwarn("Scale not stabilized yet. Skipping transformation.")
            return

        try:
            transform = self.tf_buffer.lookup_transform(self.slam_frame, self.gps_frame, rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(gps_pose, transform)
            scaled_position = [
                transformed_pose.pose.position.x / self.estimated_scale,
                transformed_pose.pose.position.y / self.estimated_scale,
                transformed_pose.pose.position.z / self.estimated_scale
            ]

            transformed_pose.pose.position.x = scaled_position[0]
            transformed_pose.pose.position.y = scaled_position[1]
            transformed_pose.pose.position.z = scaled_position[2]
            

            transformed_gps_pose_msg = PoseStamped()
            transformed_gps_pose_msg.header.stamp = hslam_pose.header.stamp
            transformed_gps_pose_msg.header.frame_id = self.slam_frame
            transformed_gps_pose_msg.pose = transformed_pose.pose
            transformed_gps_pose_msg.pose.orientation =  hslam_pose.pose.orientation

            self.transformed_gps_pose_pub.publish(transformed_gps_pose_msg)

        except Exception as e:
            rospy.logerr(f"Error transforming and publishing GPS pose: {e}")

if __name__ == "__main__":
    try:
 # Enable debug-level logging
        node = GPSTransformationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

