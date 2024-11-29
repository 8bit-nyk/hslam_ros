#!/usr/bin/env python3

import rospy
import json
import tf
import numpy as np
from tf.transformations import quaternion_from_matrix
import threading

def publish_transform_from_json(json_file, parent_frame, child_frame):
    # Load transformation data from JSON file
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Extract the transformation matrix
    transformation_matrix = np.array(data['transformation_matrix'])

    # Extract translation (last column of the first 3 rows)
    translation = transformation_matrix[:3, 3]

    # Extract rotation using the top-left 3x3 portion of the matrix
    rotation_matrix = transformation_matrix[:3, :3]

    # Convert the rotation matrix to a quaternion
    quat = quaternion_from_matrix(np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))

    # Create a transform broadcaster
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(25.0)

    while not rospy.is_shutdown():
        br.sendTransform(
            #(translation[0], translation[1], translation[2]),
            (0,0,0),
            quat,
            rospy.Time.now(),
            child_frame,
            parent_frame
        )
        # rospy.loginfo(f"Publishing transform from {parent_frame} to {child_frame} with translation: {translation} and quaternion: {quat}")

        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('publish_static_transforms')

    # Default file paths to the JSON files
    # default_odom_to_baselink_json = "src/transform_publisher_node/src/No_transformation.json"  # Adjust path as needed
    # default_map_to_odom_json = "src/transform_publisher_node/src/transformation_FICOSARAW-to-HSLAM.json"  # Adjust path as needed

    # Get the file paths from launch arguments or use default paths
    odom_to_baselink_json = rospy.get_param('~odom_to_baselink_json')#, default_odom_to_baselink_json)
    map_to_odom_json = rospy.get_param('~map_to_odom_json',)# default_map_to_odom_json)

    # Start separate threads for each transform
    thread1 = threading.Thread(target=publish_transform_from_json, args=(odom_to_baselink_json, "odom", "base_link"))
    thread2 = threading.Thread(target=publish_transform_from_json, args=(map_to_odom_json, "map", "odom"))

    # Set the threads as daemon so they stop when the main program stops
    thread1.daemon = True
    thread2.daemon = True

    # Start the threads
    thread1.start()
    thread2.start()

    # Keep the main thread alive
    rospy.spin()