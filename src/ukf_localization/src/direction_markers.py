#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def publish_text_markers():
    rospy.init_node('nsew_text_markers', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    # Directions and their coordinates
    directions = {'N': (0, 1), 'E': (1, 0), 'S': (0, -1), 'W': (-1, 0)}

    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        marker_id = 0
        for label, (x, y) in directions.items():
            marker = Marker()
            marker.header.frame_id = "base_link"  # Change to your RViz Fixed Frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "directions"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x * 5.0  # Scale the positions for visibility
            marker.pose.position.y = y * 5.0
            marker.pose.position.z = 1.0      # Place text above ground
            marker.scale.z = 1.0             # Text size
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0             # Fully visible
            marker.text = label              # "N", "E", "S", or "W"

            marker_pub.publish(marker)
            marker_id += 1

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_text_markers()
    except rospy.ROSInterruptException:
        pass

