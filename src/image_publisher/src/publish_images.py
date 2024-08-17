#!/usr/bin/env python3

import rospy
import cv2
import csv
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header

def publish_images():
    rospy.init_node('image_publisher_node', anonymous=True)
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    img_dir_path = os.path.expanduser('~/hslam_ros_ws/res/cam0/data')
    csv_file_path = os.path.expanduser('~/hslam_ros_ws/res/cam0/data.csv')
    # Path to the image directory and CSV file
    image_dir = rospy.get_param('~image_dir', img_dir_path)
    csv_file = rospy.get_param('~csv_file', csv_file_path)
    publish_rate = rospy.get_param('~publish_rate', 20)

    rospy.loginfo(f"Starting image publisher node with image directory: {image_dir} and CSV file: {csv_file}")
    rospy.loginfo(f"Publishing rate set to {publish_rate} Hz")

    try:
        with open(csv_file, 'r') as csvfile:
            image_data = csv.reader(csvfile)
            next(image_data)  # Skip the header row if there is one

            rate = rospy.Rate(publish_rate)  # Set the publishing rate
            for row in image_data:
                if rospy.is_shutdown():
                    break

                timestamp_ns = int(row[0])
                image_filename = row[0] + '.png'
                image_path = os.path.join(image_dir, image_filename)

                if os.path.exists(image_path):
                    # Load the image
                    cv_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

                    if cv_image is None:
                        rospy.logwarn(f"Failed to load image: {image_path}. Skipping...")
                        continue

                    # Convert the image to a ROS Image message
                    header = Header()
                    # Assuming the timestamp is in nanoseconds:
                    secs = timestamp_ns // 1_000_000_000
                    nsecs = timestamp_ns % 1_000_000_000

                    header.stamp = rospy.Time(secs, nsecs)
                    header.frame_id = "camera"
                    ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
                    ros_image.header = header

                    # Publish the image
                    image_pub.publish(ros_image)
                    rospy.loginfo(f"Published image: {image_filename}")

                else:
                    rospy.logwarn(f"Image not found: {image_path}")

                rate.sleep()

        rospy.loginfo("Image publishing completed.")

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Image publishing interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        rospy.loginfo("Shutting down image publisher node.")

if __name__ == '__main__':
    """
    Image Publisher Node

    This node publishes images from a directory to a specified ROS topic.

    Parameters:
    - image_dir: Path to the directory containing images
    - csv_file: Path to the CSV file containing timestamps
    - publish_rate: Rate at which images are published (Hz)

    Usage:
    $ rosrun your_package image_publisher_node.py _image_dir:=/path/to/images _csv_file:=/path/to/csv _publish_rate:=10
    """
    publish_images()
