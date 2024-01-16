#!/usr/bin/env python3

from config import *
import cv2
import rospy
import ros_numpy
from sensor_msgs.msg import Image


# Acquire the camera
vid = cv2.VideoCapture(0) 


def talker():
    """
    Publishes video frames from a webcam as ROS Image messages.

    This function initializes a ROS node named 'talker' and creates a publisher to publish
    video frames as ROS Image messages. It reads frames from a webcam, converts them into
    ROS Image messages, and publishes them at a specified rate.

    Args:
        None

    Returns:
        None
    """
    pub = rospy.Publisher(VIDEO_TOPIC, Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    # set here the Rate in Hz
    rate = rospy.Rate(VIDEO_FPS)

    while not rospy.is_shutdown():
        # Read a frame from the webcam
        ret, img = vid.read()

        if ret:
            # --------- Transform img in message and publish ------------
            msg = ros_numpy.msgify(Image, img, encoding = "bgr8")
            # rospy.loginfo(msg)
            pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        vid.release()
        pass
