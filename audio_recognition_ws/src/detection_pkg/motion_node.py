#!/usr/bin/env python3

import math
import rospy
from std_msgs.msg import Float32MultiArray, String

import rospy
from std_msgs.msg import Int32MultiArray

class Handler:
    '''
    The constructor subscribes the node to head_rotation topics
    '''
    def __init__(self):
        self.head_motion_pitch_pub = rospy.Publisher("/head_rotation/pitch", Float32MultiArray, queue_size=10)
        self.head_motion_yaw_pub = rospy.Publisher("/head_rotation/yaw", Float32MultiArray, queue_size=10)

    '''
    This method publishes the desired head position relative to yaw angle
    @param angle: Target angle in radians
    @param velocity: Movement speed
    '''
    def move_head_yaw(self, angle, velocity=0.2):
        msg = Float32MultiArray()
        msg.data = [angle, velocity]
        rospy.loginfo(f"Moving head relative to yaw of {angle} radians")
        while self.head_motion_yaw_pub.get_num_connections() <= 0:
            rospy.sleep(0.2)
        self.head_motion_yaw_pub.publish(msg)

    '''
    This method publishes the desired head position relative to pitch
    @param angle: Target angle in radians
    @velocity: Movement speed
    '''
    def move_head_pitch(self, angle, velocity=0.2):
        msg = Float32MultiArray()
        msg.data = [angle, velocity]
        rospy.loginfo(f"Moving head relative to yaw of {angle} radians")
        while self.head_motion_pitch_pub.get_num_connections() <= 0:
            rospy.sleep(0.2)
        self.head_motion_pitch_pub.publish(msg)

class PepperMotionNode(object, ):
    def __init__(self, distance, face_dimension, VELOCITY=0.15, SLEEP_TIME=3.0):
        self.distance = distance
        self.face_dimension= face_dimension #[width_cm, height_cm]
        self.relative_x_center = 0.0
        self.relative_y_center = 0.0
        self.heandler = Handler()
        self.VELOCITY = VELOCITY
        self.SLEEP_TIME = SLEEP_TIME
    
    
    def motion(self, msg):
        # message is an bboxes, width, height = self.getFaceBox(self.faceNet,cv_image)
        # publish the face box and the width and height of the image
        #msg = Float32MultiArray()
        #msg.data = [bboxes, width, height]
        # from msg I have to extract the face position of BBox in ppx and ppy and I want stimate the pixel for cm 
        # compute ppx for the face 
        # compute ppy for the face
        bbox = msg.data
        
        #width = msg.data[1]
        #height = msg.data[2]
        width_face = bbox[2]-bbox[0]
        height_face = bbox[3]-bbox[1]
        
        # compute the cm mapped with a pixel
        cm_x = self.face_dimension[0]/width_face
        cm_y = self.face_dimension[1]/height_face
        cm_pp = (cm_x+cm_y)/2
        
        # compute the center of the face
        ppx_face = (bbox[2]+bbox[0])/2
        ppy_face = (bbox[3]+bbox[1])/2
        
        # compute the x and y distance from the relative center
        x_cateto = (ppx_face - self.relative_x_center)*cm_pp    
        y_cateto = (ppy_face - self.relative_y_center)*cm_pp
        
        # compute the angle of inclination of the face respect to the robot in rad
        angle_yaw = math.atan(x_cateto/self.distance)
        angle_pitch = math.atan(y_cateto/self.distance)
        rospy.loginfo(f"Moving head relative to yaw of {angle_yaw} radians")
        rospy.loginfo(f"Moving head relative to pitch of {angle_pitch} radians")
        # update the relative center
        self.relative_x_center = ppx_face
        self.relative_y_center = ppy_face
        # normalize the angle beetwen 75 and -75 in yaw and pitch rad
        if angle_yaw > 1:
            angle_yaw = 1
        elif angle_yaw < -1:
            angle_yaw = -1
        
        # angle_pitch = 0.5
        # angle_yaw = 0.5
        
        self.heandler.move_head_pitch(angle_pitch, self.VELOCITY)
        rospy.sleep(self.SLEEP_TIME)
        self.heandler.move_head_yaw(angle_yaw, self.VELOCITY)
        rospy.sleep(self.SLEEP_TIME)
        
        
    
    def start(self):
        # Mi devo iscrivere al topic /face_detection_node/face_position
        rospy.init_node("pepper_motion_node")
        rospy.Subscriber("/face_detection_node/face_position", Int32MultiArray, self.motion)
        rospy.spin()


if __name__ == "__main__":
    try:
        node = PepperMotionNode(distance=0.5, face_dimension=[20, 20])
        node.start()    
    except rospy.ROSInterruptException:
        pass