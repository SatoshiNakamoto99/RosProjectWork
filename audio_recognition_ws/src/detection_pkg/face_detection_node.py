#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
import ros_numpy

class FaceDetectionNode(object):
    def __init__(self):
        self.faceProto = "/home/satoshinakamoto/CognitiveRobotics/Audio_Recognition_Ros/audio_recognition_ws/src/detection_pkg/opencv_face_detector.pbtxt"
        self.faceModel = "/home/satoshinakamoto/CognitiveRobotics/Audio_Recognition_Ros/audio_recognition_ws/src/detection_pkg/opencv_face_detector_uint8.pb"
        self.faceNet = cv2.dnn.readNet(self.faceModel, self.faceProto)
        
        
    def getFaceBox( self, frame, conf_threshold=0.8):
        frameOpencvDnn = frame.copy()
        frameHeight = frameOpencvDnn.shape[0]
        frameWidth = frameOpencvDnn.shape[1]
        blob = cv2.dnn.blobFromImage(frameOpencvDnn, 1.0, (300, 300), [104, 117, 123], True, False)

        self.faceNet.setInput(blob)
        detections = self.faceNet.forward()
        bboxes = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > conf_threshold and detections[0, 0, i, 5]<1 and detections[0, 0, i, 6]<1:
                x1 = int(detections[0, 0, i, 3] * frameWidth)
                y1 = int(detections[0, 0, i, 4] * frameHeight)
                x2 = int(detections[0, 0, i, 5] * frameWidth)
                y2 = int(detections[0, 0, i, 6] * frameHeight)
                bboxes.append([x1, y1, x2, y2])
                #cv2.rectangle(frameOpencvDnn, (x1, y1), (x2, y2), (0, 255, 0), int(round(frameHeight/300)), 8)
        return  bboxes, frameWidth, frameHeight
    
    def getBBoxMaxArea(self, bboxes):
        max_area = 0
        max_bbox = []
        for bbox in bboxes:
            area = (bbox[2]-bbox[0])*(bbox[3]-bbox[1])
            if area>max_area:
                max_area = area
                bbox_max = bbox
                max_bbox = bbox
        return max_bbox
    
    def detect(self, image):
        cv_image = ros_numpy.numpify(image)
        # get the face box
        bboxes, width, height = self.getFaceBox(cv_image)
        bbox = self.getBBoxMaxArea(bboxes)
        # publish the face box and the width and height of the image
        if len(bbox)!=0:
            msg = Int32MultiArray(data = bbox)
            
            self.pub_motion.publish(msg)
            # loginfo
            rospy.loginfo(f"Face position: {bbox}")
        
        
        
        
        
    
    
    def start(self):
        # Mi devo iscrivere al topic /camera/image_raw
        rospy.init_node("face_detection_node")
        rospy.loginfo("Face detection node started")
        self.pub_motion = rospy.Publisher("/face_detection_node/face_position", Int32MultiArray, queue_size=0)
        #self.pub_engage = rospy.Publisher("/face_detection_node/engage", Float32MultiArray, queue_size=10)
        rospy.Subscriber("in_rgb", Image, self.detect)
        
        rospy.spin()
        
        


if __name__ == "__main__":
    
    try:
        node = FaceDetectionNode()
        node.start()    
    except rospy.ROSInterruptException:
        pass