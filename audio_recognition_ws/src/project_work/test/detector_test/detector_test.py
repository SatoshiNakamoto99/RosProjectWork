#!/usr/bin/python3
import os

import rospy 
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from threading import Lock
import ros_numpy
import cv2

VIDEO_TOPIC = '/in_rgb'
HUMAN_PRESENCE_TOPIC = '/track/human_presence'
TEST_PATH = os.path.dirname(os.path.abspath(__file__))


class DetectorTest:
    """
    Node implementing the test of detector module
    """

    def __init__(self):
        """
        Initializes the detection module testing node.
        """
        #super().__init__()
        self._publisher = None
        self._output = list()
        self._groundtruth = None
    
    
    def _setup(self,scene_path):
        """
        Load groundtruth from file stored in specific scene_path.
        Args:
            scene_path (String)
        """
        with open(os.path.join(scene_path,"groundtruth.txt"),"r") as f:
            g = f.read()
        labels = g.split(",")
        # to manage test with only images without faces
        if len(labels)==1 and labels[0]=="False":
            self._groundtruth = list() 
            return
        self._groundtruth = [x=="True" for x in labels]

    def _cleanup(self):
        """
        Reset groundtruth and detector output
        """
        self._groundtruth = None
        self._output = list()
        
    
    
    def _test(self):
        """
        Test the correctness of the detector output
        """
        # check if the sequece 
        #se Gt è una stringa vuota  stampa Riempi la GT 
        if len(self._groundtruth) == 0:
            print("Fill the groundtruth")
            return
        if len(self._output) == 0:
            print("Fill the output")
            return
        # check if the sequence of output is the same of groundtruth
        if self._output == self._groundtruth:
            print("Passed")
        else:
            print("Failed")
        
   
    def __human_presence(self,is_presence):
        """
        Callback function for topic track/human_presence.
        Args:
            is_presence (std_msgs/Bool): is true if a person is detected, otherwise false.  
        """
        self._output.append(is_presence.data)
  
    def __test_case(self,test_case_folder):
        """
        Run test on data stoerd in specified folder
        Args:
            test_case_folder (String): folder where are stored data to test
        """
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)
 
        # test scene
        scene_path = os.path.join(test_case_path,"scene")
        frames = os.listdir(scene_path)
        frames.sort()
        rate = rospy.Rate(10)
        for frame in frames:
            img = cv2.imread(os.path.join(scene_path,frame))
            msg = ros_numpy.msgify(Image, img, encoding = "bgr8")
            self._publisher.publish(msg)
            rate.sleep()
        self._test()
        self._cleanup()
        
    def start(self):
        """
        Starts the testing process. Initializes the ROS node, subscribes to the human detection topic,
        publishes images to the topic, and waits for the ROS node to terminate.
        """
        rospy.init_node('detector_test', anonymous=True)
        rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self.__human_presence)
        self._publisher = rospy.Publisher(VIDEO_TOPIC, Image, queue_size = 10)

        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        for test in test_cases:
            if (test!="__pycache__" and test!="detector_test.py"):
                if not os.path.isfile(os.path.join(TEST_PATH,test)):
                    self.__test_case(test)
                    
        print("TEST FINISHED")
        print("Type CTRL+C to exit")
        while not rospy.is_shutdown():
            pass

if __name__ == "__main__":
    try:
        d = DetectorTest()
        d.start()
        
    except rospy.ROSInterruptException:
        pass







        
