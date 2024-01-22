#!/usr/bin/python3
import rospy
from pepper_nodes.srv import WakeUp
from config import *
from base_service import BaseService


class PepperStart:
    def __init__(self,verbose=True):
        """
        Initialize a Pepper Start node
        
        Args:
            verbose (bool): Flag indicating whether to enable verbose mode.
        """
        super().__init__()
        self._verbose = verbose
    
    def start(self):
        """
        Start the node.
        
        """
        rospy.init_node("pepper_start")
        rospy.wait_for_service("wakeup")
        wakeup_service = rospy.ServiceProxy("wakeup", WakeUp)
        wakeup_service.call()
        

if __name__ == "__main__":
    if ON_PEPPER:
        print("Pepper Start... Wake Up!")
        pepper_start = PepperStart()
        pepper_start.start()
    else:
        print("Simulation on PC!")