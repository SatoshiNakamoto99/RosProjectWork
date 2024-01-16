#!/usr/bin/python3
import rospy
from pepper_nodes.srv import WakeUp, WakeUpRequest, WakeUpResponse
from pepper_nodes.srv import Rest, RestRequest, RestResponse
from config import *
class Handler:
    '''
    The constructor creates the service proxy objects, which are able to wake and rest the robot
    '''
    def __init__(self):
        self.wakeup_service = rospy.ServiceProxy("wakeup", WakeUp)
        self.rest_service = rospy.ServiceProxy("rest", Rest)

    '''
    This method wakes up the robot
    '''
    def wakeup(self):
        msg = WakeUpRequest()
        resp = self.wakeup_service(msg)
        rospy.loginfo(resp.ack)

    '''
    This method sets the robot position at rest
    '''
    def rest(self):
        msg = RestRequest()
        resp = self.rest_service(msg)
        rospy.loginfo(resp.ack)
    
    def handle_shutdown(self):
        self.rest()
        

if __name__ == "__main__":
    NODE_NAME = "start_node"
    
    rospy.init_node(NODE_NAME)
    if ON_PEPPER:
        
        handler = Handler()
        #rospy.on_shutdown(handler.handle_shutdown)
        print("[Start Node] Waking up Pepper...")
        handler.wakeup()
        
    else
        print("[Start Node] Running on PC...")
              
    