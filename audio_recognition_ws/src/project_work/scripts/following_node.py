#!/usr/bin/python3
import rospy
from pepper_nodes.srv import startFollowing,stopFollowing
from config import *
from std_msgs.msg import Bool
from base_service import BaseService

class FollowingNode(BaseService):
    def __init__(self, verbose=True)-> None:
        super().__init__()
        self._engagement = False
        self._reset = False
        self._previus_human_presence = False
        self._human_presence = False
        self._verbose = verbose
    
    
    
    def _handle_presence(self, presence):
        """
        Callback function for the human presence topic.
        
        Sets the human presence flag to the value written on the topic.
        
        Human presence is set to True if the value is True, False otherwise.
        
        Args:
            presence (Bool): The human presence is written on topic. It is True if human is present, False otherwise.
        """
        self._previus_human_presence = self._human_presence
        self._human_presence = presence.data
        
        if not self._previus_human_presence and self._human_presence:
            self._engagement = True
            self._reset = False
        
        if  self._previus_human_presence and not self._human_presence:
            self._reset = True
    
    def start(self, human_presence_topic, rate_value = 1):
        rospy.init_node("Head_Following_Node")
        rospy.Subscriber(human_presence_topic, Bool, self._handle_presence)
        if ON_PEPPER:
            self._persistence_service_init("startFollowing", startFollowing)
            self._persistence_service_init("stopFollowing", stopFollowing)
            #rospy.on_shutdown(self._persistence_service_call("stopFollowing"))
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            rate.sleep()
            
            if self._engagement:
                if ON_PEPPER:
                    self._persistence_service_call("startFollowing")
                else:
                    print("start following")
                self._engagement = False
            elif self._reset:
                if ON_PEPPER:
                    self._persistence_service_call("stopFollowing")
                else:
                    print("stop following")
                self._reset = False

if __name__ == "__main__":
    
        
    try:
        
        following_node = FollowingNode()
        following_node.start(HUMAN_PRESENCE_TOPIC)
    except rospy.ROSInterruptException:
        pass
    #rospy.on_shutdown(handler.handle_shutdown)
    #print("[Start Node] Waking up Pepper...")
        
        
    
        

