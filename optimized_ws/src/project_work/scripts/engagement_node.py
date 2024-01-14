#!/usr/bin/env python3
from config import *
from base_node import BaseNode
import rospy
from std_msgs.msg import  Bool, String


class Engagement_Node(BaseNode):
    def __init__(self, name_node, output_topic, verbose):
        super().__init__()
        self._name_node = name_node
        self._verbose = verbose
        self._human_presence = False
        self._previus_human_presence = False
        self._engagement = False
        self._output_topic = output_topic
        self._pub = rospy.Publisher(self._output_topic, String, queue_size=0)
    
    def _get_human_presence(self):
        human_presence = self._human_presence
        return human_presence
    
    def _set_human_presence(self, human_presence):
        self._human_presence = human_presence
        
    def _get_previus_human_presence(self):
        previus_human_presence = self._previus_human_presence
        return previus_human_presence
    
    def _set_previus_human_presence(self, previus_human_presence):
        self._previus_human_presence = previus_human_presence
    
    def _get_engagement(self):
        engagement = self._engagement
        return engagement
    
    def _set_engagement(self, engagement):
        self._engagement = engagement
        
    def _handle_transition_state(self, presence):
        
        self._set_previus_human_presence(self._get_human_presence())
        self._set_human_presence(presence.data)
        #
        #0. ________________________________________________________
        # Individuate the transition state from S0 to S1
        if not self._get_previus_human_presence() and self._get_human_presence():
            self._set_engagement(True)
        
        #1. ________________________________________________________
        if self._human_presence:
            #
            #1.1 ________________________________________________________
            # State S1
            if self._verbose:
                print('[Engagement]  Human presence is detected')
        else:
            #
            #1.2 ________________________________________________________
            # State S0
            if self._verbose:
                print('[Engagement]  Human presence is not detected')
            #self._set_engagement(False)
    
    def start(self, rate_value=10):
        rospy.init_node(self._name_node, anonymous=True)
        rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_transition_state)
        
        
        rospy.on_shutdown(self._handle_shutdown)
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self._get_engagement():
                continue
            #self._set_engagement(False)
            if self._verbose:
                print('[Engagement]  Engagement is detected')
            # Publish the engagement string
            self._pub.publish("Hello, I am Pepper, your personal assistant in the Shopping Mall")    
            
            self._set_engagement(False)

if __name__ == '__main__':
    try:
        node = Engagement_Node('engagement_node', CHATBOT_OUTPUT_TOPIC ,verbose=True)
        node.start()
    except rospy.ROSInterruptException:
        pass