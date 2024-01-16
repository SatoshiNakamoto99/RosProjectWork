#!/usr/bin/env python3
from config import *
from base_node import BaseNode
import rospy
from std_msgs.msg import  Bool, String


class Engagement_Node(BaseNode):
    def __init__(self, name_node, output_topic, reset_topic, verbose):
        """
        Initializes the EngagementNode class.

        Args:
            name_node (str): The name of the node.
            output_topic (str): The topic to publish output messages.
            reset_topic (str): The topic to publish reset messages.
            verbose (bool): Flag indicating whether to enable verbose mode.

        Returns:
            None
        """
        super().__init__()
        self._name_node = name_node
        self._verbose = verbose
        self._human_presence = False
        self._previus_human_presence = False
        self._engagement = False
        self._output_topic = output_topic
        self._pub = rospy.Publisher(self._output_topic, String, queue_size=0)
        self._reset_topic = reset_topic
        self._pub_reset = rospy.Publisher(self._reset_topic, String, queue_size=0)
        self._reset = False
    
       
    def _handle_transition_state(self, presence):
        """
        Callback function for the human presence topic.
        
        Sets the human presence flag to the value written on the topic.
        
        Store the previous human presence value. 
        
        If there is a transistion from state S0 in which the human presence is not detected 
        to state S1 in which the human presence is detected, then set the engagement flag to True.
        
        If there is a transistion from state S1 in which the human presence is detected
        to state S0 in which the human presence is not detected, then set the reset flag to True.
        
        
        Args:
            presence (Bool):  If True the human presence is detected, False otherwise.
        """
        self._previus_human_presence = self._human_presence
        self._human_presence = presence.data
        
        if not self._previus_human_presence and self._human_presence:
            self._engagement = True
            self._reset = False
        
        if  self._previus_human_presence and not self._human_presence:
            self._reset = True
        
        if self._verbose:
            if self._human_presence:
                # State S1
                    print('[Engagement]  Human presence is detected')
            else:
                # State S0
                    print('[Engagement]  Human presence is not detected')
                    
    
    def start(self, rate_value=10):
            """
            Initializes the ROS node.
            
            Subscribes to the human presence topic.
            
            Publishes the output message to the output topic, if the engagement flag is True.
            
            Publishes the reset message to the reset topic, if the engagement is
            False and reset flag (indicate that there is a transition from S1 to S0 -> reset state 
            of chatbot) is True

            Args:
                rate_value (int): The rate value for the rospy.Rate object.

            Returns:
                None
            """
            rospy.init_node(self._name_node, anonymous=True)
            rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_transition_state)
            rospy.on_shutdown(self._handle_shutdown)
            rate = rospy.Rate(rate_value)
            while not rospy.is_shutdown():
                rate.sleep()
                
                if not self._engagement:
                    if self._reset:
                        self._pub_reset.publish("goodbye")
                    self._reset = False
                    continue
                
                if self._verbose:
                    print('[Engagement]  Engagement is detected')
                
                self._pub.publish("Hello, I am Pepper, your personal assistant in the Shopping Mall")    
                
                self._engagement = False

if __name__ == '__main__':
    try:
        node = Engagement_Node('engagement_node', CHATBOT_OUTPUT_TOPIC,USER_INPUT_TOPIC ,verbose=True)
        node.start()
    except rospy.ROSInterruptException:
        pass