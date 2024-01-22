#!/usr/bin/env python3
from config import *
from base_service import BaseService
import rospy
from std_msgs.msg import String , Bool
from rasa_ros.srv import Dialogue
import time

class DialogueNode(BaseService):
    def __init__(self, name_node, output_topic, verbose):
        """
        Initialize the DialogueNode class. DialogueNode is a ROS node that
        DialogueNode inherits from the BaseService class.

        Args:
            name_node (str): The name of the node.
            output_topic (str): The topic to publish the output to.
            verbose (bool): Flag indicating whether to enable verbose mode.
        """
        super().__init__()
        self._name_node = name_node
        self._verbose = verbose
        self._input_text_peresence = False
        self._input_text = String()
        self._output_topic = output_topic
        self._pub = rospy.Publisher(self._output_topic, String, queue_size=0)
        self._human_presence = False
    
    def _handle_input_text(self, input_text):
        """"
        Callback function for the input text topic.
        
        Sets the input text attribute to the value written on the topic.
        
        Sets the input text presence flag to True.
        
        Args:
            input_text (String): The input text written on topic.
        
        """
        
        self._input_text = input_text.data
        self._input_text_peresence = True   
        if self._verbose:
            print('[Dialogue Node] Input text detected: {}'.format(self._input_text))
    
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
        self._human_presence = presence.data
    def _chatbot_interaction(self, text)-> str:
        
        """"
        Call the dialogue server service to get the chatbot response if CHATBOT_RUNNING is True, else
        return a string indicating that the chatbot is not running.
        
            CHATBOT_RUNNING is a flag that indicates whether the chatbot is running or not. It
            is set in the config.py file.
        
        Args:
        
            text (str): The input text to send to the chatbot.
        
        """
        
        if CHATBOT_RUNNING:
            dialogueResponse = self._persistence_service_call('dialogue_server', text)
            output_text = dialogueResponse.answer
        else:
            output_text = "Chatbot not running"
        return output_text
            
    def start(self, input_topic, rate_value=1):
            """
            Starts the dialogue node.
            Initializes the node, the service and the subscriber.
                Subscribes to the input topic.
            
            Calls the chatbot interaction function to get the chatbot response if the input
            text presence flag is True.
            

            Args:
                input_topic (str): The topic to subscribe to for input text.
                rate_value (int, optional): The rate value for the rospy.Rate object. Defaults to 2.
            """
            # Init the node
            rospy.init_node(self._name_node, anonymous=True)
            rospy.Subscriber(input_topic, String, self._handle_input_text)
            rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_transition_state)
            
            
            if CHATBOT_RUNNING:
                rospy.on_shutdown(self._handle_shutdown)
                #rospy.wait_for_service('dialogue_server')
                self._persistence_service_init('dialogue_server', Dialogue)
            rate = rospy.Rate(rate_value)
            while not rospy.is_shutdown():
                rate.sleep()
                #
                if not self._input_text_peresence or not self._human_presence:
                        
                    text = '/reset'
                    continue
                
                text = self._input_text
                if (text == '' or text == 'ERR1' or text == 'ERR2' or text == '/reset'): 
                    continue
                else:
                    response_by_chatbot = self._chatbot_interaction(text)
                self._pub.publish(response_by_chatbot)
                if self._verbose:
                    print('[Dialogue] Chatbot Write on topic: ', response_by_chatbot)
                
                self._input_text_peresence = False
            
if __name__ == '__main__':
    try:
        node = DialogueNode('dialogue_node', CHATBOT_OUTPUT_TOPIC, VERBOSE)
        node.start(USER_INPUT_TOPIC)
    except rospy.ROSInterruptException:
        pass  
        