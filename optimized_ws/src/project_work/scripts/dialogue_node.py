#!/usr/bin/env python3
from config import *
from base_node import BaseNode
import rospy
from std_msgs.msg import String 
from threading import Lock
from rasa_ros.srv import Dialogue

class DialogueNode(BaseNode):
    def __init__(self, name_node, output_topic, verbose):
        super().__init__()
        self._name_node = name_node
        self._verbose = verbose
        self._input_text_peresence = False
        self._input_text = String()
        self._output_topic = output_topic
        self._pub = rospy.Publisher(self._output_topic, String, queue_size=0)
        
    def _get_input_text_peresence(self):
        input_text_peresence = self._input_text_peresence
        return input_text_peresence
    
    def _set_input_text_peresence(self, input_text_peresence):
        self._input_text_peresence = input_text_peresence
    
    def _get_input_text(self):
        input_text = self._input_text
        return input_text
    
    def _set_input_text(self, input_text):
        self._input_text = input_text
           
    def _handle_input_text(self, input_text):
        self._set_input_text_peresence(True)
        self._set_input_text(input_text.data)
        if self._verbose:
            print('[Dialogue] Input text detected: {}'.format(self._get_input_text()))
    
    def _chatbot_interaction(self, text)-> str:
        if CHATBOT_RUNNING:
            dialogueResponse = self._persistence_service_call('dialogue_server', text)
            output_text = dialogueResponse.answer
        else:
            output_text = "Chatbot not running"
        return output_text
            
    def start(self, input_topic, rate_value=10):
       

        # Init the node
        rospy.init_node(self._name_node, anonymous=True)
        rospy.Subscriber(input_topic, String, self._handle_input_text)
        rospy.on_shutdown(self._handle_shutdown)
        # Init the Service
        self._persistence_service_init('dialogue_server', Dialogue)
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self._get_input_text_peresence():
                continue
            text = self._get_input_text()
            if (text == '' or text == 'ERR1' or text == 'ERR2'):
                if self._verbose:
                    print(f'[Dialogue Node]  Does not unterstood, text={text}.')
                response_by_chatbot = 'I did not understand, can you repeat please?' 
            else:
                response_by_chatbot = self._chatbot_interaction(text)
            self._pub.publish(response_by_chatbot)
            if self._verbose:
                print('[Dialogue] Chatbot Response: ', response_by_chatbot)
            self._set_input_text_peresence(False)
            
if __name__ == '__main__':
    try:
        node = DialogueNode('dialogue_node', CHATBOT_OUTPUT_TOPIC, verbose=True)
        node.start(USER_INPUT_TOPIC)
    except rospy.ROSInterruptException:
        pass  
        