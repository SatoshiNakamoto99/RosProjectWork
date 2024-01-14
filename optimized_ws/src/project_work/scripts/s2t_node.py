#!/usr/bin/python3
from config import *
import rospy
from base_node import BaseNode
from std_msgs.msg import String , Int16MultiArray
from threading import Lock
from project_work.srv import Speech2Text

class s2t_node(BaseNode):
    def __init__(self, name_node, text_topic, verbose=True) -> None:
        """Initialize a Speech to text node.
        """
        super().__init__()
        # Init recognizer
        #self._r = sr.Recognizer()
        # Init the publisher
        self._name_node = name_node
        self._topic = text_topic
        self._pub = rospy.Publisher(self._topic, String, queue_size=0)
        self.text_data = String()
        self._verbose = verbose
        self._audio_presence = False
        self._audio_data = Int16MultiArray()
    
       
  
    def _get_audio_presence(self):
        audio_presence = self._audio_presence
        return audio_presence
    
    def _set_audio_presence(self, audio_presence):
        self._audio_presence = audio_presence
    
    def _get_audio_data(self):
        audio_data = self._audio_data
        return audio_data
    
    def _set_audio_data(self, audio_data):
        self._audio_data = audio_data
        
    def _handle_audio(self, audio):
        self._set_audio_presence(True)
        self._set_audio_data(audio)
        if self._verbose:
            print('[Speech2text] Audio detected: {}'.format(self._get_audio_presence()))
          
    def start(self, audio_topic, rate_value=10):
        # Init the node
        rospy.init_node(self._name_node, anonymous=True)
        self._persistence_service_init('s2t', Speech2Text)
        rospy.on_shutdown(self._handle_shutdown)
        # Init the subscriber
        rospy.Subscriber(audio_topic, Int16MultiArray, self._handle_audio)
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            rate.sleep()
            if not self._get_audio_presence():
                continue
            # Call service
           
            speech2textResp = self._persistence_service_call('s2t', self._get_audio_data())
            text = speech2textResp.output.data
            self._pub.publish(text)
            if self._verbose:
                print('[Speech2text] User Input: ', text)
            
            self._set_audio_presence(False)

if __name__ == '__main__':
    try:
        node = s2t_node('speech2text_node',USER_INPUT_TOPIC)
        node.start(USER_AUDIO_TOPIC)
    except rospy.ROSInterruptException:
        pass
        
        