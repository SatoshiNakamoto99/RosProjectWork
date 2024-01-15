#!/usr/bin/python3
from config import *
import rospy
from base_node import BaseNode
from std_msgs.msg import String ,  Bool
from pepper_nodes.srv import Text2Speech
from playsound import playsound
from gtts import gTTS

class t2s_node(BaseNode):
    def __init__(self, name_node, topic_pepper, verbose=True)-> None:
        """Initialize a Text to Speech node"""
        super().__init__()
        self._name_node=name_node
        self._verbose = verbose
        self._human_presence = False
        self._chatbot_output_presence=False
        self._chatbot_output=String()
        self._topic_pepper=topic_pepper
        self._pub = rospy.Publisher(self._topic_pepper, Bool, queue_size=0)

    def _get_human_presence(self):
        presence = self._human_presence
        return presence
    
    def _set_human_presence(self, presence):
        self._human_presence = presence
    
    def _get_chatbot_output_presence(self):
        chatbot_output_presence=self._chatbot_output_presence
        return chatbot_output_presence

    def _set_chatbot_output_presence(self,presence):
        self._chatbot_output_presence=presence

    def _get_chatbot_output(self):
        chatbot_output=self._chatbot_output
        return chatbot_output
    
    def _set_chatbot_output(self,chatbot_output):
        self._chatbot_output=chatbot_output

    def _handle_human_presence(self, presence):
        self._set_human_presence(presence.data)
        if self._verbose:
            print("[Text2Speech] Human presence: {}".format(self._get_human_presence()))
        
    def _handler_chatbot_output_presence(self, data):
        self._set_chatbot_output_presence(True)
        self._set_chatbot_output(data.data)
        if self._verbose:
            print('[Text2Speech] Text detected: {}'.format(self._get_chatbot_output_presence()))

    
    def _t2s(self, text):
        """Call Pepper text to speech or simulate that. 

        Args:
            text (str): the text to speech.
        """
        if ON_PEPPER:
            self._persistence_service_call('tts', text)
        else:
            try:
                to_speak = gTTS(text=text, lang=LANGUAGE, slow=False)
                to_speak.save("temp.wav")
                playsound("temp.wav")
                os.remove("temp.wav")
            except AssertionError:
                pass

    def start(self, rate_value=10):
        rospy.init_node(self._name_node, anonymous=True)
        
        rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_human_presence)
        rospy.Subscriber(CHATBOT_OUTPUT_TOPIC, String,self._handler_chatbot_output_presence)
        rospy.on_shutdown(self._handle_shutdown)
        if ON_PEPPER:    
            self._persistence_service_init('tts', Text2Speech)
        rate=rospy.Rate(rate_value)

        while not rospy.is_shutdown():
            rate.sleep()
            if not self._get_human_presence():
                continue
            if not self._get_chatbot_output_presence():
                continue
            #se sono qui allora ci sta una persona avanti e ho la risposta del chatbot
            text=self._get_chatbot_output() #risposta del chatbot
            self._pub.publish(True)
            self._set_chatbot_output_presence(False)
            self._t2s(text)
            self._pub.publish(False)
            if self._verbose:
                print('[Text2Speech] Text to speech: ', text)
            
            
if __name__ == '__main__':
    try:    
        node = t2s_node('t2s_node',PEPPER_TALK_TOPIC,verbose=True)
        node.start()
    except rospy.ROSInterruptException:
        pass
        
