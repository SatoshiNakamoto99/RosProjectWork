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
        """
        Initialize a Text to Speech node
        
        Args:
            name_node (str): The name of the node.
            topic_pepper (str): The topic to publish the output to.
            verbose (bool): Flag indicating whether to enable verbose mode.
        """
        super().__init__()
        self._name_node=name_node
        self._verbose = verbose
        self._human_presence = False
        self._chatbot_output_presence=False
        self._chatbot_output=String()
        self._topic_pepper=topic_pepper
        self._pub = rospy.Publisher(self._topic_pepper, Bool, queue_size=0)

    def _handle_human_presence(self, presence):
        """
        Callback function for the human presence topic.
        
        Sets the human presence flag to the value written on the topic.
        
        Args:
            presence (_type_Bool): The human presence is written on topic. It is True if human is present, False otherwise.
        """
        self._human_presence=presence.data
        if self._verbose:
            print("[T2S Node] Human presence: {}".format(self._human_presence))
        
    def _handler_chatbot_output_presence(self, data):
        """"
        Callback function for the chatbot output topic.
        
        Sets the chatbot output presence flag to True.
        
        Sets the chatbot output attribute to the value written on the topic.
        
        Args:
            data (String): The chatbot output written on topic.
        """
        
        self._chatbot_output_presence=True
        self._chatbot_output=data.data
        if self._verbose:
            print('[T2S Node] Text detected: {}'.format(self._chatbot_output_presence))
    
    def _t2s(self, text):
        """
        Call the text to speech service if ON_PEPPER is True, else call the Google Text to Speech API.

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

    def start(self, rate_value=1):
        """"
        Initialize the node and start the execution.
        
        Subscribe to the human presence topic and to the chatbot output topic.
        
        Call the text to speech service if the human presence flag and the chatbot output 
        presence flag are True.
        
        Args:
            rate_value (int, optional): rate at which the node is executed. Defaults to 10.
        """
        
        rospy.init_node(self._name_node, anonymous=True)
        
        rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_human_presence)
        rospy.Subscriber(CHATBOT_OUTPUT_TOPIC, String,self._handler_chatbot_output_presence)
        rospy.on_shutdown(self._handle_shutdown)
        if ON_PEPPER:    
            self._persistence_service_init('tts', Text2Speech)
        rate=rospy.Rate(rate_value)

        while not rospy.is_shutdown():
            rate.sleep()
            
            if not self._human_presence:
                continue
            
            if not self._chatbot_output_presence:
                continue
            
            text = self._chatbot_output
            self._pub.publish(True)
            
            
            self._chatbot_output_presence = False
            self._t2s(text)
            self._pub.publish(False)
            if self._verbose:
                print('[T2S Node] Text to speech: ', text)
            
            
if __name__ == '__main__':
    try:    
        node = t2s_node('t2s_node',PEPPER_TALK_TOPIC,VERBOSE)
        node.start()
    except rospy.ROSInterruptException:
        pass
        
