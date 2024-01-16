#!/usr/bin/python3

from datetime import datetime
import numpy as np
from config import *
import rospy
from base_node import BaseNode
from std_msgs.msg import String , Int16MultiArray
from scipy.io.wavfile import write
from project_work.srv import Speech2Text

class s2t_node(BaseNode):
    def __init__(self, name_node, text_topic, verbose=True) -> None:
        """Initialize a Speech to text node.
        """
        super().__init__()
        self._name_node = name_node
        self._topic = text_topic
        self._pub = rospy.Publisher(self._topic, String, queue_size=0)
        self.text_data = String()
        self._verbose = verbose
        self._audio_presence = False
        self._audio_data = Int16MultiArray()
      
    def _handle_audio(self, audio):
        """
        Callback function for the audio topic.
        
        Sets the audio presence flag to True.
        
        Sets the audio data attribute to the value written on the topic.
        
        Args:
            audio (Int16MultiArray): The audio data written on the topic.
        """
        self._audio_presence = True
        self._audio_data = audio
        if self._verbose:
            print('[S2T Node] Audio detected: {}'.format(self._audio_presence))
        
        if SAVE_RAW_AUDIO:
            audio_data = np.array(audio.data).astype(np.float32, order='C') / 32768.0  # to float32
            if not os.path.exists(os.path.join(REF_PATH, 'saved_audio')):
                os.mkdir(os.path.join(REF_PATH, 'saved_audio'))
            write(os.path.join(REF_PATH, 'saved_audio', f'{datetime.now().strftime("%m-%d-%Y-%H-%M-%S")}.wav'), RATE, audio_data)
        
          
    def start(self, audio_topic, rate_value=10):
        """
        Initialize the node and start the execution.
        
        Subscribe to the audio topic.
        
        Call the speech to text service. If ON_PEPPER is True, then the service is called on the Pepper robot,
        otherwise the service is called on the local machine.
        
        Publish the output text on the text topic.

        Args:
            audio_topic (_type_): name of the topic to subscribe to.
            rate_value (int, optional): rate at which the node is executed. Defaults to 10.
        """
        
        # Init the node
        rospy.init_node(self._name_node, anonymous=True)
        self._persistence_service_init('s2t', Speech2Text)
        rospy.on_shutdown(self._handle_shutdown)
        # Init the subscriber
        rospy.Subscriber(audio_topic, Int16MultiArray, self._handle_audio)
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            rate.sleep()
            
            if not self._audio_presence:
                continue
            
            # Call service
            speech2textResp = self._persistence_service_call('s2t', self._audio_data)
            text = speech2textResp.output.data
            self._pub.publish(text)
            if self._verbose:
                print('[S2T Node] User Input: ', text)
            
            self._audio_presence = False

if __name__ == '__main__':
    try:
        node = s2t_node('speech2text_node',USER_INPUT_TOPIC)
        node.start(USER_AUDIO_TOPIC)
    except rospy.ROSInterruptException:
        pass
        
        