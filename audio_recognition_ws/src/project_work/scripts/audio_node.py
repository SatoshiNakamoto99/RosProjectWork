#!/usr/bin/python3
from config import *
import numpy as np
import rospy
import speech_recognition as sr
from std_msgs.msg import Int16MultiArray, Bool

class AudioNode(object):
    def __init__(self, row_audio_topic, verbose=True) -> None:
        """
        Initializes the AudioNode class.

        Args:
            row_audio_topic (str): The audio topic to publish to.
            verbose (bool, optional): Whether to enable verbose mode. Defaults to True.
        """
        self._r = sr.Recognizer()
        self._m = sr.Microphone(device_index=MICROPHONE_INDEX, sample_rate=RATE, chunk_size=CHUNK_SIZE)
        self._audio_topic = row_audio_topic
        self._pub = rospy.Publisher(self._audio_topic, Int16MultiArray, queue_size=10)
        self._verbose = verbose 
        self._human_presence  = False 
        self._pepper_talk = False 
        self._pepper_talk_during_listening = False  
        self._start_listen = False
        
    def _handle_presence(self, presence):
        """
        Callback function for the human presence topic.
        
        Sets the human presence flag to the value written on the topic.
        
        Human presence is set to True if the value is True, False otherwise.
        
        Args:
            presence (Bool): The human presence is written on topic. It is True if human is present, False otherwise.
        """
        self._human_presence = presence.data
        if self._verbose:
            print("[AUDIO NODE] Human presence: {}".format(self._human_presence))
        
    def _handle_pepper_talk(self, pepper_talk):
        """
        Callback function for the pepper talk topic.
        
        Sets the pepper talk flag to the value written on the topic.
        
        Pepper talk is set to True if the value is True, False otherwise.
        
        Args:
            pepper_talk (Bool): The pepper talk is written on topic. It is True if pepper is talking, False otherwise.
        
        """
        
        self._pepper_talk = pepper_talk.data
        if self._start_listen and self._pepper_talk:
            self._pepper_talk_during_listening = True
        if self._verbose:
            print("[AUDIO NODE] Pepper Topic: {}".format(self._pepper_talk))
              
    def _start_listening(self):
        """
        Starts listening for audio input and returns the data as an Int16MultiArray.

        Returns:
            Int16MultiArray: The audio data as an Int16MultiArray.

        Raises:
            sr.WaitTimeoutError: If a timeout occurs while listening for audio input.
        """
        with self._m as source:
            self._r.adjust_for_ambient_noise(source, duration=CALIBRATION_TIME)
            data_to_send = Int16MultiArray()
            if self._verbose:
                print("[AUDIO NODE] Start listening")
                print("""
                                    _                          
                                    | |                         
                ___ _ __   ___  __ _| | __  _ __   _____      __
                / __| '_ \ / _ \/ _` | |/ / | '_ \ / _ \ \ /\ / /
                \__ \ |_) |  __/ (_| |   <  | | | | (_) \ V  V / 
                |___/ .__/ \___|\__,_|_|\_\ |_| |_|\___/ \_/\_/  
                    | |                                          
                    |_|                                          
                    
                    """)
            try:
                audio = self._r.listen(source, timeout=5)
                data_to_send.data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
                if self._verbose:
                    print("[AUDIO NODE] Listened")

            except sr.WaitTimeoutError:
                if self._verbose:
                    print("[AUDIO NODE] Timeout")
                data_to_send.data = b'\x00 \x00'
            return data_to_send
        
    def start(self):
            """
            Starts the audio recognition process.
            
            Subscribes to relevant topics and continuously listens for audio input.
                    Topics subscribed to:
                        - HUMAN_PRESENCE_TOPIC
                        - PEPPER_TALK_TOPIC       
            
            Publishes the recognized text when audio input is detected and certain conditions are met.
                    Constraints:
                        - Human presence must be True
                        - Pepper talk must be False
                        - Pepper talk during listening must be False
                    
                    Topics published to:
                        - USER_AUDIO_TOPIC
            Args:
                None
                
            """
            
            rospy.init_node('audio_detected_node', anonymous=True)
            rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_presence)
            rospy.Subscriber(PEPPER_TALK_TOPIC, Bool, self._handle_pepper_talk)
            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                rate.sleep()

                if not self._human_presence:
                    continue
                if self._pepper_talk:
                    continue
                self._start_listen = True              
                text = self._start_listening()
                if self._pepper_talk_during_listening or not self._human_presence:
                    self._pepper_talk_during_listening = False
                    self._start_listen = False
                    continue
                self._pub.publish(text)
        
if __name__ == '__main__':
    try:
        node = AudioNode(USER_AUDIO_TOPIC, VERBOSE)
        node.start()
    except rospy.ROSInterruptException:
        pass


