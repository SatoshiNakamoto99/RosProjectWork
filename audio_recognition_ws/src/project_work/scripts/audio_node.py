#!/usr/bin/python3
# from config import *
# from base_node import BaseNode
# import rospy
# from std_msgs.msg import Int16MultiArray, Bool, String
# from threading import Lock
# from project_work.srv import StartListening

# class AudioNode(BaseNode):
#     def __init__(self,name_node, row_audio_topic, verbose=True):
#         super().__init__()
#         self._name_node = name_node
#         self._audio_topic = row_audio_topic
#         self._pub = rospy.Publisher(self._audio_topic, Int16MultiArray, queue_size=10)
#         self._verbose = verbose
#         self._human_presence = False
#         self._pepper_talk=False
        
    
#     def _get_human_presence(self):
#         presence = self._human_presence
#         return presence
    
#     def _set_human_presence(self, presence):
#         self._human_presence = presence
    
#     def _get_pepper_talk(self):
#         pepper_talk=self._pepper_talk
#         return pepper_talk
    
#     def _set_pepper_talk(self,pepper_talk):
#         self._pepper_talk=pepper_talk
        
#     def _handle_pepper_talk(self, pepper_talk):
#         self._set_pepper_talk(pepper_talk.data)
#         if self._verbose:
#             print("[Audio Node] Pepper talk: {}".format(self._get_pepper_talk()))
        
#     def _handle_human_presence(self, presence):
#         self._set_human_presence(presence.data)
#         if self._verbose:
#             print("[Audio Node] Human presence: {}".format(self._get_human_presence()))
    
#     def start(self, rate_value=10):
        
#         rospy.init_node(self._name_node, anonymous=True)
#         rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_human_presence)
#         rospy.Subscriber(PEPPER_TALK_TOPIC, Bool, self._handle_pepper_talk)
#         rospy.on_shutdown(self._handle_shutdown)
#         # Init the Service
#         self._persistence_service_init('startListening', StartListening)
#         # Per non risentirmi SUB anche topic chatbot 
#         rate = rospy.Rate(rate_value)
        
#         while not rospy.is_shutdown():
#             rate.sleep()
#             if not self._get_human_presence():
#                 continue
#             if  self._get_pepper_talk():
#                 continue
                
#             #print("sono nel while AUDIO")
#             startListeningResponse = self._persistence_service_call('startListening')
#             audio = startListeningResponse.output
#             self._pub.publish(audio)
    
    
# if __name__ == '__main__':
#     try:
#         service_mode = True
#         node = AudioNode('audio_detected_node', USER_AUDIO_TOPIC)
#         node.start()
#     except rospy.ROSInterruptException:
#         pass 
  
    

from config import *
import numpy as np
import rospy
import speech_recognition as sr
from std_msgs.msg import Int16MultiArray, Bool

class AudioNode(object):
    def __init__(self,row_audio_topic,  verbose=True) -> None:
        
        self._r = sr.Recognizer()
        self._m = sr.Microphone(device_index=MICROPHONE_INDEX, sample_rate=RATE, chunk_size=CHUNK_SIZE)
        self._audio_topic = row_audio_topic
        self._pub = rospy.Publisher(self._audio_topic, Int16MultiArray, queue_size=10)
        self._verbose = verbose 
        self._human_presence  = False 
        self._pepper_talk = False 
        self._pepper_talk_during_listening = False  
        self._start_listen = False
    
    def _get_pepper_talk(self):
        pepper_talk=self._pepper_talk
        return pepper_talk
    
    def _set_pepper_talk(self,pepper_talk):
        self._pepper_talk=pepper_talk
    
    def _get_human_presence(self):
        presence = self._human_presence
        return presence
    
    def _set_human_presence(self, presence):
        #self._human_presence = presence
        self._human_presence = presence
        
    def _get_pepper_talk_during_listening(self):
        pepper_talk_during_listening = self._pepper_talk_during_listening
        return pepper_talk_during_listening
    
    def _set_pepper_talk_during_listening(self, pepper_talk_during_listening):
        self._pepper_talk_during_listening = pepper_talk_during_listening
        
    def _get_start_listen(self):
        start_listen = self._start_listen
        return start_listen
    
    def _set_start_listen(self, start_listen):
        self._set_start_listen = start_listen
        
    
    def _handle_presence(self, presence):
        self._set_human_presence(presence.data)
        if self._verbose:
            print("[AUDIO NODE] Human presence: {}".format(self._get_human_presence()))
        if self._get_start_listen() and not self._get_human_presence():
            self._came_out = True
        
    def _handle_pepper_talk(self, pepper_talk):
        self._set_pepper_talk(pepper_talk.data)
        if self._get_start_listen() and self._get_pepper_talk():
            self._set_pepper_talk_during_listening(Bool(True))
              
    def _start_listening(self):
        
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
            if self._get_pepper_talk_during_listening() or not self._get_human_presence():
                self._pepper_talk_during_listening=False
                self._start_listen=False
                continue
            # if self._came_out:
            #     self._start_listen=False
            #     self._came_out = False
            #     continue
            self._pub.publish(text)
        
                
            

if __name__ == '__main__':
    try:
        node = AudioNode(USER_AUDIO_TOPIC)
        node.start()
    except rospy.ROSInterruptException:
        pass


