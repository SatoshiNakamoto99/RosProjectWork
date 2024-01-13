#!/usr/bin/python3

from config import *
from fp_audio.srv import StartListening, StartListeningResponse
import numpy as np
import rospy
import speech_recognition as sr
from std_msgs.msg import Int16MultiArray
from threading import Lock


class AudioDetected(object):
    """This class use py_audio to get audio wawe with a detection."""

    def __init__(self, verbose=True) -> None:
        """Initialize an object containing the node who listen the audio from the robot and 
        implement an voice detection.

        Args:
            raw_audio_topic (str): The name of the topic on which the audio must be published.
        """
        self._r = sr.Recognizer()
        self._m = sr.Microphone(device_index=MICROPHONE_INDEX, sample_rate=RATE, chunk_size=CHUNK_SIZE)

        self._verbose = verbose
        
    
    def start(self):
        """Start the node and calibrate the microphone to the local noise.
        """
        rospy.init_node('audio_detected_node', anonymous=True)
        rospy.Service('startListening', StartListening, self._handle_start_listening, buff_size=1)
        rospy.spin()

    def _handle_start_listening(self, req):
        """Callback function for startListening service. 
        Calibrate the microphone with the ambient noise and start listening.

        Args:
            req (StartListening): empty request

        Returns:
            str: an ack who tell if the audio was listened.
        """

        with self._m as source:
            self._r.adjust_for_ambient_noise(source, duration=CALIBRATION_TIME)

            data_to_send = Int16MultiArray()

            if self._verbose:
                print("[T2S] Start listening")

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
                    print("[T2S] Listened")

            except sr.WaitTimeoutError:
                if self._verbose:
                    print("[T2S] Timeout")
                data_to_send.data = b'\x00 \x00'

            return StartListeningResponse(data_to_send)


if __name__ == "__main__":
    try:
        node = AudioDetected()
        node.start()
    except rospy.ROSInterruptException:
        node.stopListening()
