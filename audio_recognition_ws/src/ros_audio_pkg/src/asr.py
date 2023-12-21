#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np

from speech_recognition import AudioData
import speech_recognition as sr
import openai
import os
import sys
#OPENAI_API_KEY = os.environ["OPENAI_API_KEY_FREE"]



#OPENAI_API_KEY = os.getenv("OPENAI_API_KEY_FREE")
# Initialize a Recognizer
r = sr.Recognizer()

# Init node
rospy.init_node('speech_recognition', anonymous=True)
pub1 = rospy.Publisher('voice_data', Int16MultiArray, queue_size=10)
pub2 = rospy.Publisher('voice_txt', String, queue_size=0)
#OPENAI_API_KEY = "INSERT OPENAI API KEY HERE"
#try:
#    print(f"Whisper API thinks you said {.recognizre_whisper_api(audio, api_key=OPENAI_API_KEY)}")
# this is called from the background thread
def callback(audio):
    data = np.array(audio.data,dtype=np.int16)
    audio_data = AudioData(data.tobytes(), 16000, 2)

    try:
        spoken_text= r.recognize_google(audio_data, language='en-GB')
        #spoken_text = r.recognize_whisper_api(audio_data, api_key=OPENAI_API_KEY)
        
        #print("Whisper Speech Recognition pensa tu abbia detto: " + spoken_text)
        print("Google Speech Recognition pensa tu abbia detto: " + spoken_text)
        pub1.publish(audio) # Publish audio only if it contains words
        pub2.publish(spoken_text)
    except sr.UnknownValueError:
        print("Google Speech Recognition non riesce a capire da questo file audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))

def listener():
    rospy.Subscriber("mic_data", Int16MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()