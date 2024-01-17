#!/usr/bin/python3
import os
from config import *
from std_msgs.msg import Int16MultiArray, Bool
import rospy
import json
import time
TEST_PATH = os.path.dirname(os.path.abspath(__file__))
from gtts import gTTS
from playsound import playsound


class AudioNodeTest(object):
    def __init__(self):
        """
        Initializes the AudioNodeTest class.
        """
        self._output = None
        self._groundtruth = None
        self._pub_pepper_talk = rospy.Publisher(PEPPER_TALK_TOPIC, Bool, queue_size=0)      
        self._pub_human_presence = rospy.Publisher(HUMAN_PRESENCE_TOPIC, Bool, queue_size=0)      
        self._PEPPER_TALK=False
        self._HUMAN_PRESENCE=False
        self._set_output(False)

    def _setup(self,test_case_path):
        """
        Load groundtruth from file stored in specific test_case_path.
        Args:
            test_case_path (String)
        """
        with open(os.path.join(test_case_path,"groundtruth.txt"),"r") as f:
            g = f.read()
        if g=="False":
            self._groundtruth=False
        if g=="":
            self._groundtruth="Groundtruth empty"
        if g=="True":
            self._groundtruth=True
        return

    def _cleanup(self):
        """
        Reset groundtruth and detector output
        """
        self._groundtruth = None
        self._output = None

    def _test(self):
        """
        Test the correctness of the detector output
        """
        # check if the sequece 
        if self._get_groundtruth()=="Groundtruth empty":
            print("Groundtruth empty")
        else:
            if self._output == self._groundtruth:
                print("Passed")
            else:
                print("gt: ")
                print(self._get_groundtruth())
                print("output: ")
                print(self._get_output())
                print("Failed")
    
    def _get_output(self):
        """
        Return detector output
        """
        return self._output
    
    def _set_output(self,output):
        """
        Set detector output
        """
        self._output=output
    
    def _get_groundtruth(self):
        """
        Return groundtruth
        """
        return self._groundtruth

    def __read_config(self, file_path):
        """
        Read configuration settings from a JSON file.

        Args:
            file_path (String): Path to the configuration JSON file.
        """
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                # Imposta le variabili in base ai valori nel file JSON
                self._PEPPER_TALK = data.get("TEST_PEPEPER_TALK", False)
                self._HUMAN_PRESENCE = data.get("TEST_HUMAN_PRESENCE", False)
        except FileNotFoundError:
            print(f"File '{file_path}' non trovato. Impostazione di valori predefiniti.")


    def _test_case(self,test_case_folder):
        """
        Execute a test case, including publishing values on ROS topics, generating temporary audio,
        and testing the audio node output.

        Args:
            test_case_folder (String): Name of the test case folder.
        """
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)

        text_path = test_case_path
        self.__read_config(os.path.join(text_path, "config_test.json"))
        #pubblico sui topic, il nodo di audio_node leggerà questi valori
        self._pub_pepper_talk.publish(Bool(self._PEPPER_TALK ))
        self._pub_human_presence.publish(Bool(self._HUMAN_PRESENCE))

        #audio_path=os.path.join(test_case_path,"test_audio.wav")
        
        text="Hello Pepper"
        to_speak = gTTS(text=text, lang=LANGUAGE, slow=False)
        to_speak.save("temp.wav")
        playsound("temp.wav")
        os.remove("temp.wav")


        time.sleep(2)#in questo modo il nodo di audio_node ha il tempo di pubblicare l'audio sul topic
        if (self._get_output()==None): #se è None allora non ho eseguito la callback _audio_user
            self._set_output(False)
        self._test()
        self._cleanup()     


    def _audio_user(self, audio):
        """
        Callback method for handling audio-related events.

        Args:
            audio (Int16MultiArray): Audio data received from the audio node.
        """
        self._set_output(True)

    def start(self):
        """
        Start the testing process. Initializes the ROS node, subscribes to audio topic, 
        iterates through test cases, and waits for the ROS node to shutdown.
        """
        rospy.init_node('audio_node_test', anonymous=True)
        rospy.Subscriber(USER_AUDIO_TOPIC, Int16MultiArray,self._audio_user)
    
        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        
        for test in test_cases:
            if test!="__pycache__" and test!="audio_node_test.py" and test!="config.py":
                #if (test=="test_case_3"):
                if not os.path.isfile(os.path.join(TEST_PATH,test)):
                    self._test_case(test)
        print("TEST FINISHED")
        print("Type CTRL+C to exit")
        while not rospy.is_shutdown():
            pass
        
if __name__ == "__main__":
    try:
        d = AudioNodeTest()
        d.start()
    except rospy.ROSInterruptException:
        pass