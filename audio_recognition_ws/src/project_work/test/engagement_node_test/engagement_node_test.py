#!/usr/bin/python3
import os
import json
TEST_PATH = os.path.dirname(os.path.abspath(__file__))
from base_test import BaseTest
from config import *
from std_msgs.msg import String, Bool
import rospy
import time

class EngagementNodeTest(BaseTest):
    def __init__(self):
        super().__init__()
        self._pub_human_presence = rospy.Publisher(HUMAN_PRESENCE_TOPIC, Bool, queue_size=0)
        self._HUMAN_PRESENCE = False

        self._set_output(False)
        
    def __read_config(self, file_path):
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                # Imposta le variabili in base ai valori nel file JSON
                self._HUMAN_PRESENCE = data.get("TEST_HUMAN_PRESENCE", False)
        except FileNotFoundError:
            print(f"File '{file_path}' non trovato. Impostazione di valori predefiniti.")



    def __test_case(self, test_case_folder):
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)
        
        print("Groundtruth: {}".format(self._get_groundtruth()))
        print("Init Output: {}".format(self._get_output()))

        text_path = test_case_path
        self.__read_config(os.path.join(text_path, "config_test.json"))
        #pubblico le cose sui topic che saranno usati da engagement_node
        if self._HUMAN_PRESENCE:
            self._pub_human_presence.publish(Bool(True))
        else:
            self._pub_human_presence.publish(Bool(False))

        #dopo aver pubblicato questa cosa il nodo di engagement può valutare se fare o meno l'engagment
        #Se lo fa pubblica su CHATBOT_OUTPUT_TOPIC e quindi andrò ad eseguire __response_by_chatbot

        #confronto tra output che ho ottenuto (cioè se ci sta la stringa di interesse in CHATBOT_OUTPUT_TOPIC) e la gt
        time.sleep(2) #do il tempo a engagement_node di pubblicare su CHATBOT_OUTPUT_TOPIC
        #così questo nodo mette il valore in output e lo posso confrontare con gt
        print(" Output: {}".format(self._get_output()))
        self._test()
        self._cleanup()

        

    def __response_by_chatbot(self, response):
        if (response.data == "Hello, I am Pepper, your personal assistant in the Shopping Mall"):
            self._set_output(True) #se è true allora ho fatto l'engagement


    def start(self):
        rospy.init_node('engagement_node_test', anonymous=True)
        rospy.Subscriber(CHATBOT_OUTPUT_TOPIC, String,self.__response_by_chatbot)
   
        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        
        for test in test_cases:
            if test!="__pycache__" and test!="base_test.py" and test!="engagement_node_test.py":
                if not os.path.isfile(os.path.join(TEST_PATH,test)):
                    self.__test_case(test)
        print("TEST FINISHED")
        print("Type CTRL+C to exit")
        while not rospy.is_shutdown():
            pass
                    
if __name__ == "__main__":
    try:
        d = EngagementNodeTest()
        d.start()
    except rospy.ROSInterruptException:
        pass