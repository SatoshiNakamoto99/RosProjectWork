#!/usr/bin/python3
import os
import json
#from base_test import BaseTest
from std_msgs.msg import String
from config import *
import rospy
import time

TEST_PATH = os.path.dirname(os.path.abspath(__file__))


class DialogueNodeTest(object):
    def __init__(self):
        """
        Initializes the dialogue node testing module. """
        self._output = None
        self._groundtruth = None
        self._pub_user_input = rospy.Publisher(USER_INPUT_TOPIC, String, queue_size=0)
        self.INPUT_FROM_USER=""
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
        Read configuration values from a JSON file.

        Args:
            file_path (String): Path to the JSON configuration file.
        """
        try:
            with open(file_path, 'r') as json_file:
                data = json.load(json_file)
                # Imposta le variabili in base ai valori nel file JSON
                self.INPUT_FROM_USER = data.get("TEST_INPUT_FROM_USER", False)
        except FileNotFoundError:
            print(f"File '{file_path}' non trovato. Impostazione di valori predefiniti.")

    def __test_case(self, test_case_folder):
        """
        Execute a test case based on data stored in the specified folder.

        Args:
            test_case_folder (String): Folder where test data is stored.
        """
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)

        text_path = test_case_path
        self.__read_config(os.path.join(text_path, "config_test.json"))
        #pubblico su USER_INPUT_TOPIC il contenuto di config_test.json
        self._pub_user_input.publish(String(self.INPUT_FROM_USER))#dopo aver pubblicato
        #su questo topic il nodo dialogue_node farà la callback _handle_input_text
        #Se quello che pubblico è diverso da err1 e err2 o "" allora il dialogue_node publica su CHATBOT_OUTPUT_TOPIC
        print("inizio sleep")
        time.sleep(10)#In questo modo il dialogue_node ha tempo di pubblicare su CHATBOT_OUTPUT_TOPIC,
        #di conseguenza questo nodo aggiorna il valore di output per poterlo poi confrontare con gt.
        if(self._get_output()==None):
            print("############_test_case")
            self._set_output(False)#se sono qui allora il nodo dialogue_node non ha pubblicato su CHATBOT_OUTPUT_TOPIC
        self._test()
        self._cleanup()


    def __response_by_chatbot(self, response):
        """
        Callback function for the CHATBOT_OUTPUT_TOPIC.

        Args:
            response (std_msgs/String): The response message from the chatbot.
        """
        print("############_response_by_chatbot")
        self._set_output(True) #se è true allora dialogue_node ha pubblicato qualcosa in CHATBOT_OUTPUT_TOPIC


    def start(self):
        """
        Start the testing process. Initialize the ROS node, subscribe to the chatbot output topic,
        publish user input messages, and wait for the ROS node to terminate.
        """
        rospy.init_node('dialogue_node_test', anonymous=True)
        rospy.Subscriber(CHATBOT_OUTPUT_TOPIC, String,self.__response_by_chatbot)
   
        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        
        for test in test_cases:
            if test!="__pycache__" and test!="base_test.py" and test!="dialogue_node_test.py":
                if not os.path.isfile(os.path.join(TEST_PATH,test)):
                    self.__test_case(test)
        print("TEST FINISHED")
        print("Type CTRL+C to exit")
        while not rospy.is_shutdown():
            pass
        
        

if __name__ == "__main__":
    try:
        d = DialogueNodeTest()
        d.start()
    except rospy.ROSInterruptException:
        pass