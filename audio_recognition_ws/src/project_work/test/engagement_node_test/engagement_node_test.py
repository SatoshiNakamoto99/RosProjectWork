#!/usr/bin/python3
import os
import json
TEST_PATH = os.path.dirname(os.path.abspath(__file__))
#from base_test import BaseTest
from config import *
from std_msgs.msg import String, Bool
import rospy
import time

class EngagementNodeTest(object):
    def __init__(self):
        """
        Initializes the engagement node testing module.
        """
        self._output = None
        self._groundtruth = None
        self._pub_human_presence = rospy.Publisher(HUMAN_PRESENCE_TOPIC, Bool, queue_size=0)
        self._HUMAN_PRESENCE = False
        self._set_output(False)
        self._user_go_out=False

    def _setup(self,test_case_path):
        """
        Load groundtruth from file stored in specific test_case_path.
        Args:
            test_case_path (String)
        """
        with open(os.path.join(test_case_path,"groundtruth.txt"),"r") as f:
            g = f.read()
        if g=="User go out":
            self._groundtruth="User go out"
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
                self._HUMAN_PRESENCE = data.get("TEST_HUMAN_PRESENCE", False)
                self._USER_GO_OUT_FLAG=data.get("TEST_USER_GO_OUT", False)
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
        #pubblico sul topic, il nodo di engagement_node leggerà questi valori
        if self._USER_GO_OUT_FLAG:
            self._pub_human_presence.publish(Bool(True))
            self._pub_human_presence.publish(Bool(False))
            #in questo modo in engagement_node ho che self._reset = True
        else:
            if self._HUMAN_PRESENCE:
                self._pub_human_presence.publish(Bool(True))
            
        #dopo aver pubblicato il nodo di engagement può valutare se fare o meno l'engagment
        #Se lo fa pubblica su CHATBOT_OUTPUT_TOPIC e quindi il nodo di engagement_node_test andrà
        #ad eseguire __response_by_chatbot

        #confronto tra output che ho ottenuto (cioè se ci sta la stringa di interesse in CHATBOT_OUTPUT_TOPIC) e la gt
        time.sleep(2) #do il tempo a engagement_node di pubblicare su CHATBOT_OUTPUT_TOPIC
        #In questo modo il nodo di engagement_node_test può aggiornare il valore di "output" e lo posso confrontare con gt
        
        if(self._user_go_out==True):
            #se sono qui allora l'utente è uscito dalla scena
            self._set_output("User go out")
        print(self._get_output())
        
        self._test()
        self._cleanup()     

    def __response_by_chatbot(self, response):
        """
        Callback function for the CHATBOT_OUTPUT_TOPIC.

        Args:
            response (std_msgs/String): The response message from the chatbot.
        """

        if (response.data == "Hello, I am Pepper, your personal assistant in the Shopping Mall"):
            self._set_output(True) #se è true allora ho fatto l'engagement

    def __user_input(self, text):
        """
        Callback function for the USER_INPUT_TOPIC

        """
        if(text.data=="/reset"):
            self._user_go_out=True


    def start(self):
        """
        Start the testing process. Initialize the ROS node, subscribe to the chatbot output topic,
        publish human presence messages, publish user input messages, and wait for the ROS node to terminate.
        """
        
        rospy.init_node('engagement_node_test', anonymous=True)
        rospy.Subscriber(CHATBOT_OUTPUT_TOPIC, String,self.__response_by_chatbot)
        rospy.Subscriber(USER_INPUT_TOPIC, String,self.__user_input)
        

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