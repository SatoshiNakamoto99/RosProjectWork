#!/usr/bin/python3
import os
from config import *
from std_msgs.msg import Int16MultiArray, String
import rospy
import time
import numpy as np
from pydub import AudioSegment
TEST_PATH = os.path.dirname(os.path.abspath(__file__))

class Speech2TextNodeTest(object):
    def __init__(self):
        self._output = None
        self._groundtruth = None
        self._pub_user_audio = rospy.Publisher(USER_AUDIO_TOPIC, Int16MultiArray, queue_size=0)      
    
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

    def __get_audio(self,audio_path):
        #Controlla se la cartella è vuota
        if not os.path.exists(audio_path) or not os.listdir(audio_path):
            print(f"La cartella {audio_path} è vuota. Nessun file audio trovato.")
            array_with_none = np.array([None], dtype=object) #array con un solo elemento che è None
            return array_with_none 

        try:
            # Ottieni il primo file audio nella directory
            audio_file = next(f for f in os.listdir(audio_path) if f.endswith(('.wav', '.mp3', '.ogg', '.flac', '.aiff')))

            # Crea il percorso completo al file audio
            full_audio_path = os.path.join(audio_path, audio_file)
            #Carica l'audio utilizzando pydub
            audio = AudioSegment.from_file(full_audio_path)

            #Estrai i dati audio come array numpy di Int16
            audio_data = np.array(audio.get_array_of_samples(), dtype=np.int16)

            return audio_data
        except Exception as e:
            print(f"Errore durante il caricamento e la conversione dell'audio: {e}")
            return None

    def _test_case(self, test_case_folder):
        print(test_case_folder.upper()+":",end="\t")
        test_case_path = os.path.join(TEST_PATH,test_case_folder)
        # get groundtruth
        self._setup(test_case_path)
        
        #Per prendere l'audio e pubblicarlo sul topic USER_AUDIO_TOPIC
        audio=self.__get_audio(os.path.join(test_case_path, "test_audio"))
        if (len(audio)!=1): 
            audio_msg = Int16MultiArray(data=list(audio))
            #pubblico l'audio
            self._pub_user_audio.publish(audio_msg)#in questo modo il nodo s2t_node fa la s2t 
            #e pubblicherà il risultato su USER_INPUT_TOPIC

        time.sleep(5)#per dare il tempo a s2t_node di lavorare
        if (self._get_output()==None):
            self._set_output(False)#se entro in questo if il nodo s2t_node non ha pubblicato su USER_INPUT_TOPIC
        self._test()
        self._cleanup()     

    def __response_s2t(self,text):
        #se sono in questa callback allora il nodo s2t_node ha pubblicato su USER_INPUT_TOPIC
        self._set_output(True)

    def start(self):
        rospy.init_node('speech2text_node_test', anonymous=True)
        rospy.Subscriber(USER_INPUT_TOPIC, String,self.__response_s2t)
   
        test_cases = os.listdir(TEST_PATH)
        test_cases.sort()
        
        for test in test_cases:
            if test!="__pycache__" and test!="s2t_node_test.py" and test!="config.py":
                if not os.path.isfile(os.path.join(TEST_PATH,test)):
                    self._test_case(test)
        print("TEST FINISHED")
        print("Type CTRL+C to exit")
        while not rospy.is_shutdown():
            pass
        
if __name__ == "__main__":
    try:
        d = Speech2TextNodeTest()
        d.start()
    except rospy.ROSInterruptException:
        pass