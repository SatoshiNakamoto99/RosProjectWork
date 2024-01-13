#!/usr/bin/python3

from config import *
from datetime import datetime
from fp_audio.srv import Speech2Text,  StartListening
from gtts import gTTS
import numpy as np
from rasa_ros.srv import Dialogue
from pepper_nodes.srv import Text2Speech, WakeUp, StartFollowing
from playsound import playsound
import rospy
from scipy.io.wavfile import write
from std_msgs.msg import Int16MultiArray, String, Int16, Bool
from threading import Lock
from time import sleep

class ManagerNode(object):
    
    """Node implementing the core flow and the two states of the robot."""
    
    def __init__(self, verbose = True)-> None:
        """Initialize the manager node.
        """
        self._human_presence = False
        self._mutex_human_presence = Lock()
        self._persistent_services = dict()  # dict str -> Touple[func, module]
        self._verbose = verbose
        self._previous_human_presence = False
        self._engage = False
    
    def _persistence_service_init(self, service_name, service_srv)-> None:
        """Init a persistent connection to a service and store the needed parameters.

        Args:
            service_name (str): the service name.
            service_srv (Module): the srv module.
        """
        self._persistent_services[service_name] = (rospy.ServiceProxy(service_name, service_srv, persistent=True), service_srv)

    def _persistence_service_call(self, service_name, *args)-> any:
        """Call a service with the provided arguments and return what the service provide.
        This method handle connection problem, but do NOT handle wrong call.

        Args:
            service_name (str): the service name.

        Returns:
            any: the service output.
        """
        rospy.wait_for_service(service_name)
        try:
            return self._persistent_services[service_name][0](*args)
        except rospy.ServiceException as e:
            self._persistence_service_init(service_name, self._persistent_services[service_name][1])
            return self._persistent_services[service_name][0](*args)
    
    def _persistence_service_close(self, service_name)-> None:
        """Close a persistent connection to a service.

        Args:
            service_name (str): the service name.
        """
        try:
            self._persistent_services[service_name][0].close()
        except rospy.ServiceException :
            pass
    
    def _service_call(self, service_name, service_srv, *args)-> any:
        """Call a service whitout use a persistent connection.

        Args:
            service_name (str): the service name
            service_srv (Module): the service srv

        Returns:
            any: The service output
        """
        rospy.wait_for_service(service_name)
        func = rospy.ServiceProxy(service_name, service_srv)
        return func(*args)

    def _chatbot_interaction(self, text)-> str:
        """Call the chatbot service to get the answer to a question.

        Args:
            text (str): the question.

        Returns:
            str: the answer.
        """
        if CHATBOT_RUNNING:
            bot_answer = self._persistence_service_call('dialogue_server', text)
            
            response = bot_answer.answer
        else:
            "Simulate the chatbot interaction"
            response = "Response: "
            
        return response
    
    def _t2s(self, text):
        """Call Pepper text to speech or simulate that. 

        Args:
            text (str): the text to speech.
        """
        if ON_PEPPER:
            self._persistence_service_call('tts', text)
        else:
            try:
                to_speak = gTTS(text=text, lang=LANGUAGE, slow=False)
                to_speak.save("temp.wav")
                playsound("temp.wav")
                os.remove("temp.wav")
            except AssertionError:
                pass
        
        if self._verbose:
            print(f'[Manager Node] TTS: {text}')
    
    def _handle_tracking(self, presence):
        """Callback function for topic track/human_presence. Implement an FSM to 
        stop all operation and reset the state if the human exit the scene and 
        restart if an human enter in the scene.


        Args:
            presence (std_msgs/Bool): True if a person is present, otherwise False. 
        """
        
        # ______________________________________________________________________________  
        # 0.    This variable contains True if an human is present, false if not. We can
        #       assume if this function is running that state this value is just changed.
        
        self._mutex_human_presence.acquire()
        self._previous_human_presence = self._human_presence
        self._human_presence = presence.data
        
        if self._human_presence and not self._previous_human_presence:
             # Transition from S0 to S1
            if self._verbose:
                print('[Manager Node] Tracking: Transation from S0 to S1')
        #     # Call the service to say Hi to the user
        #     self._t2s('Hello')
           # Set the flag to engage the chatbot
            self._engage = True
        else:
            # set the flag to disengage the chatbot
            self._engage = False

        if self._human_presence:
            # ______________________________________________________________________________  
            # STATE S1:
            # In this state the microphone is enabled and consequentially all the 
            # application work.

            if self._verbose:
                print('[Manager Node] Tracking: Human presence detected.')

        else:
            # ______________________________________________________________________________  
            # STATE S0:
            # In this state there is no human so we can disable the microphone and reset
            # the state of the application so that can restart when a new human come.

            if self._verbose:
                print('[Manager Node] Tracking: There is not human anymore.')
                
            # Also reset the state of the chatbot, so that will be ready to start a new
            # conversation.
            if CHATBOT_RUNNING:
                self._chatbot_interaction('bye')

        self._mutex_human_presence.release()
        
    def _handle_shutdown(self):
        """On killing the application, stop all the service that can be running on 
        Pepper, so the following and the movement. After that set Pepper in rest 
        position.
        """
        if self._verbose:
            print('[Manager Node] Shutdown signal received. Stopping the application')

        # Close all persistent service
        for ps in self._persistent_services:
            self._persistence_service_close(ps)
            
    def start(self):
        """Start the node. Subscribe to all topic and services required. Wake up pepper
        and start some modules on that.
        """
        rospy.init_node('manager', anonymous=True)
        rospy.Subscriber(HUMAN_PRESENCE_TOPIC, Bool, self._handle_tracking)
        rospy.on_shutdown(self._handle_shutdown)

        # Init the persistent service proxy
        self._persistence_service_init('dialogue_server', Dialogue)
        self._persistence_service_init('s2t', Speech2Text)
        self._persistence_service_init('startListening', StartListening)
        self._persistence_service_init('tts', Text2Speech)

        # Pepper WakeUp & start following people
        if ON_PEPPER:
            if self._verbose:
                print('[Manager Node] Pepper wakeup')
            self._service_call('wakeup', WakeUp)

            if self._verbose:
                print('[Manager Node] Pepper startfollowing')
            self._service_call('startFollowing', StartFollowing)
        
        if self._verbose:
            print('[Manager Node] Starting done.')

        # ______________________________________________________________________________
        # This is the main cycle of the application.
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()
            if self._engage:
                self._t2s("Hello")
                if self._verbose:
                    print(f'[Manager Node] 0. Engage phase')
                
            # ______________________________________________________________________________
            # 1.    This method must be thread-safe, so we check with a mutex. 
            #       In case no human is in front of Pepper, we stop the unnecessary run.
            self._mutex_human_presence.acquire()

            if not self._human_presence:
                self._mutex_human_presence.release()
                continue
            else:                
                self._mutex_human_presence.release()
            
            # ______________________________________________________________________________
            # 1.1.  If a person is present listen audio.
            startListeningResponse = self._persistence_service_call('startListening')
            audio = startListeningResponse.output

            # ______________________________________________________________________________
            # 1.2   Speech2Text to understand if there is noise or not in that audio.
            #       Also, get the text from the audio. 
            #       If there's no words in the audio, restart the cicle
            speech2textResp = self._persistence_service_call('s2t', audio)
            text = speech2textResp.output.data

            if (text == '' or text == 'ERR1' or text == 'ERR2'):
                if self._verbose:
                    print(f'[Manager Node] 1. Does not unterstood, text={text}.')
                
                self._mutex_human_presence.acquire()
                if self._human_presence:
                    self._t2s("I don't understood.")
                self._mutex_human_presence.release()
                continue

            if self._verbose:
                print(f'[Manager Node] 1. Listened: {text}')

            # This part is used just for save some audio to use in the tests from home.
            if SAVE_RAW_AUDIO:
                audio_data = np.array(audio.data).astype(np.float32, order='C') / 32768.0  # to float32
                if not os.path.exists(os.path.join(REF_PATH, 'saved_audio')):
                    os.mkdir(os.path.join(REF_PATH, 'saved_audio'))
                write(os.path.join(REF_PATH, 'saved_audio', f'{datetime.now().strftime("%m-%d-%Y-%H-%M-%S")}.wav'), RATE, audio_data)
            # ______________________________________________________________________________
            # 2.    Interaction with the chatbot. The chatbot take in input the sentence from 
            #       the user and return the answer.
            response_text = self._chatbot_interaction(text)
            if self._verbose:
                print(f'[Manager Node] 2. Chatbot: {response_text}')
            
            # ______________________________________________________________________________
            # 3.    The response can come after a person go away, so we chech if the person is
            #       here or not.
            self._mutex_human_presence.acquire()
            if not self._human_presence:
                self._mutex_human_presence.release()
                if self._verbose:
                    print(f'[Manager Node] 3. Human presence lost.')
                continue
            else:
                if self._verbose:
                    print(f'[Manager Node] 3. Human presence still here.')
                self._mutex_human_presence.release()
            
            # ______________________________________________________________________________
            # 4.    Text to speech the answer.
            self._t2s(response_text)
            if self._verbose:
                print(f'[Manager Node] 4. TTS: {response_text}')
                
if __name__ == '__main__':
    try:
        node = ManagerNode()
        node.start()
    except rospy.ROSInterruptException as e:
        print(f'[Manager Node] rospy.ServiceException. Service call failed: {e}')
    except Exception as e:
        print(f'[Manager Node] ERROR in execution: {e}')

        