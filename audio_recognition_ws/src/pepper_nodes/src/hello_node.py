#!/usr/bin/env python3
from utils import Session
from optparse import OptionParser
import rospy         
from pepper_nodes.srv import Hello  
import time

        
class HelloNode:
    def __init__(self, ip, port):
        self.session = Session(ip, port)
        #self.motion_service = self.session.get_service("ALMotion")    
        self.animatedSpeechProxy = self.session.get_service("ALAnimatedSpeech")    
        #pass

    def hello(self,req):
        # Imposta il braccio destro in una posizione di saluto
        # self.motion_service.setAngles("RShoulderPitch", -0.5, 0.2)
        # self.motion_service.setAngles("RShoulderRoll", 0.2, 0.2)
        # self.motion_service.setAngles("RElbowYaw", 1.5, 0.2)
        # self.motion_service.setAngles("RElbowRoll", 1.0, 0.2)
        # self.motion_service.setAngles("RWristYaw", 1.0, 0.2)

        # # Aspetta un po' per far visualizzare il gesto di saluto
        # time.sleep(2)

        # # Ripristina la posizione iniziale del braccio destro
        # self.motion_service.setAngles("RShoulderPitch", 0.0, 0.2)
        # self.motion_service.setAngles("RShoulderRoll", 0.0, 0.2)
        # self.motion_service.setAngles("RElbowYaw", 0.0, 0.2)
        # self.motion_service.setAngles("RElbowRoll", 0.0, 0.2)
        # self.motion_service.setAngles("RWristYaw", 0.0, 0.2)
        #print("Gesture Hello")
        
        # dici hello pepper e saluto 
        #self._animated_service.say("Hello I'm Pepper")
        # set the local configuration
        configuration = {"bodyLanguageMode": "contextual", "bodyLanguageKey": "hey_1"}

        # say the text with the local configuration
        self.animatedSpeechProxy.say("Hello, I am Pepper", configuration)
        
        return 'ACK Hello'
    def start(self):
        # Espongo il Servizio start e stop
        rospy.init_node("hello_node")
        #rospy.on_shutdown(self.reset_head_position)
        rospy.Service("hello", Hello, self.hello)
        rospy.spin()
    
if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = HelloNode(options.ip, int(options.port))
        node.start()
    except rospy.ROSInterruptException:
        pass