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