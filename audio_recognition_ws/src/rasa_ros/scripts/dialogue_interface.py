#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse
from std_msgs.msg import Int16MultiArray, String
from pepper_nodes.srv import Text2Speech, Text2SpeechResponse, Text2SpeechRequest


class Handler:
    '''
    The constructor creates the service proxy object, which is able to make the robot speak
    '''
    def __init__(self):
        self.tts = rospy.ServiceProxy("/tts", Text2Speech)

    '''
    This method calls the Text to Speech service and sends it the desired text to be played.
    '''
    def call(self, text: str):
        msg = Text2SpeechRequest()
        msg.speech = text.lower()
        resp = self.tts(text)
        rospy.loginfo(resp.ack)

class TerminalInterface:
   
        
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal

    '''

    def get_text(self):
        return input("[IN]:  ") 

    def set_text(self,text):
        print("[OUT]:",text)
        
def audio_callback(msg):
    # message = terminal.get_text()
    
    # if message == 'exit': 
      #  break
    try:
        bot_answer = dialogue_service(msg.data) # chiamata servizio
        
        print("[IN]: ", msg.data)
        terminal.set_text(bot_answer.answer)    # deve arrivare a pepper
        
        #handler.call(bot_answer.answer)
        #if bot_answer.answer == 'Bye':
        #    pub.publish('end')
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
pub = rospy.Publisher('state', String, queue_size=1) 
terminal = TerminalInterface()
handler = Handler()
def main():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    
    
    

    
    rospy.Subscriber('voice_txt', String, audio_callback)

    rospy.spin()

    # while not rospy.is_shutdown():
        # message = terminal.get_text()
        

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException:
        pass