#!/usr/bin/env python3
from rasa_ros.srv import Dialogue, DialogueResponse

import rospy
import requests


def handle_service(req):
    input_text = req.input_text   

    # Get answer        
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
    message = {
        "sender": 'bot',
        "message": input_text
    }

    r = requests.post(get_answer_url, json=message)
    response = DialogueResponse()
    response.answer = ""
    for i in r.json():
        response.answer += i['text'] + ' ' if 'text' in i else ''

    return response

def dialogue_start():

    # Server Initialization
    rospy.init_node('dialogue_service')

    s = rospy.Service('dialogue_server',
                        Dialogue, handle_service)

    rospy.logdebug('Dialogue server READY.')
    print("""
    ______  ___   _____  ___   ______ _____  ___ ________   __
    | ___ \/ _ \ /  ___|/ _ \  | ___ \  ___|/ _ \|  _  \ \ / /
    | |_/ / /_\ \\ `--./ /_\ \ | |_/ / |__ / /_\ \ | | |\ V / 
    |    /|  _  | `--. \  _  | |    /|  __||  _  | | | | \ /  
    | |\ \| | | |/\__/ / | | | | |\ \| |___| | | | |/ /  | |  
    \_| \_\_| |_/\____/\_| |_/ \_| \_\____/\_| |_/___/   \_/  
                                                            
                                                            """)
    print('[CHATBOT INTERFACE] ready.')
    rospy.spin()


if __name__ == '__main__':
    try: 
        dialogue_start()
    except rospy.ROSInterruptException as e:
        pass
