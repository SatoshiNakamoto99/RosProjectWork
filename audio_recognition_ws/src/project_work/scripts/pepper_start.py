#!/usr/bin/python3
from config import *
from base_node import BaseNode
from pepper_nodes.srv import WakeUp, StartFollowing
import rospy
if __name__ == '__main__':
    _verbose = True
    if ON_PEPPER:
        caller = BaseNode()
        rospy.init_node('Start Node', anonymous=True)
        rospy.on_shutdown(caller._handle_shutdown)
        if _verbose:
            print('[Starter Node] Pepper wakeup')
        caller._service_call('wakeup', WakeUp)

        if _verbose:
            print('[Starter Node] Pepper startfollowing')
        caller._service_call('startFollowing', StartFollowing)
        
    if _verbose:
        print('[Starter Node] Starting done.')
              
    