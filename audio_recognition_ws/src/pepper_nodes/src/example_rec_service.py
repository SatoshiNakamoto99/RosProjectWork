#!/usr/bin/python3
from utils import Session
from optparse import OptionParser
import rospy         
from pepper_nodes.srv import startFollowing, stopFollowing    
        
# class FollowNode:
#     def __init__(self, ip, port):
#         # self.session = Session(ip, port)
#         # self.tracker_service = self.session.get_service("ALTracker")
#         # self.motion_service = self.session.get_service("ALMotion")        
#         pass

#     def startFollow(self, req):
#         # self.tracker_service.registerTarget(self._target_name, self._faceSize)
#         # self.tracker_service.setMode(self._mode)
#         # # Subscribe to the necessary modules
#         # self.motion_service.setStiffnesses(self._mode, 1.0)  # Enable head movement
#         # #self.reset_head_position()
#         # # Start tracking
#         # self.tracker_service.track(self._target_name)
        
#         print("start following")
#         return 'ACK Start Follow'
        
#     def stop_following(self, req):
#         # # Stop the tracker when done
#         # self.tracker_service.stopTracker()
#         # self.tracker_service.unregisterAllTargets()
#         # self.reset_head_position()
#         # self.motion_service.setStiffnesses("Head", 0.0)  # Disable head movementù
        
#         print("stop following")
#         return 'ACK Stop Follow'
        
#     def reset_head_position(self):
#         self.motion_service.setAngles("Head", [0.0, -0.3], 0.2)
        
#     def start(self):
#         rospy.init_node("follow_node")
#         #rospy.on_shutdown(self.stop_following)
#         rospy.Service("startFollowing", startFollowing, self.startFollow)
#         rospy.Service("stopFollowing", stopFollowing, self.stop_following)
#         rospy.spin()
    
# if __name__ == "__main__":
#     parser = OptionParser()
#     parser.add_option("--ip", dest="ip", default="10.0.1.207")
#     parser.add_option("--port", dest="port", default=9559)
#     (options, args) = parser.parse_args()

#     try:
#         node = FollowNode(options.ip, int(options.port))
#         node.start()
#     except rospy.ROSInterruptException:
#         pass
    
# # #!/usr/bin/python3
#!/usr/bin/python3
from utils import Session
from optparse import OptionParser
import rospy
from pepper_nodes.srv import *

'''
This class implements a ROS node used to controll the Pepper posture
'''
class WakeUpNode:
    
    '''
    The costructor creates a session to Pepper and inizializes the services
    '''
    def __init__(self, ip, port):
        # self.ip = ip
        # self.port = port
        # self.session = Session(ip, port)
        # self.motion_proxy = self.session.get_service("ALMotion")
        # self.posture_proxy = self.session.get_service("ALRobotPosture")
        pass
    '''
    This method calls the ALMotion service and sets the robot to rest position
    '''
    def rest(self, *args):
        # try:
        #     self.motion_proxy.rest()
        # except:
        #     self.motion_proxy = self.session.get_service("ALMotion")
        #     self.motion_proxy.rest()
        # return "ACK"
        print("rest")
    '''
    This method calls the ALMotion and ALRobotPosture services and it sets motors on and then it sets the robot posture to initial position
    '''
    def wakeup(self, *args):
        # try:
        #     self.motion_proxy.wakeUp()
        #     self.stand()
        # except:
        #     self.motion_proxy = self.session.get_service("ALMotion")
        #     self.posture_proxy = self.session.get_service("ALRobotPosture")
        #     self.motion_proxy.wakeUp()
        #     self.stand() 
        print("wakeup")        

        return "ACK"   
    
    '''
    This method sets the robot posture to "StandInit" posture
    '''
    def stand(self, *args):
        #self.posture_proxy.goToPosture("StandInit", 0.5)
        pass
    '''
    Starts the node and wake up the robot
    '''
    def start(self):
        rospy.init_node("wakeup_node")
        #self.wakeup()
        #self.stand()        
        rospy.Service("wekup", WakeUp, self.wakeup)
        rospy.Service("rest", Rest, self.rest)
        rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = WakeUpNode(options.ip, int(options.port))
        node.start()
    except rospy.ROSInterruptException:
        pass