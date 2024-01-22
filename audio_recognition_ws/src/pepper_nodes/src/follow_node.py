#!/usr/bin/python3
# from utils import Session
# from optparse import OptionParser
# import rospy

# class FollowNode:
#     def __init__(self, ip, port):
#         self.session = Session(ip, port)
#         self.tracker_service = self.session.get_service("ALTracker")
#         self.motion_service = self.session.get_service("ALMotion")
#         self.reset_head_position()
#         self._reset = False

#     def follow_person(self):
#         # Set up tracker parameters
#         target_name = "Face"
#         faceSize = 0.6  # Diameter of the person to track
#         self.tracker_service.registerTarget(target_name, faceSize)

#         # Set tracker mode to "Head" for head tracking
#         mode = "Head"
#         self.tracker_service.setMode(mode)

#         # Subscribe to the necessary modules
#         self.motion_service.setStiffnesses("Head", 1.0)  # Enable head movement

#         # Start tracking
#         self.tracker_service.track(target_name)
        
#         # if the person is not detected, the robot will rotate on itself

#         # Initialize rospy node
#         rospy.init_node("follow_person_node")
#         rospy.on_shutdown(self.reset_head_position)
#         while not rospy.is_shutdown():
#             if not self.tracker_service.isTargetLost():
#                 rospy.sleep(1.0)  # Waiting for 1 second
                
#             else:
                
#                 self.reset_head_position()
                    
                    
        
#         # try:
#         #     rospy.spin()
#         # except rospy.ROSInterruptException:
#         #     pass

#     def stop_following(self):
#         # Stop the tracker when done
#         self.reset_head_position()
#         self.tracker_service.stopTracker()
#         self.tracker_service.unregisterAllTargets()
#         self.motion_service.setStiffnesses("Head", 0.0)  # Disable head movement
#         #print("Stopped tracking.")
#         #self.reset_head_position()
        
#     def reset_head_position(self):
#         # Reset head position
#         self.motion_service.setAngles("Head", [0.0, -0.3], 0.2)
#         #print("Head reset.")
        
# if __name__ == "__main__":
#     parser = OptionParser()
#     parser.add_option("--ip", dest="ip", default="10.0.1.230")
#     parser.add_option("--port", dest="port", default=9559)
#     (options, args) = parser.parse_args()

#     try:
#         follow_node = FollowNode(options.ip, int(options.port))
#         follow_node.follow_person()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         follow_node.stop_following()
#         #pass

# ######## FOLLOWING NODE ARTIGIANALE ########

# # from utils import Session
# # from optparse import OptionParser
# # import rospy
# # from std_msgs.msg import Bool

# # HUMAN_PRESENCE_TOPIC ='/track/human_presence'

# # class FollowNode:
# #     def __init__(self, ip, port):
# #         self.session = Session(ip, port)
# #         self.tracker_service = self.session.get_service("ALTracker")
# #         self.motion_service = self.session.get_service("ALMotion")
        
# #         self._engagement = False
# #         self._reset = False
# #         self._previus_human_presence = False
# #         self._human_presence = False
# #         self._target_name = "Face"
# #         self._faceSize = 0.6  # 
# #         self._mode = "Head"
        
# #     def startFollow(self):
# #         self.tracker_service.registerTarget(self._target_name, self._faceSize)
# #         self.tracker_service.setMode(self._mode)
# #         # Subscribe to the necessary modules
# #         self.motion_service.setStiffnesses(self._mode, 1.0)  # Enable head movement
# #         #self.reset_head_position()
# #         # Start tracking
# #         self.tracker_service.track(self._target_name)
        
# #     def stop_following(self):
# #         # Stop the tracker when done
# #         self.reset_head_position()
# #         self.tracker_service.stopTracker()
# #         self.tracker_service.unregisterAllTargets()
# #         self.motion_service.setStiffnesses("Head", 0.0)  # Disable head movement
        
# #     def reset_head_position(self):
# #         self.motion_service.setAngles("Head", [0.0, -0.3], 0.2)
        
        
# #     def _handle_presence(self, presence):
# #         """
# #         Callback function for the human presence topic.
        
# #         Sets the human presence flag to the value written on the topic.
        
# #         Human presence is set to True if the value is True, False otherwise.
        
# #         Args:
# #             presence (Bool): The human presence is written on topic. It is True if human is present, False otherwise.
# #         """
# #         self._previus_human_presence = self._human_presence
# #         self._human_presence = presence.data
        
# #         if not self._previus_human_presence and self._human_presence:
# #             self._engagement = True
# #             self._reset = False
        
# #         if  self._previus_human_presence and not self._human_presence:
# #             self._reset = True
# #         # if self._verbose:
# #         #     print("[AUDIO NODE] Human presence: {}".format(self._human_presence))
    
# #     def start(self,human_presence_topic, rate_value = 1):
        
# #         # Initialize rospy node
# #         rospy.init_node("follow_person_node")
# #         rospy.Subscriber(human_presence_topic, Bool, self._handle_presence)
# #         rate = rospy.Rate(rate_value)
# #         while not rospy.is_shutdown():
# #             rate.sleep()
# #             print("engagement: ", self._engagement)
# #             print("reset: ", self._reset)
# #             if self._engagement:
# #                 print("start follow")
# #                 self.startFollow()
# #                 self._engagement = False
# #             elif self._reset:
# #                 print("stop follow")
# #                 self.stop_following()
# #                 self._reset = False
                
#         #Per generalizzare dovrei esporre solo i servizi start e stop 
        
#         # rospy.init_node("follow_person_node")
#         # rospy.Service("startFollowing", startFollowing, self.startFollow)
#         # rospy.Service("stopFollowing", stopFollowing, self.stop_following)
#         # rospy.spin()
    
# if __name__ == "__main__":
#     parser = OptionParser()
#     parser.add_option("--ip", dest="ip", default="10.0.1.230")
#     parser.add_option("--port", dest="port", default=9559)
#     (options, args) = parser.parse_args()
    
#     try:
#         follow_node = FollowNode(options.ip, int(options.port))
#         follow_node.start(HUMAN_PRESENCE_TOPIC)
#     except rospy.ROSInterruptException:
#         pass
     
     
############# GENERAL FOLLOWING NODE #############        

from utils import Session
from optparse import OptionParser
import rospy         
from pepper_nodes.srv import startFollowing, stopFollowing    
        
class FollowNode:
    def __init__(self, ip, port, target_name = 'Face', faceSize=0.6 , mode = "Head"):
        self.session = Session(ip, port)
        self.tracker_service = self.session.get_service("ALTracker")
        self.motion_service = self.session.get_service("ALMotion")
        self._target_name = target_name
        self._faceSize = faceSize   
        self._mode = mode
         
        #pass

    def startFollow(self, req):
        self.tracker_service.registerTarget(self._target_name, self._faceSize)
        self.tracker_service.setMode(self._mode)
        # Subscribe to the necessary modules
        self.motion_service.setStiffnesses(self._mode, 1.0)  # Enable head movement
        #self.reset_head_position()
        # Start tracking
        self.tracker_service.track(self._target_name)
        
        print("start following")
        return 'ACK Start Follow'
    
    def stop(self):
        try:
            self.tracker_service.stopTracker()
            self.tracker_service.unregisterAllTargets()
            self.reset_head_position()
        except:
            pass
    
    def stop_following(self, req):
        # Stop the tracker when done
        # self.tracker_service.stopTracker()
        # self.tracker_service.unregisterAllTargets()
        # self.reset_head_position()
        
        self.stop()
        print("stop following")
        return 'ACK Stop Follow'
    
    
        
    def reset_head_position(self):
        self.motion_service.setAngles("Head", [0.0, -0.3], 0.2)
        
    def start(self):
        # Espongo il Servizio start e stop
        rospy.init_node("follow_node")
        rospy.on_shutdown(self.stop)
        rospy.Service("startFollowing", startFollowing, self.startFollow)
        rospy.Service("stopFollowing", stopFollowing, self.stop_following)
        rospy.spin()
    
if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = FollowNode(options.ip, int(options.port))
        node.start()
    except rospy.ROSInterruptException:
        pass