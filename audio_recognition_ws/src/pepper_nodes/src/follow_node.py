#!/usr/bin/python3
from utils import Session
from optparse import OptionParser
import rospy

class FollowNode:
    def __init__(self, ip, port):
        self.session = Session(ip, port)
        self.tracker_service = self.session.get_service("ALTracker")
        self.motion_service = self.session.get_service("ALMotion")

    def follow_person(self):
        # Set up tracker parameters
        target_name = "People"
        diameter = 0.4  # Diameter of the person to track
        self.tracker_service.registerTarget(target_name, diameter)

        # Set tracker mode to "Head" for head tracking
        mode = "Head"
        self.tracker_service.setMode(mode)

        # Subscribe to the necessary modules
        self.motion_service.setStiffnesses("Head", 1.0)  # Enable head movement

        # Start tracking
        self.tracker_service.track(target_name)

        # Initialize rospy node
        rospy.init_node("follow_person_node")
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def stop_following(self):
        # Stop the tracker when done
        self.tracker_service.stopTracker()
        self.tracker_service.unregisterAllTargets()
        self.motion_service.setStiffnesses("Head", 0.0)  # Disable head movement
        #self.reset_head_position()
        
    def reset_head_position(self):
        # Reset head position
        self.motion_service.setAngles("Head", [0.0, 0.0], 0.5)
        
if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.230")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        follow_node = FollowNode(options.ip, int(options.port))
        follow_node.follow_person()
    except rospy.ROSInterruptException:
        pass
    finally:
        follow_node.stop_following()

