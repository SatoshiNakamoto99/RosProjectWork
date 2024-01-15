#!/usr/bin/python3
from utils import Session
from optparse import OptionParser
from pepper_nodes.srv import StartFollowing, StopFollowing
import rospy
from std_msgs.msg import Bool

HUMAN_PRESENCE_TOPIC = '/track/human_presence'

class FollowingNode:

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.session = Session(ip, port)
        self.ba_service  = self.session.get_service("ALBasicAwareness")
       

    def start(self):
        """Start the node.
        """
        rospy.init_node("follow_node")      
        rospy.Service("startFollowing", StartFollowing, self._handle_start_following)
        rospy.Service("stopFollowing", StopFollowing, self._handle_stop_following)
        rospy.on_shutdown(self._handle_shutdown)
        rospy.spin()

    def _handle_start_following(self, req):
        """Start the following head service

        Args:
            req (any): the request (in this case is empty)

        Returns:
            std_msgs/string: ack
        """
        try:
            self.ba_service.setEnabled(True)
        except:
            self.ba_service  = self.session.get_service("ALBasicAwareness")
            self.ba_service.setEnabled(True)
        return "ACK"

    def _handle_stop_following(self, req):
        """Stop the following head service.

        Args:
            req (any): The request (in this case is empty)

        Returns:
            std_msgs/string: ack
        """
        try:
            self.ba_service.setEnabled(False)
        except:
            self.ba_service  = self.session.get_service("ALBasicAwareness")
            self.ba_service.setEnabled(False)
        return "ACK" 

    def _handle_shutdown(self):
        """On shutdown stop the following head service.
        """
        try:
            self.ba_service.setEnabled(False)
        except:
            self.ba_service  = self.session.get_service("ALBasicAwareness")
            self.ba_service.setEnabled(False)

    

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    try:
        node = FollowingNode(options.ip, int(options.port))
        node.start()
    except rospy.ROSInterruptException:
        pass
