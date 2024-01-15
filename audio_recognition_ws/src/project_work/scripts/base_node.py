from config import *
import rospy
from std_msgs.msg import Bool
from threading import Lock

class BaseNode(object):
    def __init__(self) -> None:
        self._persistent_services = dict()  # dict str -> Touple[func, module]
        
        
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

    def _handle_shutdown(self):
        """On killing the application, stop all the service that can be running on 
        Pepper, so the following and the movement. After that set Pepper in rest 
        position.
        """
        if self._verbose:
            print('[Node] Shutdown signal received. Stopping the application')

        # Close all persistent service
        for ps in self._persistent_services:
            self._persistence_service_close(ps)
    
