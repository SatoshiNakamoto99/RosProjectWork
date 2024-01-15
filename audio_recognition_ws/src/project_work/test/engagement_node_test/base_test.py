import os
import rospy 
import ros_numpy

class BaseTest(object):
    """
    Node implementing the base test of module
    """

    def __init__(self):
        self._output = list()
        self._groundtruth = None

    def _setup(self,scene_path):
        """
        Load groundtruth from file stored in specific scene_path.
        Args:
            scene_path (String)
        """
        with open(os.path.join(scene_path,"groundtruth.txt"),"r") as f:
            g = f.read()
        labels = g.split(",")
        # to manage test with only images without faces
        if len(labels)==1 and labels[0]=="False":
            self._groundtruth = list() 
            return
        self._groundtruth = [x=="True" for x in labels]

    def _cleanup(self):
        """
        Reset groundtruth and detector output
        """
        self._groundtruth = None
        self._output = list()
    
    def _test(self):
        """
        Test the correctness of the detector output
        """
        # check if the sequece 
        if self._output == self._groundtruth:
            print("Passed")
        else:
            print("Failed")
        
    def _get_output(self):
        """
        Return detector output
        """
        return self._output
    
    def _set_output(self,output):
        """
        Set detector output
        """
        self._output.append(output)
    
    def _get_groundtruth(self):
        """
        Return groundtruth
        """
        return self._groundtruth
        