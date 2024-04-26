from roboticstoolbox import *
from math import pi
import numpy as np

class GEN3_LITE(ERobot):

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read("/home/riot/kinova_gen3_lite/src/ros_kortex/kortex_description/robots/gen3_lite_gen3_lite_2f.xacro")
    
        super().__init__(
            links,
            name='gen3-lite',
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            manufacturer="Kinova",
            gripper_links=links[7]
        )

        self.home = np.array([0   * pi/180, 
                              -16 * pi/180, 
                              75  * pi/180, 
                              0   * pi/180, 
                              -60 * pi/180, 
                              0   * pi/180])
        
        self.qr = np.zeros(6)
        
        self.addconfiguration(
            "qr", np.array([0, 0, 0, 0, 0, 0])
            
        )
        
        self.addconfiguration(
            "home", np.array([0   * pi/180, 
                              -16 * pi/180, 
                              75  * pi/180, 
                              0   * pi/180, 
                              -60 * pi/180, 
                              0   * pi/180])
            
        )
        


if __name__ == "__main__":
    kinova_lite = GEN3_LITE()
    print(kinova_lite)
    
