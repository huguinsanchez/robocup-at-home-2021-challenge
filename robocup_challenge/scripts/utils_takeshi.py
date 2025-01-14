# -*- coding: utf-8 -*-

from utils import *
import moveit_msgs.msg

import smach
#import matplotlib.pyplot as plt


##### Publishers #####
scene_pub = rospy.Publisher('planning_scene', moveit_msgs.msg.PlanningScene, queue_size = 5)

##### KNOWN LOCATIONS #####
kl_mess1 = [1.04, 0.3, 90]
kl_tray =  [2.318411366833172, 0.09283744344925589, -90]
kl_box1 =  [-0.04168256822546347, 2.427268271720426, -90]
kl_table1 = [1.04, 1.2, 90]
kl_table2= [0 , 1.2,90]
kl_shelf = [2.08, 3.6, 1.7]
kl_deliver = [0.8, 3.4, 180]
kl_l_deliver = [0.6, 3, 180]
kl_r_deliver = [0.6, 4.2, 180]
kl_door=[2.68,1.52, 1.7]


##### ARM #####
arm_grasp_from_above = [0.19263830140116414,
 -2.2668981568652917,
 -0.007358947463759424,
 -0.9939144210462025,
 -0.17365421548386273,
 0.0]
arm_grasp_from_above_table = [0.41349380130577407,
 -1.671584191489468,
 -0.02774372779356371,
 -1.5952436225825641,
 0.22362492457833927,
 0.0]
arm_grasp_table=[0.41349380130577407,
 -1.671584191489468,
 -0.02774372779356371,
 0.0,
 0.22362492457833927,
 0.0]
arm_grasp_floor = [-1.5151551103007697e-05,
 -2.4,
 -0.2620865401925543,
 0.7019536624449207,
 0.20120924571306453,
 0.0]
arm_train_pose = [0.033749214744071214,
 -2.1204421063180217,
 -1.3982377978814715,
 -1.7296544561013807,
 2.135675364707808,
 0.0]
arm_ready_to_place = [0.03999320441056991,
 -0.4729690540086997,
 0.19361475012179108,
 -1.5269847787383313,
 -0.009753879176134461,
 0.0]
<<<<<<< HEAD
arm_grasp_shelf_hl = [0.3,
=======
arm_grasp_shelf_hl = [0.2,
>>>>>>> df952a40441f4a2e15bf23396741cfa313065deb
 -0.4,
 0.0,
 -1.3,
 0.0,
 0.0]
<<<<<<< HEAD
 #arm_ready_to_deliver = [0.03999320441056991,
""" -0.4729690540086997,
 0.19361475012179108,
 -1.5269847787383313,
 -0.009753879176134461,
 0.0]"""

=======
arm_ready_to_deliver = [0.43999320441056991,
 -0.4729690540086997,
 0.19361475012179108,
 -1.5269847787383313,
 -0.009753879176134461,
 0.0]
>>>>>>> df952a40441f4a2e15bf23396741cfa313065deb

##### GRASP 
ungrasped=[-0.00047048998088961014,
 -0.03874743486886725,
 -0.04825256513113274,
 0.038463464485261056,
 -0.03874743486886725]
grasped=[0.12814103131904275,
 -0.30672794406396453,
 0.21972794406396456,
 0.13252877558892262,
 -0.30672794406396453]



########## Base Class for takeshi state machine ##########
class Takeshi_states(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succ', 'failed', 'tries'])
        self.tries = 1
        self.max_tries = 5
    
    #This function must be overwritten
    def takeshi_run(self):
        print("Takeshi execute")
        success = True        
        return success

    def execute(self, userdata):
        #rospy.loginfo('STATE : INITIAL')
        print("Excecuting Takeshi State: " + self.__class__.__name__)
        print('Try', self.tries, 'of 5 attempts')
        succ = self.takeshi_run()
        if succ: 
            print('success')            
            return 'succ'
        else:
            print('failed')
            self.tries += 1
            if self.tries > self.max_tries:
                return 'tries'
            else:
                return 'failed'


########## Util Functions ##########

def cart2spher(x,y,z):
    ro= np.sqrt(x**2+y**2+z**2)
    th=np.arctan2(y,x)
    phi=np.arctan2((np.sqrt(x**2+y**2)),z)
    return np.asarray((ro,th,phi))


def spher2cart(ro,th,phi):
    x= ro * np.cos(th)* np.sin(phi)
    y= ro * np.sin(th)* np.sin(phi)
    z= ro*  np.cos(th)
    return np.asarray((x,y,z))

def point_2D_3D(points_data, px_y, px_x):
    ##px pixels /2D world  P1 3D world
    P = np.asarray((points_data[px_y, px_x]['x'], points_data[px_y, px_x]['y'], points_data[px_y, px_x]['z']))
    return P
