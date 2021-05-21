#!/usr/bin/env python

from utils_takeshi import *
from utils import *
from geometry_msgs.msg import PointStamped
from takeshi_tools.nav_tool_lib_OSS import nav_module

def segment_floor():
    image_data = rgbd.get_image()
    points_data = rgbd.get_points()

    P1 = point_2D_3D(points_data, -1, -200)
    P2 = point_2D_3D(points_data, -1, 200)
    P3 = point_2D_3D(points_data, -150, -320)

    V1 = P1 - P2
    V2 = P3 - P2
    nx, ny, nz = np.cross(V2, V1)
    print('look at the phi angle  in normal vector', np.rad2deg(cart2spher(nx, ny, nz))[2] - 90)
    trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
    euler = tf.transformations.euler_from_quaternion(rot)
    print(np.rad2deg(euler)[1], ' if this degree is not the same as head tilt plane was not found')
       
    mask = np.zeros((image_data.shape))
    plane_mask = np.zeros((image_data.shape[0], image_data.shape[1]))
    mask[:, :, 0] = points_data['x'] - P1[0]
    mask[:, :, 1] = points_data['y'] - P1[1]
    mask[:, :, 2] = points_data['z'] - P1[2]
    
    for i in range (image_data.shape[0]):
        for j in range (image_data.shape[1]):
            plane_mask[i, j] = -np.dot(np.asarray((nx, ny, nz, )), mask[i, j])
    plane_mask = plane_mask - np.min(plane_mask)
    plane_mask = plane_mask * 256 / np.max(plane_mask)
    plane_mask.astype('uint8')

    ret, thresh = cv2.threshold(plane_mask, 3, 255, 0)

    cv2_img = plane_mask.astype('uint8')
    img = plane_mask.astype('uint8')
    _, contours, hierarchy = cv2.findContours(thresh.astype('uint8'), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    i = 0
    cents = []
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        
        if area > 100 and area < 50000 :                        
            boundRect = cv2.boundingRect(contour)            
            img = cv2.rectangle(img, (boundRect[0], boundRect[1]), (boundRect[0] + boundRect[2], boundRect[1] + boundRect[3]), (255, 0, 0), 2)
            # calculate moments for each contour
            xyz = []
                
            for jy in range (boundRect[0], boundRect[0] + boundRect[2]):
                for ix in range(boundRect[1], boundRect[1] + boundRect[3]):
                    xyz.append(np.asarray((points_data['x'][ix, jy], points_data['y'][ix, jy], points_data['z'][ix, jy])))
            xyz = np.asarray(xyz)
            cent = xyz.mean(axis = 0)
            
            cents.append(cent)

            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(img, "centroid_" + str(i) + "_" + str(cX) + ',' + str(cY), (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cents = np.asarray(cents)    
    return cents  


def static_tf_publish(cents):
    for  i, cent  in enumerate(cents):
        x, y, z = cent
        print(cent,i)
        broadcaster.sendTransform((x, y, z), rot, rospy.Time.now(), 'Closest_Object' + str(i), "head_rgbd_sensor_link")
        rospy.sleep(0.2)
        xyz_map, cent_quat = listener.lookupTransform('/map', 'Closest_Object' + str(i), rospy.Time(0))
        print(xyz_map[0],i)
        map_euler = tf.transformations.euler_from_quaternion(cent_quat)
        rospy.sleep(0.2)
        static_transformStamped = TransformStamped()

        ##FIXING TF TO MAP ( ODOM REALLY)    
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "static" + str(i)
        static_transformStamped.transform.translation.x = float(xyz_map[0])
        static_transformStamped.transform.translation.y = float(xyz_map[1])
        static_transformStamped.transform.translation.z = float(xyz_map[2])
        static_transformStamped.transform.rotation.x = 0    
        static_transformStamped.transform.rotation.y = 0    
        static_transformStamped.transform.rotation.z = 0    
        static_transformStamped.transform.rotation.w = 1    

        tf_static_broadcaster.sendTransform(static_transformStamped)
    return True


def callback_message(data):
     print("##################")
     print("##################\n")
     print("message "+ str(count_message)+": "+str(data.data))
     print("\n##################")
     print("##################")





########## Functions for takeshi states ##########

def segment_shelf(chan):
    image_data=rgbd.get_image()
    points_data = rgbd.get_points()

    mask=np.zeros((image_data.shape))
    plane_mask=np.zeros((image_data.shape[0],image_data.shape[1]))

    plane_mask=image_data[:,:,chan]

    ret,thresh = cv2.threshold(image_data[:,:,2],240,255,200)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    img=image_data[:,:,0]
    _,contours, hierarchy = cv2.findContours(thresh.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)

        if area > 200 and area < 25000 :
            print('contour',i,'area',area)

            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            print boundRect
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            # calculate moments for each contour
            xyz=[]


            for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                    xyz.append(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
            xyz=np.asarray(xyz)
            cent=xyz.mean(axis=0)
            cents.append(cent)
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
            cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
            print ('cX,cY',cX,cY)
    cents=np.asarray(cents)
    
    return (cents)


def static_tf_publish(cents):
    global rot
    for  i, cent  in enumerate(cents):
        x, y, z = cent
        broadcaster.sendTransform((x, y, z), rot, rospy.Time.now(), 'Closest_Object' + str(i), "head_rgbd_sensor_link")
        rospy.sleep(0.2)
        xyz_map, cent_quat = listener.lookupTransform('/map', 'Closest_Object' + str(i), rospy.Time(0))
        map_euler = tf.transformations.euler_from_quaternion(cent_quat)
        rospy.sleep(0.2)
        static_transformStamped = TransformStamped()

        ##FIXING TF TO MAP ( ODOM REALLY)    
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "static" + str(i)
        static_transformStamped.transform.translation.x = float(xyz_map[0])
        static_transformStamped.transform.translation.y = float(xyz_map[1])
        static_transformStamped.transform.translation.z = float(xyz_map[2])
        static_transformStamped.transform.rotation.x = 0    
        static_transformStamped.transform.rotation.y = 0    
        static_transformStamped.transform.rotation.z = 0    
        static_transformStamped.transform.rotation.w = 1    

        tf_static_broadcaster.sendTransform(static_transformStamped)
    return True


def add_object(name, size, pose, orientation):
    p = PoseStamped()
    p.header.frame_id = "map"       # "head_rgbd_sensor_link"
    
    p.pose.position.x = pose[0]
    p.pose.position.y = pose[1]
    p.pose.position.z = pose[2]

    p.pose.orientation.x = orientation[0] * np.pi
    p.pose.orientation.w = orientation[1] * np.pi

    scene.add_box(name, p, size)


def publish_scene():
    add_object("shelf", [1.5, 0.04, 0.4], [2.5, 4.7, 0.78], [0.5, 0.5])
    add_object("shelf1", [1.5, 0.04, 0.4], [2.5, 4.7, 0.49], [0.5, 0.5])
    add_object("shelf2", [1.5, 0.04, 0.4], [2.5, 4.7, 0.18], [0.5, 0.5])
    add_object("shelf_wall", [1, 1, 0.04], [2.5, 4.9, 0.5], [0.5, 0.5])
    add_object("shelf_wall1", [.04, 1, 0.4], [2.7, 4.9, 0.5], [0.5, 0.5])
    add_object("shelf_wall2", [.04, 1, 0.4], [1.8, 4.9, 0.5], [0.5, 0.5])
    add_object("table_big", [1.7, 0.13, 0.7], [0.95, 1.9, 0.34], [0.5, 0.5])
    add_object("table_small", [0.5, 0.01, 0.4], [0.1, 1.9, 0.61], [0.5, 0.5])
    add_object("table_tray", [0.65, 0.01, 0.7], [1.8, -0.65, 0.4], [0.5, 0.5])    
    return True







########## Clases derived from Takeshi_states, please only define takeshi_run() ##########

##### Define state INITIAL #####
#Estado inicial de takeshi, neutral
class Initial(Takeshi_states):
    def takeshi_run(self):          
        scene.remove_world_object()
        #Takeshi neutral
        move_hand(0)
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ = head.go()             
        return succ

class go_to_door(Takeshi_states):
    def takeshi_run(self):
    	#self.msg = message.get_data()
        #self.whom = str(self.msg).split('"')
        #rospy.loginfo("Message recieved - object goal: %s", self.whom[1])
        goal_x , goal_y, goal_yaw = kl_door
        try:
            omni_base.getClose(goal_x, goal_y, goal_yaw,20)
            return True
        except:
            return False

class looking_for_objects(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        global  rot, trans
        head_val = head.get_current_joint_values()
        head_val[0] = np.deg2rad(0)
        head_val[1] = np.deg2rad(-40)        
        head.go(head_val)
        cents = segment_floor()
        if len(cents)<1:
            return 'failed'
        print("cents")
        print(len(cents),cents)
        pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=10)
        trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
        print("trans")
        print(trans)
        static_tf_publish(cents)
        print("after_publish")
        rospy.sleep(0.2)

        point=PointStamped()
        point.header.frame_id='map'
        trans_cents = []
        
        for i, cent in enumerate(cents):
            trans_map, _ = listener.lookupTransform('/map', 'static' + str(i), rospy.Time(0))
            point.header.stamp=rospy.Time(0)
            point.point.x=trans_map[0]
            point.point.y=trans_map[1]
            point.point.z=trans_map[2]
            pub.publish(point)
            rospy.sleep(0.2)

        succ=True
    
        if self.tries==3:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'

class go_to_shelf(Takeshi_states):
    def takeshi_run(self):
        goal_x , goal_y, goal_yaw = kl_shelf
        try:
            omni_base.getClose(goal_x, goal_y, goal_yaw,40)
            return True
        except:
            return False



##### Define state SCAN_SHELF #####
#Va al shelf, voltea  la cabeza y escanea el estante
class Scan_shelf_hl(Takeshi_states):
    def takeshi_run(self):     
        global cents, rot, trans
        wb=whole_body.get_current_joint_values()
        wb[3]=wb[3]+0.25
        wb[4]=wb[4]-2
        whole_body.go(wb)
        head_val = head.get_current_joint_values()
        head_val[0] = np.deg2rad(0)
        head_val[1] = np.deg2rad(-25)        
        head.go(head_val)
        rospy.sleep(.2)       
        
        trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        
        cents = segment_shelf(0)
        if len (cents==0):
            cents=segment_shelf(1)
            if len (cents==0):
                cents=segment_shelf(2)
        static_tf_publish(cents)
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ = head.go()
        return succ

##### Define state PRE_GRASP_SHELF_HL #####
#Acomoda el brazo, abre la garra y se acerca al objeto para grasp
class Pre_grasp_shelf_hl(Takeshi_states):
    
    def takeshi_run(self):
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
    
        global closest_cent 
        global cents              
    
        trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        print("centroids wrt head " +str(cents))
        publish_scene()
        cents= segment_shelf(2)
        trans_cents = []        
        for i, cent in enumerate(cents):
            trans_map, _ = listener.lookupTransform('/map', 'static' + str(i), rospy.Time(0))
            trans_cents.append(trans_map)
        
        np.linalg.norm(np.asarray(trans_cents) - trans , axis = 1)
        closest_cent = np.argmin(np.linalg.norm(np.asarray(trans_cents) - trans , axis = 1))
        print("Closest Cent " + str(closest_cent))
        publish_scene()
        move_hand(1)
        arm.set_joint_value_target(arm_grasp_table)
        succ=arm.go()
        return succ
        
##### Define state GRASP_SHELF_HL #####
#Se acerca mas al objeto y cierra la garra
class Grasp_shelf_hl(Takeshi_states):
    def takeshi_run(self):
        global trans_hand, closest_cent, cents
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static'+str(closest_cent), rospy.Time(0))        
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] - 0.15
        wb[1] += trans_hand[1]
        wb[3] += trans_hand[0]
        whole_body.go(wb)
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static'+str(closest_cent), rospy.Time(0))
        scene.remove_world_object()
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] - 0.06
        wb[1] += trans_hand[1]
        wb[3] += trans_hand[0]
        succ = whole_body.go(wb)
        
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static'+str(closest_cent), rospy.Time(0))
        scene.remove_world_object()
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] - 0.06
        wb[1] += trans_hand[1]
        wb[3] += trans_hand[0]
        whole_body.go(wb)
        move_hand(0)
        a = gripper.get_current_joint_values()
        if np.linalg.norm(a - np.asarray(grasped))  >  (np.linalg.norm(a - np.asarray(ungrasped))):
            print ('grasp seems to have failed')
            return False
        else:
            print('super primitive fgrasp detector points towards succesfull ')
            wb = whole_body.get_current_joint_values()
            wb[0] += trans_hand[2] - 0.3
            succ = whole_body.go(wb)
            #Takeshi neutral
            arm.set_named_target('go')
            arm.go()
            head.set_named_target('neutral')
            head.go()
            return succ


##### Define state GO_BOX #####
# Se mueve hacia la posicion de entrega 
class Go_deliver_center(Takeshi_states):
    def takeshi_run(self):
        goal_x, goal_y, goal_yaw =  kl_deliver #Known location friends
        succ = move_base_goal(goal_x, goal_y - 0.1, 180)
        publish_scene()
        return succ

class Listen_deliver_goal(Takeshi_states):
    def takeshi_run(self):
        #self.msg = message.get_data()
        #self.whom = str(self.msg).split('"')
        ##self.whom[1].strip('"')
        #rospy.loginfo("Message recieved - person goal: %s", self.whom[1])

        self.whom = "left person"
        if self.whom== "left person":
            rospy.loginfo("Delivering to the %s", self.whom)
            goal_x, goal_y, goal_yaw =  kl_l_deliver #Known location friends_left
            succ = move_base_goal(goal_x, goal_y - 0.1, 180)
            publish_scene()
            return succ
        else:
            if self.whom[1] == "right person":
                rospy.loginfo("Delivering to the %s", self.whom)
                goal_x, goal_y, goal_yaw =  kl_r_deliver #Known location friends_right
                succ = move_base_goal(goal_x, goal_y - 0.1, 180)
                publish_scene()
                return succ
            else: 
                print("I don't know to whom should I deliver the object")


##### Define state DELIVER #####
# Suelta el objeto
class Deliver(Takeshi_states):
    def takeshi_run(self):
        arm.set_joint_value_target(arm_ready_to_deliver)
        arm.go()
        wb = whole_body.get_current_joint_values()
        move_hand(1)
        whole_body.set_joint_value_target(wb)
        succ = whole_body.go()
        scene.remove_world_object()
        return succ

#Initialize global variables and node
def init(node_name):
    global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd, message, omni_base, count_message 
    rospy.init_node(node_name)
    rospy.sleep(35)
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGBD()
    message = Message()
    omni_base=nav_module()
    count_message=0
    rospy.Subscriber("/message", String, callback_message)


   
#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0

    with sm:
        #State machine for grasping from shelf
        smach.StateMachine.add("INITIAL",               Initial(),              transitions = {'failed':'INITIAL',              'succ': 'GO_TO_DOOR',           'tries':'END'}) 
        smach.StateMachine.add("GO_TO_DOOR",            go_to_door(),           transitions = {'failed':'GO_TO_DOOR',           'succ': 'LOOKING_FOR_OBJECTS',  'tries':'END'}) 
        smach.StateMachine.add("LOOKING_FOR_OBJECTS",   looking_for_objects(),  transitions = {'failed':'LOOKING_FOR_OBJECTS',  'succ': 'GO_TO_SHELF',          'tries':'END'}) 
        smach.StateMachine.add("GO_TO_SHELF",           go_to_shelf(),          transitions = {'failed':'GO_TO_SHELF',          'succ': 'SCAN_SHELF_HL',        'tries':'END'}) 
        smach.StateMachine.add("SCAN_SHELF_HL",         Scan_shelf_hl(),        transitions = {'failed':'SCAN_SHELF_HL',        'succ': 'PRE_GRASP_SHELF_HL',   'tries':'END'}) 
        smach.StateMachine.add('PRE_GRASP_SHELF_HL',    Pre_grasp_shelf_hl(),   transitions = {'failed':'PRE_GRASP_SHELF_HL',   'succ': 'GRASP_SHELF_HL',       'tries':'END'}) 
        smach.StateMachine.add('GRASP_SHELF_HL',        Grasp_shelf_hl(),       transitions = {'failed':'GRASP_SHELF_HL',       'succ': 'GO_DELIVER_CENTER',    'tries':'END'}) 
        smach.StateMachine.add('GO_DELIVER_CENTER',     Go_deliver_center(),    transitions = {'failed':'GO_DELIVER_CENTER',    'succ': 'LISTEN_DELIVER_GOAL',  'tries':'END'})
        smach.StateMachine.add('LISTEN_DELIVER_GOAL',   Listen_deliver_goal(),  transitions = {'failed':'LISTEN_DELIVER_GOAL',  'succ': 'DELIVER',              'tries':'END'})
        smach.StateMachine.add('DELIVER',               Deliver(),              transitions = {'failed':'DELIVER',              'succ': 'END',                  'tries':'END'})

    outcome = sm.execute()

