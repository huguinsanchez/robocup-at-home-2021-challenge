#!/usr/bin/env python

from utils_takeshi import *

########## Functions for takeshi states ##########
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
        
        if area > 200 and area < 50000 :                        
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
    add_object("table_big", [1.7, 0.13, 0.7], [0.95, 1.9, 0.34], [0.5, 0.5])
    add_object("table_small", [0.5, 0.01, 0.4], [0.1, 1.9, 0.61], [0.5, 0.5])
    add_object("table_tray", [0.65, 0.01, 0.7], [1.8, -0.65, 0.4], [0.5, 0.5])    
    return True


def segment_table():
    image_data = rgbd.get_image()
    points_data = rgbd.get_points()

    mask = np.zeros((image_data.shape))
    plane_mask = np.zeros((image_data.shape[0], image_data.shape[1]))

    plane_mask = image_data[:, :, 1]

    ret, thresh = cv2.threshold(image_data[:, :, 2], 240, 255, 200)
    plane_mask = points_data['z']
    cv2_img = plane_mask.astype('uint8')
    img = image_data[:, :, 0]
    _, contours, hierarchy = cv2.findContours(thresh.astype('uint8'), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    i = 0
    cents = []
    for i, contour in enumerate(contours):
        area = cv2.contourArea(contour)

        if area > 2000 and area < 50000 :
            boundRect = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (boundRect[0], boundRect[1]), (boundRect[0] + boundRect[2], boundRect[1] + boundRect[3]), (0, 0, 0), 2)
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
            cv2.putText(img, "centroid_" + str(i) + "_" + str(cX) + ',' + str(cY), (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
    cents = np.asarray(cents)
    return cents
    


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


##### Define state SCAN_FLOOR #####
#Va al mess1 piso y voltea hacia abajo la cabeza y escanea el piso
class Scan_floor(Takeshi_states):
    def takeshi_run(self):     
        global cents, rot, trans
        goal_x , goal_y, goal_yaw = kl_mess1        
        head_val = head.get_current_joint_values()
        head_val[0] = np.deg2rad(0)
        head_val[1] = np.deg2rad(-45)        
        head.go(head_val)
        succ = move_base_goal(goal_x, goal_y, goal_yaw)        
        trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
        #print(trans, euler)
        cents = segment_floor()
        static_tf_publish(cents)
        return succ

##### Define state PRE_FLOOR #####
#Baja el brazo al suelo, abre la garra y se acerca al objeto para grasp
class Pre_floor(Takeshi_states):
    def takeshi_run(self):
        global closest_cent
        move_hand(1)
        publish_scene()
        arm.go(arm_grasp_floor)        
        trans_cents = []
        
        for i, cent in enumerate(cents):
            trans_map, _ = listener.lookupTransform('/map', 'static' + str(i), rospy.Time(0))
            trans_cents.append(trans_map)
        
        np.linalg.norm(np.asarray(trans_cents) - trans , axis = 1)
        closest_cent = np.argmin(np.linalg.norm(np.asarray(trans_cents) - trans , axis = 1))
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static' + str(closest_cent), rospy.Time(0))
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] -0.1
        wb[1] += trans_hand[1]
        succ = whole_body.go(wb)
        return succ

##### Define state GRASP_FLOOR #####
#Se acerca mas al objeto y cierra la garra
class Grasp_floor(Takeshi_states):
    def takeshi_run(self):
        global trans_hand
        print(closest_cent)
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static' + str(closest_cent), rospy.Time(0))
        trans_hand, rot_hand
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] - 0.05
        wb[1] += trans_hand[1]
        succ = whole_body.go(wb)
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static0', rospy.Time(0))
        move_hand(0)
        return succ


##### Define state POST_FLOOR #####
#Se hace para atras, verifica grasp y pone posicion neutral
class Post_floor(Takeshi_states):
    def takeshi_run(self):
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
#Se mueve hacia la caja baja el brazo y se acerca mas 
class Go_box(Takeshi_states):
    def takeshi_run(self):
        goal_x, goal_y, goal_yaw =  kl_tray #Known location tray 1
        succ = move_base_goal(goal_x, goal_y + 0.3, -90)
        publish_scene()
        return succ
        

##### Define state DELIVER #####
#Suelta el objeto
class Deliver(Takeshi_states):
    def takeshi_run(self):
        arm.set_joint_value_target(arm_ready_to_place)
        arm.go()
        wb = whole_body.get_current_joint_values()
        wb[0] += -0.45
        wb[4] += -.3
        whole_body.set_joint_value_target(wb)
        whole_body.go()
        move_hand(1)
        wb = whole_body.get_current_joint_values()
        wb[0] += 0.3
        whole_body.set_joint_value_target(wb)
        succ = whole_body.go()
        return succ






#TABLE


##### Define state SCAN_TABLE #####
class Scan_table(Takeshi_states):
    def takeshi_run(self):
        global cents, rot, trans
        goal_x , goal_y, goal_yaw = kl_mess1
        head_val = head.get_current_joint_values()
        head_val[0] = np.deg2rad(0)
        head_val[1] = np.deg2rad(-45)        
        head.go(head_val)
        succ = move_base_goal(goal_x, goal_y + 0.99, goal_yaw)      
        trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)        
        cents = segment_table()                                    
        static_tf_publish(cents)
        print("Los cents " +str(cents))
        return succ

##### Define state PRE_TABLE #####
class Pre_table(Takeshi_states):
    def takeshi_run(self):
        global closest_cent 
        global cents              
        goal_x , goal_y, goal_yaw = kl_mess1 
        print("Los cents " +str(cents))
        publish_scene()

        trans_cents = []        
        for i, cent in enumerate(cents):
            trans_map, _ = listener.lookupTransform('/map', 'static' + str(i), rospy.Time(0))
            trans_cents.append(trans_map)
        
        print("los trans " + str(trans_cents))
        
        np.linalg.norm(np.asarray(trans_cents) - trans , axis = 1)
        closest_cent = np.argmin(np.linalg.norm(np.asarray(trans_cents) - trans , axis = 1))
        print("Mas cerca " + str(closest_cent))
        
        
                     
        succ = move_base_goal(goal_x+0.5 ,goal_y+.99,goal_yaw)      ##FIJO??

        head_val = head.get_current_joint_values()
        head_val[0] = np.deg2rad(0)
        head_val[1] = np.deg2rad(-35)
        head.go(head_val)
        cents = segment_table()
        static_tf_publish(cents)
        publish_scene()
        print("Nuevos cents " + str(cents))
        
        move_hand(1)
        arm.set_joint_value_target(arm_grasp_table)
        arm.go()

        return succ

     
#Define state GRASP_TABLE
class Grasp_table(Takeshi_states):
    def takeshi_run(self):
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static0', rospy.Time(0))
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] - 0.07
        wb[1] += trans_hand[1]
        wb[3] += trans_hand[0] + 0.1
        whole_body.go(wb)
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static0', rospy.Time(0))        
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] - 0.07
        wb[1] += trans_hand[1]
        wb[3] += trans_hand[0]
        whole_body.go(wb)
        trans_hand, rot_hand = listener.lookupTransform('/hand_palm_link', 'static0', rospy.Time(0))
        scene.remove_world_object()
        wb = whole_body.get_current_joint_values()
        wb[0] += trans_hand[2] - 0.07
        wb[1] += trans_hand[1]
        wb[3] += trans_hand[0]
        succ = whole_body.go(wb)
        move_hand(0)
        return succ


##### Define state POST_TABLE #####
class Post_table(Takeshi_states):
    def takeshi_run(self):
        a = gripper.get_current_joint_values()
        if np.linalg.norm(a - np.asarray(grasped)) > (np.linalg.norm(a - np.asarray(ungrasped))):
            print ('grasp seems to have failed')
            succ = False
        else:
            print('assuming succesful grasp')
            wb = whole_body.get_current_joint_values()
            wb[0] += -0.2
            wb[3] += 0.2
            whole_body.set_joint_value_target(wb)
            succ = whole_body.go()
        
        publish_scene()
        #Takeshi neutral
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        head.go()
        return succ


        


#Initialize global variables and node
def init(node_name):
    global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd    
    rospy.init_node(node_name)
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGBD()

#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0

    with sm:
        #State machine for grasping on Floor
        """smach.StateMachine.add("INITIAL",       Initial(),      transitions = {'failed':'INITIAL',      'succ':'SCAN_FLOOR',    'tries':'END'}) 
        smach.StateMachine.add("SCAN_FLOOR",    Scan_floor(),   transitions = {'failed':'SCAN_FLOOR',   'succ':'PRE_FLOOR',     'tries':'END'}) 
        smach.StateMachine.add('PRE_FLOOR',     Pre_floor(),    transitions = {'failed':'PRE_FLOOR',    'succ': 'GRASP_FLOOR',  'tries':'END'}) 
        smach.StateMachine.add('GRASP_FLOOR',   Grasp_floor(),  transitions = {'failed':'GRASP_FLOOR',  'succ': 'POST_FLOOR',   'tries':'END'}) 
        smach.StateMachine.add('POST_FLOOR',    Post_floor(),   transitions = {'failed':'INITIAL',      'succ': 'GO_BOX',       'tries':'END'}) 
        smach.StateMachine.add('GO_BOX',        Go_box(),       transitions = {'failed':'GO_BOX',       'succ': 'DELIVER',      'tries':'END'})
        smach.StateMachine.add('DELIVER',       Deliver(),      transitions = {'failed':'DELIVER',      'succ': 'INITIAL',      'tries':'END'})"""

        #State machine for grasping on Table
        smach.StateMachine.add("INITIAL",       Initial(),      transitions = {'failed':'INITIAL',      'succ':'SCAN_TABLE',    'tries':'END'}) 
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),   transitions = {'failed':'SCAN_TABLE',   'succ':'PRE_TABLE',     'tries':'END'}) 
        smach.StateMachine.add('PRE_TABLE',     Pre_table(),    transitions = {'failed':'PRE_TABLE',    'succ': 'GRASP_TABLE',  'tries':'END'}) 
        smach.StateMachine.add('GRASP_TABLE',   Grasp_table(),  transitions = {'failed':'GRASP_TABLE',  'succ': 'POST_TABLE',   'tries':'END'}) 
        smach.StateMachine.add('POST_TABLE',    Post_table(),   transitions = {'failed':'POST_TABLE',   'succ': 'GO_BOX',       'tries':'END'}) 
        smach.StateMachine.add('GO_BOX',        Go_box(),       transitions = {'failed':'GO_BOX',       'succ': 'DELIVER',      'tries':'END'})
        smach.StateMachine.add('DELIVER',       Deliver(),      transitions = {'failed':'DELIVER',      'succ': 'INITIAL',      'tries':'END'})

        #Must join state machines for Floor and Table

    outcome = sm.execute()


    