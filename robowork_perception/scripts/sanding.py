#!/usr/bin/env python
import rospy
import os
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Pose, PoseArray
import argparse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from control_msgs.msg import FollowJointTrajectoryActionResult
from moveit_msgs.msg import MoveGroupActionFeedback  
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
import csv
import matplotlib.pyplot as plt
import numpy as np
body_goal_reached = True
arm_goal_reached = 0
arm_planned = 0

def get_quat_from_norm(n1,n2,n3):
    normals = np.array([n1,n2,n3])
    normals_norm = np.linalg.norm(normals)
    if normals_norm != 0:
        normals = normals/normals_norm  
    else: 
        normals
    reference = np.array([1., 0., 0.]).astype(np.float32)
    angle_normals = np.arccos(np.dot(reference, normals))


    axis_normals = np.cross(reference, normals)
    axis_normals_norm =  np.linalg.norm(axis_normals)
    if axis_normals_norm != 0:
        axis_normals = axis_normals / np.linalg.norm(axis_normals) 
    else: 
        axis_normals



    w = np.cos(angle_normals / 2)
    x, y, z = axis_normals* np.sin(angle_normals / 2)

    q = np.array([0,0,0,1]).astype(np.float32)
    q[0] = x
    q[1] = y
    q[2] = z
    q[3] = w
    q = q/np.linalg.norm(q)
    return q

def find_path():
    u_z_values = []
    u_y_values = []
    u_x_values = []
    r_y_values = []
    r_x_values = []
    nx = []
    ny = []
    nz = []
    with open(os.path.expanduser('~/autonomous_mobile_manipulation_ws/src/autonomous_mobile_manipulation/robowork_perception/scripts/output_0-8n.csv'), mode='r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            u_x_values.append(float(row['offset_x']))
            u_y_values.append(float(row['offset_y']))
            u_z_values.append(float(row['z']))
            r_y_values.append(float(row['y']))
            r_x_values.append(float(row['x']))
            nx.append(float(row['nx']))
            ny.append(float(row['ny']))
            nz.append(float(row['nz']))

    sorted_xyz = sorted(zip(u_x_values,u_y_values,u_z_values,nx,ny,nz))
    x_values,y_values,z_values,nx_values,ny_values,nz_values = zip(*sorted_xyz)
    x_stops = []
    y_stops = []
    nx_stops = []
    ny_stops = []
    nz_stops = []
    flag=1
    max_x = max(r_x_values)
    max_y = max(r_y_values)
    min_x = min(r_x_values)
    min_y = min(r_y_values)
    for i in range (0, len(x_values)):
        if (y_values[i] < 0):
            if x_stops == []:
                x_stops.append(round(x_values[i],2))
                y_stops.append(round(y_values[i],2))
                nx_stops.append(nx_values[i])
                ny_stops.append(ny_values[i])
                nz_stops.append(nz_values[i])
                x_current = x_values[i]
                y_current = y_values[i]
            if (x_values[i] - x_current)**2 + (y_values[i] - y_current)**2 > 0.49 :	
                flag = True
                for j in range (0, len(r_x_values)):
                    if (y_values[i] - r_y_values[j]) ** 2 + (x_values[i] - r_x_values[j]) ** 2 < 0.6:
                        flag = False
                
                if flag:
                    x_stops.append(round(x_values[i],2))
                    y_stops.append(round(y_values[i],2))
                    nx_stops.append(nx_values[i])
                    ny_stops.append(ny_values[i])
                    nz_stops.append(nz_values[i])
                    x_current=x_values[i]
                    y_current=y_values[i]
    x_current=x_values[i-1]
    y_current=y_values[i-1]

    for i in range (len(x_values)-1, -1 , -1):
        if (y_values[i] > 0):
            if (x_values[i] - x_current)**2 + (y_values[i] - y_current)**2 > 0.49:
                flag = True
                for j in range (0, len(r_x_values)):
                    if (y_values[i] - r_y_values[j]) ** 2 + (x_values[i] - r_x_values[j]) ** 2 < 0.6:
                        flag = False
            #if (x_values[i-1] != x_current and x):
                if flag:
                    x_stops.append(round(x_values[i],2))
                    y_stops.append(round(y_values[i],2))
                    nx_stops.append(nx_values[i])
                    ny_stops.append(ny_values[i])
                    nz_stops.append(nz_values[i])
                    x_current = x_values[i]
                    y_current = y_values[i]
                    
    return (x_stops,y_stops,max_x,min_x,max_y,min_y,nx_stops,ny_stops,nz_stops)

def find_path_second(x_points, y_points, z_points):
    z_values = []
    y_values = []
    x_values = []
    r_y_values = []
    r_x_values = []
    nx = []
    ny = []
    nz = []
    with open(os.path.expanduser('~/autonomous_mobile_manipulation_ws/src/autonomous_mobile_manipulation/robowork_perception/scripts/output_0-7n.csv'), mode='r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            x_values.append(float(row['offset_x']))
            y_values.append(float(row['offset_y']))
            z_values.append(float(row['z']))
            r_y_values.append(float(row['y']))
            r_x_values.append(float(row['x']))
            nx.append(float(row['nx']))
            ny.append(float(row['ny']))
            nz.append(float(row['nz']))

    sorted_xyz = sorted(zip(x_values,y_values,z_values,nx,ny,nz))
    x_values,y_values,z_values,nx_values,ny_values,nz_values = zip(*sorted_xyz)
    x_stops = []
    y_stops = []
    nx_stops = []
    ny_stops = []
    nz_stops = []
    flag=1
    max_x = max(r_x_values)
    max_y = max(r_y_values)
    min_x = min(r_x_values)
    min_y = min(r_y_values)
    for k in range (0, len(x_points)):
        max_dist = 0.0
        min_dist = 1000.0
        for i in range (0, len(x_values)):
            if (x_values[i] - x_points[k])**2 + (y_values[i] - y_points[k])**2 < min_dist :
                flag = True
                for j in range (0, len(r_x_values)):
                    if (y_values[i] - r_y_values[j]) ** 2 + (x_values[i] - r_x_values[j]) ** 2 < 0.45:
                        flag = False
                    #if (x_values[i-1] != x_current and x):
                if flag:
                    min_dist = (x_values[i] - x_points[k])**2 + (y_values[i] - y_points[k])**2
                    x_append = (x_values[i])
                    y_append = (y_values[i])
                    nx_append = (nx_values[i])
                    ny_append = (ny_values[i])
                    nz_append = (nz_values[i])
        x_stops.append(round(x_append,2))
        y_stops.append(round(y_append,2))
        nx_stops.append(nx_append)
        ny_stops.append(ny_append)
        nz_stops.append(nz_append)

                    
    return (x_stops,y_stops,max_x,min_x,max_y,min_y,nx_stops,ny_stops,nz_stops)

def arm_path(x_stops,y_stops,max_x,min_x,nx_stops,ny_stops,nz_stops):
    z_values = []
    y_values = []
    x_values = []
    r_y_values = []
    r_x_values = []
    r_z_values = []
    nx = []
    ny = []
    nz = []
    with open(os.path.expanduser('~/autonomous_mobile_manipulation_ws/src/autonomous_mobile_manipulation/robowork_perception/scripts/output_0-10.csv'), mode='r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            x_values.append(float(row['offset_x']))
            y_values.append(float(row['offset_y']))
            z_values.append(float(row['offset_z']))
            r_z_values.append(float(row['z']))
            r_y_values.append(float(row['y']))
            r_x_values.append(float(row['x']))
            nx.append(float(row['nx']))
            ny.append(float(row['ny']))
            nz.append(float(row['nz']))
    
    x_arms = []
    y_arms = []
    z_arms = []
    nx_arms = []
    ny_arms = []
    nz_arms = []
    min_max = []
    for i in range (0, len(x_stops)):
        x_arms.append([])
        y_arms.append([]) 
        z_arms.append([]) 
        nx_arms.append([]) 
        ny_arms.append([]) 
        nz_arms.append([]) 
        min_max.append([])
    s_current = 0
    s_next = 1
    breaker = 0
    max_dis=0
    min_dis=1000
    for i in range (0, len(x_values)):
        distance=[]
        for j in range(0, len(x_stops)):
            distance. append(((x_values[i]-x_stops[j])**2) + ((y_values[i]-y_stops[j])**2) + z_values[i] **2)
        min_index = distance.index(min(distance))
        
        
        x_arms[min_index].append(round(x_values[i],2))
        y_arms[min_index].append(round(y_values[i],2))
        z_arms[min_index].append(round(z_values[i],2))
        nx_arms[min_index].append(nx[i]) 
        ny_arms[min_index].append(ny[i]) 
        nz_arms[min_index].append(nz[i]) 
        
    for i in range (0, len(x_stops)):
        for j in range(0, len(x_arms[i])-1):
            min_point=100000.0
            for k in range(j+1, len(x_arms[i])):
                if (x_arms[i][j] - x_arms[i][k])**2 + (y_arms[i][j] - y_arms[i][k])**2 + (z_arms[i][j] - z_arms[i][k])**2 < min_point:
                    min_point = (x_arms[i][j] - x_arms[i][k])**2 + (y_arms[i][j] - y_arms[i][k])**2 + (z_arms[i][j] - z_arms[i][k])**2 
                    x_tra = x_arms[i][k]
                    y_tra = y_arms[i][k]
                    z_tra = z_arms[i][k]
                    nx_tra = nx_arms[i][k]
                    ny_tra = ny_arms[i][k]
                    nz_tra = nz_arms[i][k]
                    
                    x_arms[i][k]= x_arms[i][j+1]
                    y_arms[i][k] = y_arms[i][j+1]
                    z_arms[i][k] = z_arms[i][j+1]
                    nx_arms[i][k] = nx_arms[i][j+1]
                    ny_arms[i][k] = ny_arms[i][j+1]
                    nz_arms[i][k] = nz_arms[i][j+1]
                    
                    x_arms[i][j+1]= x_tra
                    y_arms[i][j+1] = y_tra
                    z_arms[i][j+1] = z_tra
                    nx_arms[i][j+1] = nx_tra
                    ny_arms[i][j+1] = ny_tra
                    nz_arms[i][j+1] = nz_tra 
                    
                  
                    
    for i in range (0, len(x_stops)):  
        normals = np.array([-nx_stops[i], -ny_stops[i], 0])
        normals_norm = np.linalg.norm(normals)
        if normals_norm != 0:
            normals = (normals/normals_norm) * 0.6  
        else: 
            normals	* 0.6
        x_arms[i].append(round(x_stops[i] + normals[0], 2))
        y_arms[i].append(round(y_stops[i] + normals[1], 2)) 
        z_arms[i].append(0.8) 
        nx_arms[i].append(nx_stops[i]) 
        ny_arms[i].append(ny_stops[i]) 
        nz_arms[i].append(0)

    return(x_arms,y_arms,z_arms,nx_arms,ny_arms,nz_arms)

def body_result_callback(data):
    global body_goal_reached       
    status_code = data.status.status
    if status_code == 3:  
        rospy.loginfo("Goal reached.")
        body_goal_reached = True
    else:
        rospy.loginfo("Goal not reached, status code: %d", status_code)
        body_goal_reached = False

def arm_plan_callback(data):
    global arm_planned       
    status_code = data.status.status
    if status_code == 3:  
        rospy.loginfo("Arm Planned.")
        arm_planned = 3
    elif status_code == 4:
        rospy.loginfo("Planning not possible, status code: %d", status_code)
        arm_planned = 4

def arm_exe_callback(data):
    global arm_goal_reached       
    status_code = data.status.status
    if status_code == 3: 
        rospy.loginfo("Arm_Goal reached.")
        arm_goal_reached = 3
    elif status_code == 4:
        rospy.loginfo("Arm_Goal not reached, status code: %d", status_code)
        arm_goal_reached = 4

def next_message():
    next_pub.publish(joy_message)
    rospy.loginfo("Published Joy message to /rviz_visual_tools_gui")
    rospy.sleep(1) 

def publish_goal():
    #Initilizing the node, subscribers and publishers
    global body_goal_reached 
    global arm_goal_reached
    global arm_planned
    rospy.init_node('Publish_nav_goal')
    rate=rospy.Rate(10)
    rviz_pub = rospy.Publisher('/rviz_2d_nav_goal', PoseStamped, queue_size=1)
    rospy.sleep(1)  
    endeffector_pub = rospy.Publisher('/endeffector_goal_pose', PoseStamped, queue_size=1)
    rospy.sleep(1)
    rospy.Subscriber("/bvr_SIM/move_base/result", MoveBaseActionResult, body_result_callback)
    rospy.sleep(1)
    rospy.Subscriber("/bvr_SIM/move_group/feedback", MoveGroupActionFeedback, arm_plan_callback)
    rospy.sleep(1)
    rospy.Subscriber("/bvr_SIM/execute_trajectory/feedback", MoveGroupActionFeedback, arm_exe_callback)
    rospy.sleep(1)
    next_pub = rospy.Publisher('/rviz_visual_tools_gui', Joy, queue_size=1)
    rospy.sleep(1)
    visual_all_pub = rospy.Publisher('/pose_all', PoseArray, queue_size=10)
    rospy.sleep(1)
    visual_group_pub = rospy.Publisher('/pose_group', PoseArray, queue_size=10)
    rospy.sleep(1)
    # Finding the path for base and arm
    x, y, max_x, min_x, max_y, min_y,nx_stops,ny_stops,nz_stops = find_path()
    x_arms,y_arms,z_arms,nx_arms,ny_arms,nz_arms = arm_path(x, y, max_x,min_x,nx_stops,ny_stops,nz_stops)
    rospy.loginfo("Calculated the path")
    pub_flag = 0
    # Create a Joy message
    joy_message = Joy()
    joy_message.header.seq = 0
    joy_message.header.stamp = rospy.Time.now()
    joy_message.header.frame_id = 'map'
    joy_message.buttons = [0, 1, 0, 0, 0, 0]
    second_round_point_x = []
    second_round_point_y = []
    second_round_point_z = []
    second_round_point_nx = []
    second_round_point_ny = []
    second_round_point_nz = []

    for i in range (0, len(x)): # I changed this to (0,3) when I recorded the video
        pose_arr_all = PoseArray()
        pose_arr_all.header.frame_id = 'map'
        pose_arr_all.header.stamp = rospy.Time.now()

        pose_arr_group = PoseArray()
        pose_arr_group.header.frame_id = 'map'
        pose_arr_group.header.stamp = rospy.Time.now()

        for k in range (0, len(x)):
            for l in range (0, len(x_arms[k])-1):
                pose = Pose()
                pose.position.x = x_arms[k][l]
                pose.position.y = y_arms[k][l]
                pose.position.z = z_arms[k][l]
                q = get_quat_from_norm(-nx_arms[k][l],-ny_arms[k][l],-nz_arms[k][l])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                if k == i:
                    pose_arr_group.poses.append(pose)
                else:
                    pose_arr_all.poses.append(pose)
        visual_all_pub.publish(pose_arr_all)
        visual_group_pub.publish(pose_arr_group)

        rospy.loginfo("Body_goal_reached_flag ({})".format(body_goal_reached))
        while not rospy.is_shutdown():            
            if pub_flag == 0:    
                pub_flag = 1
                body_goal_reached = False
                body_pose = PoseStamped()
                body_pose.header = Header(seq=0, stamp=rospy.Time.now(), frame_id='map')
                body_pose.pose.position.x = x[i]
                body_pose.pose.position.y = y[i]
                q_body = get_quat_from_norm(-nx_stops[i], -ny_stops[i], 0)
                body_pose.pose.orientation = Quaternion (q_body[0], q_body[1], q_body[2], q_body[3])
                rviz_pub.publish(body_pose)
                rospy.loginfo("Published to RViz at position ({}, {}, {})".format(x[i], y[i],q_body))
                rospy.sleep(1)  

            if body_goal_reached:
                rospy.loginfo("Body_goal_reached_flag ({})".format(body_goal_reached))
                pub_flag = 0
                break
            rate.sleep()
        for j in range (0, len(x_arms[i])):
            rospy.loginfo("Point ({})".format(j))
            rospy.sleep(1)
            while not rospy.is_shutdown():
                arm_pose = PoseStamped()
                arm_pose.header = Header(seq=0, stamp=rospy.Time.now(), frame_id='map')
                arm_pose.pose.position.x = x_arms[i][j]
                arm_pose.pose.position.y = y_arms[i][j]
                arm_pose.pose.position.z = z_arms[i][j]
                q_arm = get_quat_from_norm(-nx_arms[i][j], -ny_arms[i][j], -nz_arms[i][j])
                arm_pose.pose.orientation = Quaternion(q_arm[0],q_arm[1],q_arm[2],q_arm[3])
                endeffector_pub.publish(arm_pose)
                rospy.loginfo("Arm_Goal Published at position ({}, {}, {}, {}, {}, {}, {})".format(x_arms[i][j], y_arms[i][j], z_arms[i][j], q_arm[0], q_arm[1], q_arm[2], q_arm[3] ))
                rospy.sleep(2)
                next_pub.publish(joy_message)
                rospy.loginfo("Planning") 
                rospy.sleep(3)
                while not rospy.is_shutdown():    
                    if arm_planned == 3:
                        rospy.loginfo("Planning Possible")  
                        arm_planned = 0   
                        break
                    elif arm_planned == 4:
                        rospy.loginfo("Planning not possible")     
                        second_round_point_x.append(x_arms[i][j])
                        second_round_point_y.append(y_arms[i][j])
                        second_round_point_z.append(z_arms[i][j])
                        second_round_point_nx.append(nx_arms[i][j])
                        second_round_point_ny.append(ny_arms[i][j])
                        second_round_point_nz.append(nz_arms[i][j])
                        arm_planned = 0
                        break
                    rate.sleep()
                rospy.loginfo("Executing the plan ({})".format(arm_goal_reached))
                while not rospy.is_shutdown():
                    if arm_goal_reached == 3:
                        arm_goal_reached = 0
                        break
                    elif arm_goal_reached == 4:
                        next_pub.publish(joy_message)
                        rospy.loginfo("planning again")
                        rospy.sleep(1)
                        arm_goal_reached = 0 
                    rate.sleep()
                break
                
################################  Second Round ################################

    x, y, max_x, min_x, max_y, min_y,nx_stops,ny_stops,nz_stops = find_path_second(second_round_point_x, second_round_point_y, second_round_point_z)
    rospy.loginfo("Second round, number of points: {}".format(len(second_round_point_x)))
    pub_flag = 0
    n_remaining = 0

    for i in range (0, len(x)):
        pose_arr_all = PoseArray()
        pose_arr_all.header.frame_id = 'map'
        pose_arr_all.header.stamp = rospy.Time.now()

        pose_arr_group = PoseArray()
        pose_arr_group.header.frame_id = 'map'
        pose_arr_group.header.stamp = rospy.Time.now()

        for k in range (0, len(x)):
                pose = Pose()
                pose.position.x = second_round_point_x[k]
                pose.position.y = second_round_point_y[k]
                pose.position.z = second_round_point_z[k]
                q = get_quat_from_norm(-second_round_point_nx[k], -second_round_point_ny[k], -second_round_point_nz[k])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                if k == i:
                    pose_arr_group.poses.append(pose)
                else:
                    pose_arr_all.poses.append(pose)
        visual_all_pub.publish(pose_arr_all)
        visual_group_pub.publish(pose_arr_group)
        
        rospy.loginfo("Body_goal_reached_flag ({})".format(body_goal_reached))
                    
        while not rospy.is_shutdown():            
            if pub_flag == 0:    
                pub_flag = 1
                body_goal_reached = False
                body_pose = PoseStamped()
                body_pose.header = Header(seq=0, stamp=rospy.Time.now(), frame_id='map')
                body_pose.pose.position.x = x[i]
                body_pose.pose.position.y = y[i]
                q_body = get_quat_from_norm(-nx_stops[i], -ny_stops[i], 0)
                body_pose.pose.orientation = Quaternion (q_body[0], q_body[1], q_body[2], q_body[3])
                rviz_pub.publish(body_pose)
                rospy.loginfo("Published to RViz at position ({}, {}, {})".format(x[i], y[i],q_body))
                rospy.sleep(1) 

            if body_goal_reached:
                rospy.loginfo("Body_goal_reached_flag ({})".format(body_goal_reached))
                pub_flag = 0
                break
            rate.sleep()
        for j in range (0, 1):
            rospy.loginfo("Point ({})".format(j))
            rospy.sleep(1)
            while not rospy.is_shutdown():
                arm_pose = PoseStamped()
                arm_pose.header = Header(seq=0, stamp=rospy.Time.now(), frame_id='map')
                arm_pose.pose.position.x = second_round_point_x[i]
                arm_pose.pose.position.y = second_round_point_y[i]
                arm_pose.pose.position.z = second_round_point_z[i]
                q_arm = get_quat_from_norm(-second_round_point_nx[i], -second_round_point_ny[i], -second_round_point_nz[i])
                arm_pose.pose.orientation = Quaternion(q_arm[0],q_arm[1],q_arm[2],q_arm[3])
                endeffector_pub.publish(arm_pose)
                rospy.loginfo("Arm_Goal Published at position ({}, {}, {}, {}, {}, {}, {})".format(second_round_point_x[i], second_round_point_y[i], second_round_point_z[i], q_arm[0], q_arm[1], q_arm[2], q_arm[3] ))
                rospy.sleep(2)
                next_pub.publish(joy_message)
                rospy.loginfo("planning") 
                rospy.sleep(3)
                while not rospy.is_shutdown():    
                    if arm_planned == 3:
                        rospy.loginfo("Planning Possible")  
                        arm_planned = 0   
                        break
                    elif arm_planned == 4:
                        rospy.loginfo("Planning not possible")     
                        n_remaining = n_remaining + 1
                        arm_planned = 0
                        break
                    rate.sleep()
                rospy.loginfo("Executing the plan ({})".format(arm_goal_reached))
                while not rospy.is_shutdown():
                    if arm_goal_reached == 3:
                        arm_goal_reached = False
                        break
                    elif arm_goal_reached == 4:
                        next_pub.publish(joy_message)
                        rospy.loginfo("planning again")
                        rospy.sleep(1)
                        arm_goal_reached = 0 
                    rate.sleep()
          
                normals = np.array([-nx_stops[i], -ny_stops[i], 0])
                normals_norm = np.linalg.norm(normals)
                if normals_norm != 0:
                    normals = (normals/normals_norm) * 0.6  
                else: 
                    normals	* 0.6
                break
            while not rospy.is_shutdown():
                arm_pose = PoseStamped()
                arm_pose.header = Header(seq=0, stamp=rospy.Time.now(), frame_id='map')
                arm_pose.pose.position.x = x[i] + normals[0]
                arm_pose.pose.position.y = y[i] + normals[1]
                arm_pose.pose.position.z = 0.8
                q_arm = get_quat_from_norm(-nx_stops[i], -ny_stops[i], 0)
                arm_pose.pose.orientation = Quaternion(q_arm[0],q_arm[1],q_arm[2],q_arm[3])
                endeffector_pub.publish(arm_pose)
                rospy.loginfo("Arm_Goal Published at position ({}, {}, {}, {}, {}, {}, {})".format(x[i] + normals[0], y[i] + normals[1], 0.45, q_arm[0], q_arm[1], q_arm[2], q_arm[3] ))
                rospy.sleep(2)
                next_pub.publish(joy_message)
                rospy.loginfo("planning") 
                rospy.sleep(3)
                while not rospy.is_shutdown():    
                    if arm_planned == 3:
                        rospy.loginfo("Planning Possible")  
                        arm_planned = 0   
                        break
                    elif arm_planned == 4:
                        rospy.loginfo("Planning not possible")     
                        n_remaining = n_remaining + 1
                        arm_planned = 0
                        break
                    rate.sleep()
                rospy.loginfo("Executing the plan ({})".format(arm_goal_reached))
                while not rospy.is_shutdown():
                    if arm_goal_reached == 3:
                        arm_goal_reached = False
                        break
                    elif arm_goal_reached == 4:
                        next_pub.publish(joy_message)
                        rospy.loginfo("planning again")
                        rospy.sleep(1)
                        arm_goal_reached = 0 
                    rate.sleep()
                break
                
    rospy.loginfo("Remaining number of points ({})".format(n_remaining))

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
