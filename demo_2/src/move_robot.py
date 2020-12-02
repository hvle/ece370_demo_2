#! /usr/bin/env python

import rospy
import os
from std_msgs.msg import String
from gazebo_msgs.srv import ApplyJointEffort
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetJointProperties
import time

## move_robot_package
# Global Variables
t0 = time.time()
t1 = time.time()


class Server:
    def __init__(self,*args, **kwargs):
        pass
    
    def keyboard_callback(self,msg):
        global t0
        key = msg.data
        # print("keyboard_callback")
        # print(key)
        
        joint_left = "dd_robot::left_wheel_hinge"
        joint_right = "dd_robot::right_wheel_hinge"

        pub = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        speed_multiplier = 1.0
        # (Left Effort, Right Effort)
        move_bindings = {
            "w": (1.0,1.0),
            "a": (-1.0,1.0),
            "s": (-1.0,-1.0),
            "d": (1.0,-1.0),
            "q": (0.5,1.0),
            "e": (1.0,0.5),
        }
        start_time = rospy.Time(0,0)
        end_time = rospy.Time(1.0,0)

        try:
            left_effort = speed_multiplier*move_bindings[key][0]
            right_effort = speed_multiplier*move_bindings[key][1]
        except:
            left_effort = 0.0
            right_effort = 0.0
            # msg_topic_feedback = "/gazebo/get_joint_properties"
            # pub_feedback = rospy.ServiceProxy(msg_topic_feedback, GetJointProperties)

            # left_feedback = pub_feedback(joint_left)
            # right_feedback = pub_feedback(joint_right)
            # print("Left Wheel Rate:", left_feedback.rate)
            # print("Right Wheel Rate:", right_feedback.rate)
            # left_effort = left_feedback.rate[0] * -1
            # right_effort = right_feedback.rate[0] * -1
                    
        # print("Left effort:", left_effort)
        # print("Right effort:", right_effort)
        msg_topic_feedback = "/gazebo/get_joint_properties"
        pub_feedback = rospy.ServiceProxy(msg_topic_feedback, GetJointProperties)
        left_feedback = pub_feedback(joint_left)
        right_feedback = pub_feedback(joint_right)
        print("Left Wheel Rate:", left_feedback.rate)
        print("Right Wheel Rate:", right_feedback.rate)
        pub(joint_left,left_effort,start_time,end_time)
        pub(joint_right,right_effort,start_time,end_time)

if __name__ == '__main__': 
    print("Starting...")
    rospy.init_node('hvle_move_robot')

    server = Server()
    rospy.Subscriber('/key_press', String, server.keyboard_callback)
    msg_topic_feedback = "/gazebo/get_model_state"

    top_coords = [[-22.5, 25.5,0], [-20.5, 25.5,0], [-18.5, 25.5,0], [-16.5, 25.5,0], [-14.5, 25.5,0], [-12.5, 25.5,0], [-10.5, 25.5,0], [-8.5, 25.5,0], [-6.5, 25.5,0], [-4.5, 25.5,0], [-2.5, 25.5,0], [-0.5, 25.5,0], [1.5, 25.5,0], [3.5, 25.5,0], [5.5, 25.5,0], [7.5, 25.5,0], [9.5, 25.5,0], [11.5, 25.5,0], [13.5, 25.5,0], [15.5, 25.5,0], [17.5, 25.5,0], [19.5, 25.5,0], [21.5, 25.5,0], [23.5, 25.5,0]]
    right_coords = [[25.5, 25.5,0], [25.5, 23.5,0], [25.5, 21.5,0], [25.5, 19.5,0], [25.5, 17.5,0], [25.5, 15.5,0], [25.5, 13.5,0], [25.5, 11.5,0], [25.5, 9.5,0], [25.5, 7.5,0], [25.5, 5.5,0], [25.5, 3.5,0], [25.5, 1.5,0], [25.5, -0.5,0], [25.5, -2.5,0], [25.5, -4.5,0], [25.5, -6.5,0], [25.5, -8.5,0], [25.5, -10.5,0], [25.5, -12.5,0], [25.5, -14.5,0], [25.5, -16.5,0], [25.5, -18.5,0], [25.5, -20.5,0], [25.5, -22.5,0], [25.5, -24.5,0]]
    bottom_coords = [[24.5, -25.5,0], [22.5, -25.5,0], [20.5, -25.5,0], [18.5, -25.5,0], [16.5, -25.5,0], [14.5, -25.5,0], [12.5, -25.5,0], [10.5, -25.5,0], [8.5, -25.5,0], [6.5, -25.5,0], [4.5, -25.5,0], [2.5, -25.5,0], [0.5, -25.5,0], [-1.5, -25.5,0], [-3.5, -25.5,0], [-5.5, -25.5,0], [-7.5, -25.5,0], [-9.5, -25.5,0], [-11.5, -25.5,0], [-13.5, -25.5,0], [-15.5, -25.5,0], [-17.5, -25.5,0], [-19.5, -25.5,0], [-21.5, -25.5,0], [-23.5, -25.5,0], [-25.5, -25.5,0]]
    left_coords = [[-25.5, -23.5,0], [-25.5, -21.5,0], [-25.5, -19.5,0], [-25.5, -17.5,0], [-25.5, -15.5,0], [-25.5, -13.5,0], [-25.5, -11.5,0], [-25.5, -9.5,0], [-25.5, -7.5,0], [-25.5, -5.5,0], [-25.5, -3.5,0], [-25.5, -1.5,0], [-25.5, 0.5,0], [-25.5, 2.5,0], [-25.5, 4.5,0], [-25.5, 6.5,0], [-25.5, 8.5,0], [-25.5, 10.5,0], [-25.5, 12.5,0], [-25.5, 14.5,0], [-25.5, 16.5,0], [-25.5, 18.5,0], [-25.5, 20.5,0], [-25.5, 22.5,0], [-25.5, 24.5,0],[-24.5, 25.5,0]]

    radius = 25
    buff = 2
    buff_zone = radius - buff

    id = 1
    while True:
        pub_feedback = rospy.ServiceProxy(msg_topic_feedback, GetModelState)
        val = pub_feedback("dd_robot","")
        x_location = val.pose.position.x
        y_location = val.pose.position.y
        # print(x_location,y_location)


        if (abs(x_location) >= buff_zone):
            if (x_location > 0):
                # print("Out of bounds right")
                diff = 100
                idx = 0
                for i in range(len(right_coords)):
                    if abs(right_coords[i][1] - y_location) <= diff:
                        diff = abs(right_coords[i][1] - y_location)
                        idx = i 
                
                if right_coords[idx][2] == 0:
                    right_coords[idx][2] = 1
                    cmd = "~/catkin_ws/src/demo_2/models/spawnBox.sh " +str(right_coords[idx][0])+" "+str(right_coords[idx][1])+" "+str(id)
                    os.system(cmd)
                    id += 1
                else:
                    pass
                    # print("Box already placed")
            else:
                # print("Out of bounds left")
                diff = 100
                idx = 0
                for i in range(len(left_coords)):
                    if abs(left_coords[i][1] - y_location) <= diff:
                        diff = abs(left_coords[i][1] - y_location)
                        idx = i 

                if left_coords[idx][2] == 0:
                    left_coords[idx][2] = 1
                    cmd = "~/catkin_ws/src/demo_2/models/spawnBox.sh " +str(left_coords[idx][0])+" "+str(left_coords[idx][1])+" "+str(id)
                    os.system(cmd)
                    id += 1
                else:
                    pass
                    # print("Box already placed")
                
        elif (abs(y_location) >= buff_zone):
            if (y_location > 0):
                # print("Out of bounds top")
                diff = 100
                idx = 0
                for i in range(len(top_coords)):
                    if abs(top_coords[i][0] - x_location) <= diff:
                        diff = abs(top_coords[i][0] - x_location)
                        idx = i 
                if top_coords[idx][2] == 0:
                    top_coords[idx][2] = 1
                    cmd = "~/catkin_ws/src/demo_2/models/spawnBox.sh " +str(top_coords[idx][0])+" "+str(top_coords[idx][1])+" "+str(id)
                    os.system(cmd)
                    id += 1
                else:
                    pass
                    # print("Box already placed")
            else:
                # print("Out of bounds bottom")
                diff = 100
                idx = 0
                for i in range(len(bottom_coords)):
                    if abs(bottom_coords[i][0] - x_location) <= diff:
                        diff = abs(bottom_coords[i][0] - x_location)
                        idx = i 
                if bottom_coords[idx][2] == 0:
                    bottom_coords[idx][2] = 1
                    cmd = "~/catkin_ws/src/demo_2/models/spawnBox.sh " +str(bottom_coords[idx][0])+" "+str(bottom_coords[idx][1])+" "+str(id)
                    os.system(cmd)
                    id += 1
                else:
                    pass
                    # print("Box already placed")
            # print("Robot out of bounds")
        else:
            pass
            # print("Robot in bounds")





        time.sleep(1)
    rospy.spin()
    

    