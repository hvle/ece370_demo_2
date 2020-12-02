#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from gazebo_msgs.srv import ApplyJointEffort
from gzebo_msgs.srv import GetModelState
import time

## move_robot_package
# Global Variables
disable = 0
t0 = time.time()
t1 = time.time()



class Server:
    def __init__(self,*args, **kwargs):
        # self.disable = disable
        self.MAX_RANGE = 1
        # self.key = None
        # self.ranges = None

    def keyboard_callback(self, msg):
        global disable,t0
        # print("keyboard_callback")
        key = msg.data
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        twist = Twist()  
        t0 = time.time()


        # print(key)
        # print("System Disable Status: ", self.disable)
   
        move_bindings = {
            'w':(0.5,0,0,0),
            'a':(0,0,0,0.5),
            's':(-0.5,0,0,0),
            'd':(0,0,0,-0.5),
            'q':(0.5,0,0,0.5),
            'e':(0.5,0,0,-0.5),    
        }

        disabled_move_bindings = {
            'w':(0,0,0,0),
            'a':(0,0,0,0.5),
            's':(-0.5,0,0,0),
            'd':(0,0,0,-0.5),
            'q':(0,0,0,0.5),
            'e':(0,0,0,-0.5),    
        }
        try:
            if disable:
                x = disabled_move_bindings[key][0]
                y = disabled_move_bindings[key][1]
                z = disabled_move_bindings[key][2]
                th = disabled_move_bindings[key][3]
            else:
                x = move_bindings[key][0]
                y = move_bindings[key][1]
                z = move_bindings[key][2]
                th = move_bindings[key][3]
        except:
            x = 0
            y = 0
            z = 0
            th = 0

        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = 0
        twist.angular.x = 0
        twist.angular.z = th

        pub.publish(twist)
        

    def laserscan_callback(self, msg):
        global disable, t0, t1
        scan = msg.ranges[315:405]
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        twist = Twist()   
        t1 = time.time()
        if (t1-t0 >  0.05):
            pub.publish(twist)
            # print("timeout")
        # print(scan)
        for n in scan:
            if (n <= self.MAX_RANGE):
                # print("pre-if disable", disable)
                if not(disable):

                    twist.linear.x = 0
                    pub.publish(twist)
                    # print("stopping")
                    disable = 1
                    # print("changing disable to 1")
                return None

        disable = 0
                # print("changing disable to 0")
        # print("disable end of callback", disable)
       
        






if __name__ == '__main__':
    rospy.init_node('hvle_move_robot')

    server = Server()

    rospy.Subscriber('/key_press', String, server.keyboard_callback)
    # rospy.Subscriber('/kobuki/laser/scan', LaserScan, server.laserscan_callback)
    
    rospy.spin()
