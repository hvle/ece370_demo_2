#!/usr/bin/env python

import rospy

from std_msgs.msg import String

import sys, select, termios, tty


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

###############################################################################

settings = termios.tcgetattr(sys.stdin)
rospy.init_node('teleop_twist_keyboard')
pub = rospy.Publisher('/key_press', String, queue_size=1)

key_timeout = rospy.get_param("~key_timeout", 0.0)
if key_timeout == 0.0:
    key_timeout = None
try:
    while(1):
        key = getKey(key_timeout)
        print(key)
        pub.publish(key)
        if (key == '\x03'):
            break
        

except Exception as e:
    print(e)

finally:
    # pub_thread.stop()
    print("exiting")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)