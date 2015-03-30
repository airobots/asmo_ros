#!/usr/bin/env python

'''
    Node 1 Example
    --------------
    Author:
        Rony Novianto (rony@ronynovianto.com)
        University of Technology Sydney
'''

import rospy
import std_msgs.msg
import geometry_msgs.msg

publishers = {}

def handle_person_position(r):
    global publishers
    twist = geometry_msgs.msg.Twist()
    # Turn the robot to face the person
    if r.data > 0:
        twist.angular.z = 0.25
    else:
        twist.angular.z = -0.25
    publishers['/turtle1/cmd_vel'].publish(twist)
    
def init():
    rospy.init_node('plan1')
    rospy.Subscriber('/person/position', std_msgs.msg.Float32, handle_person_position)
    publishers['/turtle1/cmd_vel'] = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    print '[ OK ] Start plan1'
    rospy.spin()
    
if __name__ == '__main__':
    init()
