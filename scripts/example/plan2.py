#!/usr/bin/env python

'''
    Node 2 Example
    --------------
    Author:
        Rony Novianto (rony@ronynovianto.com)
        University of Technology Sydney
'''

import rospy
import std_msgs.msg
import geometry_msgs.msg
import asmo.msg

publishers = {}

def setup_process():
    # Sleep to let the publisher finish initializing
    rospy.sleep(0.25)
    msg = asmo.msg.SetupProcess(['/asmo/process/approach_person/turtle1/cmd_vel'])
    publishers['setup_process'].publish(msg)
    
def handle_emotion_reaction(r):
    global publishers
    twist = geometry_msgs.msg.Twist()
    # Turn the robot to face the person
    if r.x > 0:
        twist.angular.z = 0.25
    else:
        twist.angular.z = -0.25
    publishers['cmd_vel'].publish(twist)
    publishers['probability'].publish(r.y)
    msg = asmo.msg.AttentionBoost('approach_person', r.y*100, 0)
    publishers['attention_boost'].publish(msg)
    
def init():
    rospy.init_node('plan2')
    rospy.Subscriber('/emotion/reaction', geometry_msgs.msg.Point32, handle_emotion_reaction)
    publishers['probability'] = rospy.Publisher('/plan2/probability', std_msgs.msg.Float32, queue_size=10)
    #publishers['/turtle1/cmd_vel'] = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    publishers['cmd_vel'] = rospy.Publisher('/asmo/process/approach_person/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    publishers['setup_process'] = rospy.Publisher('/asmo/setup_process', asmo.msg.SetupProcess, queue_size=10)
    publishers['attention_boost'] = rospy.Publisher('/asmo/attention_boost', asmo.msg.AttentionBoost, queue_size=10)
    setup_process()
    print '[ OK ] Start plan2'
    rospy.spin()
    
if __name__ == '__main__':
    init()
