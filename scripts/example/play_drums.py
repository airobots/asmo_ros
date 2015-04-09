#!/usr/bin/env python

'''
    Play drums example
    ---------------------------
    Author:
        Rony Novianto (rony@ronynovianto.com)
        University of Technology Sydney, Australia
'''

import math
import random
import rospy
import geometry_msgs.msg
import asmo.msg

_process_name = 'play_drums'
attention_value = 0.0
subjective_weight = 0.0
time = 0

def run(publishers):
    global attention_value, time
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = 1.0
    twist.angular.z = -1.0
    #publishers['cmd_vel'].publish(twist)
    
    time -= 1
    if time < 0:
        time = random.randint(0, 120)
        attention_value = 0.5 * (100 - 100 / (1 + math.exp(-0.06 * (time - 60)))) + 0.5 * subjective_weight
    
    message_actions = []
    message_actions.append(asmo.msg.MessageAction(
        topic_name = '/turtle1/cmd_vel',
        message = str(twist)
    ))
    publishers['message_non_reflex'].publish(
        name = _process_name,
        attention_value = attention_value,
        message_actions = message_actions
    )
    
def main():
    publishers = {}
    rospy.init_node(_process_name)
    #publishers['cmd_vel'] = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    publishers['message_non_reflex'] = rospy.Publisher('/asmo/message_non_reflex', asmo.msg.MessageNonReflex, queue_size=10)
    print('[ OK ] Start {process_name}'.format(process_name=_process_name))
    while not rospy.is_shutdown():
        run(publishers)
        rospy.sleep(0.1)
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
