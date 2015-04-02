#!/usr/bin/env python

'''
    2nd Example of ASMO Process
    ---------------------------
    Author:
        Rony Novianto (rony@ronynovianto.com)
        University of Technology Sydney, Australia
'''

import rospy
import geometry_msgs.msg
import yaml
import asmo.msg

_process_name = 'approach_person_by_shortest_distance'
velocity = 0.0

def handle_person_location(point32):
    global velocity
    # Turn the robot to face the person
    if point32.x > 0:
        velocity = -0.25
    else:
        velocity = 0.25
        
def run(publishers):
    twist = geometry_msgs.msg.Twist()
    twist.angular.z = velocity
    #publishers['cmd_vel'].publish(twist)
    
    message_actions = []
    message_actions.append(asmo.msg.MessageAction(
        topic_name = '/turtle1/cmd_vel',
        message = '"{}"'.format(str(twist))
    ))
    publishers['message_non_reflex'].publish(
        name = _process_name,
        attention_value = 50.0,
        message_actions = message_actions
    )
    
    
def main():
    rospy.init_node(_process_name)
    publishers = {}
    #publishers['cmd_vel'] = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    publishers['message_non_reflex'] = rospy.Publisher('/asmo/message_non_reflex', asmo.msg.MessageNonReflex, queue_size=10)
    rospy.Subscriber('/person/position', geometry_msgs.msg.Point32, handle_person_location)
    print('[ OK ] Start {process_name}'.format(process_name=_process_name))
    while not rospy.is_shutdown():
        run(publishers)
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
