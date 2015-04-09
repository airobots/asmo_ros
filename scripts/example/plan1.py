#!/usr/bin/env python

'''
    1st example of ASMO Process
    ---------------------------
    Author:
        Rony Novianto (rony@ronynovianto.com)
        University of Technology Sydney, Australia
'''

import rospy
import geometry_msgs.msg
import asmo.msg

_process_name = 'approach_person_by_shortest_distance'
attention_value = 50.0
publishers = {}

def handle_person_position(point32):
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = 1.0
    # Shortest path: turn left if the person is on the left and vice versa
    if point32.x > 0:
        twist.angular.z = 1.0
    else:
        twist.angular.z = -1.0
        
    #publishers['cmd_vel'].publish(twist)
    
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
    global publishers
    rospy.init_node(_process_name)
    #publishers['cmd_vel'] = rospy.Publisher('/turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    publishers['message_non_reflex'] = rospy.Publisher('/asmo/message_non_reflex', asmo.msg.MessageNonReflex, queue_size=10)
    rospy.Subscriber('/person/position', geometry_msgs.msg.Point32, handle_person_position)
    print('[ OK ] Start {process_name}'.format(process_name=_process_name))
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
