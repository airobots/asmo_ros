#!/usr/bin/env python

'''
  
  ASMO Process Template
  
  Rony Novianto (www.ronynovianto.com)
  University of Technology, Sydney, Australia
  
'''

import roslib
import rospy

roslib.load_manifest('asmo')

import asmo.msg

# Modify here #

process_name = 'process_name'
attention_value = 0.0

def run(publisher):
    global attention_value
    topic = '/asmo/plan1/approach_person/turtle1/cmd_vel'
    attention_value = 50.0
    msg = asmo.msg.TopicAttention(topic, attention_value)
    publisher.publish(msg)
    rospy.sleep(1.0)
    
# End modification #

def init():
    rospy.init_node(process_name)
    publisher = rospy.Publisher('/asmo/topic_attention', asmo.msg.TopicAttention, queue_size=10)
    print '[ OK ] Start', process_name
    while not rospy.is_shutdown():
        run(publisher)
        
if __name__ == '__main__':
    init()
