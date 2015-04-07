#!/usr/bin/env python

'''
    ASMO
    ----
    Author:
        Rony Novianto (rony@ronynovianto.com)
        University of Technology Sydney, Australia
'''

# General import files
import os
# Web: import files
import yaml
import json
import requests_futures.sessions
# ROS: import files
import rospy
import rostopic
import roslib
import std_msgs.msg
import asmo.msg

# Constants
_process_name = 'ASMO'
__version__ = '0.4'
# Web: global variables
if rospy.has_param('/asmo/host_uri'):
    host_uri = rospy.get_param('/asmo/host_uri')
else:
    host_uri = 'http://localhost:12766'
json_headers = {'content-type': 'application/json'}
request_session = requests_futures.sessions.FuturesSession()
# ROS: global variables
publishers = {}
message_dict = {}
if rospy.has_param('/asmo/allow_message_actions_execution'):
    allow_message_actions_execution = rospy.get_param('/asmo/allow_message_actions_execution')
else:
    allow_message_actions_execution = True
if rospy.has_param('/asmo/allow_custom_actions_execution'):
    allow_custom_actions_execution = rospy.get_param('/asmo/allow_custom_actions_execution')
else:
    allow_custom_actions_execution = False
    
# General functions

def post_process(json_object):
    url = '{host_uri}/process/{name}'.format(host_uri=host_uri, name=json_object['name'])
    json_data = json.dumps(json_object)
    request_session.post(url, data=json_data, headers=json_headers)
    
def get_publisher(topic_name, topic_type):
    global publishers
    
    if topic_type == '':
        msg_class = rostopic.get_topic_class(topic_name)[0]
        if not msg_class:
            raise rostopic.ROSTopicException('[ Error ] no subscriber or publisher for {}'.format(topic_name))
    else:
        msg_class = roslib.message.get_message_class(topic_type)
        
    if topic_name not in publishers:
        publishers[topic_name] = rospy.Publisher(topic_name, msg_class, queue_size=10)
        
    return (publishers[topic_name], msg_class)
    
def publish_message(pub, msg_class, pub_args):
    msg = msg_class()
    now = rospy.get_rostime()
    keys = {'now': now, 'auto': std_msgs.msg.Header(stamp=now)}
    roslib.message.fill_message_args(msg, pub_args, keys=keys)
    pub.publish(msg)
    
def redirect_message(winner_names):
    for name in winner_names:
        if name not in message_dict: continue
        for ma in message_dict[name].message_actions:
            try:
                (pub, msg_class) = get_publisher(ma.topic_name, ma.topic_type)
                pub_args = [yaml.load(ma.message)]
                publish_message(pub, msg_class, pub_args)
            except:
                raise
                
# Handle subscription functions

def handle_process(msg):
    for key in msg.__slots__:
        process[key] = msg.__getattribute__(key)
    post_process(process)
    
def handle_message_process(msg):
    global message_dict
    message_dict[msg.name] = msg
    process = {}
    for key in msg.__slots__:
        process[key] = msg.__getattribute__(key)
    del process['message_actions']
    process['required_resources'] = []
    process['actions'] = []
    for ma in msg.message_actions:
        if ma.topic_name not in process['required_resources']:
            process['required_resources'].append(ma.topic_name)
    post_process(process)
    
def handle_name_value(msg):
    process = {'name': msg.name}
    topic = msg._connection_header['topic']
    # Strip the character if the topic ends with '/'
    last_index = len(topic) - 1
    if topic[last_index] == '/': topic = topic[:last_index]
    slash_index = topic.rfind('/')
    key = topic[slash_index+1:]
    process[key] = msg.value
    post_process(process)
    
def handle_remove_process(msg):
    url = '{host_uri}/process/{name}'.format(host_uri=host_uri, name=msg.data)
    request_session.delete(url)
    
def handle_compete(msg):
    global message_dict
    url = '{host_uri}/compete'.format(host_uri=host_uri)
    future = request_session.post(url)
    # Non-blocking: future.result() is a callback
    json_data = future.result().json()
    if 'error' not in json_data:
        json_data['names'] = json_data['winners'].keys()
        del json_data['winners']
        winners = asmo.msg.Winners(**json_data)
        publishers['/asmo/winners'].publish(winners)
        if allow_message_actions_execution:
            redirect_message(json_data['names'])
            message_dict = {}
        if allow_custom_actions_execution:
            for action in json_data['actions']: os.system(action + ' &')
            
# Main function

def main():
    global publishers
    
    if rospy.has_param('/asmo/allow_periodic_competition'):
        allow_periodic_competition = rospy.get_param('/asmo/allow_periodic_competition')
    else:
        allow_periodic_competition = True
    if rospy.has_param('/asmo/periodic_milliseconds'):
        periodic_milliseconds = rospy.get_param('/asmo/periodic_milliseconds')
    else:
        periodic_milliseconds = 110
        
    # periodic_rate is in Hz
    periodic_rate = 1000 / periodic_milliseconds
    rospy.init_node(_process_name)
    publishers['/asmo/winners'] = rospy.Publisher('/asmo/winners', asmo.msg.Winners, queue_size=10)
    rospy.Subscriber('/asmo/non_reflex', asmo.msg.NonReflex, handle_process)
    rospy.Subscriber('/asmo/reflex', asmo.msg.Reflex, handle_process)
    rospy.Subscriber('/asmo/message_non_reflex', asmo.msg.MessageNonReflex, handle_message_process)
    rospy.Subscriber('/asmo/message_reflex', asmo.msg.MessageReflex, handle_message_process)
    rospy.Subscriber('/asmo/attention_value', asmo.msg.NameValue, handle_name_value)
    rospy.Subscriber('/asmo/boost_value', asmo.msg.NameValue, handle_name_value)
    rospy.Subscriber('/asmo/priority_level', asmo.msg.NameValue, handle_name_value)
    rospy.Subscriber('/asmo/remove_process', std_msgs.msg.String, handle_remove_process)
    rospy.Subscriber('/asmo/compete', std_msgs.msg.Empty, handle_compete)
    print('[ OK ] Start {process_name}'.format(process_name=_process_name))
    
    if allow_periodic_competition:
        while not rospy.is_shutdown():
            handle_compete([])
            rospy.Rate(periodic_rate).sleep()
    else:
        rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
