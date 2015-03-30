#!/usr/bin/env python

'''
    ASMO Gateway
    ------------
    Rony Novianto (rony@ronynovianto.com)
    University of Technology, Sydney, Australia
'''

# General import files
import os

# Tornado: import files
import json
import tornado.httpclient

# ROS: import files
import rospy
import rostopic
import std_msgs.msg
import asmo.msg

# Tornado: global variables
if rospy.has_param('/asmo/host_uri'):
    host_uri = rospy.get_param('/asmo/host_uri')
else:
    host_uri = 'http://localhost:12766'
json_headers = {'content-type': 'application/json'}
http_client = tornado.httpclient.HTTPClient()

# ROS: global variables
publishers = {}
process_dict = {}
message_dict = {}
received_topics = [] # Temporary
if rospy.has_param('/asmo/process_signature'):
    asmo_signature = rospy.get_param('/asmo/process_signature')
else:
    asmo_signature = '/asmo/process/'
if rospy.has_param('/asmo/allow_actions_execution'):
    allow_actions_execution = rospy.get_param('/asmo/allow_actions_execution')
else:
    allow_actions_execution = True
    
# Tornado: Functions

def post_non_reflex(name, actions, required_resources, attention_value = 0.0, boost_value = 0.0):
    url = host_uri + '/non_reflex/' + name
    json_object = {
        "attention_value": attention_value,
        "boost_value": boost_value,
        "actions": actions,
        "required_resources": required_resources
    }
    #print(name, json_object)
    json_data = json.dumps(json_object)
    request = tornado.httpclient.HTTPRequest(url, method='POST', body=json_data, headers=json_headers)
    response = http_client.fetch(request)
    
def post_reflex(name, actions, required_resources, priority_level = 0.0):
    url = host_uri + '/reflex/' + name
    json_object = {
        "priority_level": priority_level,
        "actions": actions,
        "required_resources": required_resources
    }
    json_data = json.dumps(json_object)
    request = tornado.httpclient.HTTPRequest(url, method='POST', body=json_data, headers=json_headers)
    response = http_client.fetch(request)
    
def post_all_processes():
    for process in process_dict.values():
        if 'priority_level' in process:
            post_reflex(process['name'], process['actions'], process['required_resources'], process['priority_level'])
        else:
            process.setdefault('attention_value', 0.0)
            process.setdefault('boost_value', 0.0)
            post_non_reflex(process['name'], process['actions'], process['required_resources'], process['attention_value'], process['boost_value'])
            
def post_compete():
    url = host_uri + '/compete'
    request = tornado.httpclient.HTTPRequest(url, method='POST', body='')
    response = http_client.fetch(request)
    
    if response.error:
        print('[ Error ]', response.error)
    else:
        body = json.loads(response.body)
        winners = asmo.msg.Winners(*body)
        publishers['winners'].publish(winners)
        if allow_actions_execution:
            for action in body['actions']: os.system(action + ' &')
            
# ROS: Functions

def decode_topic(encoded_topic, signature):
    start = len(signature)
    end = encoded_topic.find('/', start)
    process_name = encoded_topic[start:end]
    target_topic = encoded_topic[end:]
    if len(signature) > 0 and encoded_topic.find(signature) == 0 and end > start and len(target_topic) > 0:
        result = (process_name, target_topic)
    else:
        result = (False, False)
    return result
    
def handle_receive_message(r):
    global message_dict, received_topics
    # Update required resources
    source_topic = r._connection_header['topic']
    message_dict[source_topic] = r
    
    if source_topic not in received_topics:
        received_topics.append(source_topic)
        
def handle_setup_process(r):
    global publishers, process_dict
    for encoded_topic in r.topic_list:
        (process_name, target_topic) = decode_topic(encoded_topic, asmo_signature)
        if not target_topic:
            print('[ Error ] process signature %s is not detected in %s' % (asmo_signature, encoded_topic))
            print('[ Solution ] set ros parameter /asmo/process_signature to %s' % asmo_signature)
            continue
        encoded_type = rostopic.get_topic_class(encoded_topic)[0]
        if encoded_type == None:
            print('[ Error ] %s has not been initialized' % encoded_topic)
            print('[ Solution ] initialize publisher for %s before calling setup_process()' % encoded_topic)
            continue
        rospy.Subscriber(encoded_topic, encoded_type, handle_receive_message)
        if target_topic not in publishers:
            publishers[target_topic] = rospy.Publisher(target_topic, encoded_type, queue_size=10)
        if process_name not in process_dict:
            process_dict[process_name] = {
                'name': process_name,
                'actions': [],
                'encoded_topics': [],
                'required_resources': []
            }
        if target_topic not in process_dict[process_name]['required_resources']:
            process_dict[process_name]['required_resources'].append(target_topic)
            
def handle_attention_boost(r):
    global process_dict
    process_dict[r.name]['attention_value'] = r.attention_value
    process_dict[r.name]['boost_value'] = r.boost_value
    
def handle_priority_level(r):
    global process_dict
    process_dict[r.name]['priority_level'] = r.priority_level
    
def handle_non_reflex(r):
    #post_non_reflex(*r)
    post_non_reflex(r.name, r.actions, r.required_resources, r.attention_value, r.boost_value)
    
def handle_reflex(r):
    post_reflex(r.name, r.actions, r.required_resources, r.priority_level)
    
def handle_remove_process(r):
    global process_dict
    if r.data in process_dict: del process_dict[r.data]
    
def handle_compete(r):
    post_all_processes()
    #post_compete()
    
def handle_redirect_message(r):
    for name in r.names:
        for encoded_topic in process_dict[name].encoded_topics:
            target_topic = message_dict[encoded_topic]['target_topic']
            publishers[target_topic].publish(message_dict[encoded_topic])
            
def init():
    global publishers
    
    if rospy.has_param('/asmo/allow_periodic_competition'):
        allow_periodic_competition = rospy.get_param('/asmo/allow_periodic_competition')
    else:
        allow_periodic_competition = True
    # periodic_rate is in Hz
    if rospy.has_param('/asmo/periodic_rate'):
        periodic_rate = rospy.get_param('/asmo/periodic_rate')
    else:
        periodic_rate = 10
        
    rospy.init_node('asmo_gateway')
    publishers['winners'] = rospy.Publisher('/asmo/winners', asmo.msg.Winners, queue_size=10)
    rospy.Subscriber('/asmo/setup_process', asmo.msg.SetupProcess, handle_setup_process)
    rospy.Subscriber('/asmo/attention_boost', asmo.msg.AttentionBoost, handle_attention_boost)
    rospy.Subscriber('/asmo/priority_level', asmo.msg.PriorityLevel, handle_priority_level)
    rospy.Subscriber('/asmo/non_reflex', asmo.msg.NonReflex, handle_non_reflex)
    rospy.Subscriber('/asmo/reflex', asmo.msg.Reflex, handle_reflex)
    rospy.Subscriber('/asmo/remove_process', std_msgs.msg.String, handle_remove_process)
    rospy.Subscriber('/asmo/compete', std_msgs.msg.Empty, handle_compete)
    print('[ OK ] Start asmo_gateway')
    
    if allow_periodic_competition:
        while not rospy.is_shutdown():
            handle_compete([])
            rospy.Rate(periodic_rate).sleep()
    else:
        rospy.spin()
        
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
