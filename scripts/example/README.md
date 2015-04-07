```
# terminal A
roscore
rosrun turtlesim turtlesim_node
# terminal B
rosrun asmo plan1.py
rostopic pub --rate=1 /person/position geometry_msgs/Point32 'x: 5.0'
# terminal C
# reverse the modification of plan2.py
rosrun asmo plan2.py
rostopic pub --rate=1 /emotion/reaction geometry_msgs/Point32 '{x: -10.0, y: 0.4, z: 0.0}'

# terminal B
rosrun asmo plan1.py /turtle1/cmd_vel:=/asmo/plan1/approach_person/turtle1/cmd_vel
rosrun asmo aplan1.py
# terminal C
# modify plan2.py
rosrun asmo plan2.py
# terminal A
roslaunch asmo asmo.launch

rostopic pub --rate=1 /person/position std_msgs/Float32 'data: 10.0'
rostopic pub --rate=1 /emotion/reaction geometry_msgs/Point32 '{x: -10.0, y: 0.4, z: 0.0}'

# Test
rostopic pub --rate=1 /emotion/reaction geometry_msgs/Point32 '{x: -10.0, y: 0.6, z: 0.0}'
```
