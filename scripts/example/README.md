```
# Terminal A
# Run ASMO web
roscore
rosrun turtlesim turtlesim_node
rostopic echo /turtle1/cmd_vel
# Terminal B
# Reverse any changes to not use ASMO
rosrun asmo plan1.py
rostopic pub --rate=20 /person/position geometry_msgs/Point32 'x: 2.0'
# Terminal C
# Reverse any changes to not use ASMO
rosrun asmo plan2.py
rostopic pub --rate=20 /fastlane/position geometry_msgs/Point32 'x: -2.0'

# Terminate plan1.py and plan2.py
# Modify plan1.py and plan2 to use ASMO

# Terminal B
rosrun asmo plan1.py
# Terminal C
rosrun asmo plan2.py
# Terminal A
# Rerun turtlesim
rosrun asmo gateway.py


# Terminate plan1.py and plan2.py
# Modify the position of the person to 'x: 4.0'
# Run plan1.py and plan2.py
rostopic pub --rate=1 /person/position geometry_msgs/Point32 'x: 4.0'
```
