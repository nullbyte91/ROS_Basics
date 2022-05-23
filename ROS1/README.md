# ROS 1 Basics
## Create a First package
1. create a package
2. create a node [Python]
3. Create a launch file
4. Compile
5. Run 

### Create a Package 
```bash
mkdir ~/catkin_ws/src/  # Create a directory
cd ~/catkin_ws/src/     # Move to src directory
catkin_create_pkg hello_world rospy # Create a package
# rospy is python clinet lib for ROS
```

![alt text](./images/dir_structure_1.png "directory")

### Create a node
```bash
cd hello_world/ && touch src/hello_world.py # Create a empty file
chmod a+x src/hello_world.py # Execution permission
```

```bash
hello_world.py
```

```python
#! /usr/bin/env python3 
# interpreter used
import rospy # Python client lib for ROS

rospy.init_node("hello_world")     # Initiate a node called hello_world
rate = rospy.Rate(1)               # We create a Rate object of 1Hz
while not rospy.is_shutdown():     # Continous loop
   print("My first ROS package Hello world ")
   rate.sleep()                    # We sleep the needed time to maintain the above Rate
```

### Create a Launch file
```bash
mkdir launch && touch launch/hello_world.launch # Create a empty lauch file
```
```bash
hello_world.launch
```

```xml
<launch>
    <!-- My Package launch file -->
    <node pkg="hello_world" type="hello_world.py" name="hello_world"  output="screen">
    </node>
</launch>
```

### Compile
```bash
cd ~/catkin_ws/ # go to root directory of the packages
catkin_make # Compile a ROS package
source devel/setup.bash # Source the ROS env variable
```

### RUN
```bash
roslaunch hello_world hello_world.launch
```

#### output
![alt text](./images/hw_output_1.png "directory")

## Basic ROS Concepts:
### Param Server:
A parameter server is a shared, multi-variate dictionary that is accessible via network APIs. Nodes use this server to store and retrieve parameters at runtime. 

### Create a Param file
```bash
cd ~/catkin_ws/src/hello_world/ && mkdir config/ && touch config/param.yaml
```
```bash
config/param.yaml
```

```xml
text: "Hello_world"
count: 0 
```

### Load and Retrieving ROS param 

```bash
hello_world.launch #Updated version
```

```xml
<launch>
    <!-- Load config file -->
    <rosparam file="$(find hello_world)/config/param.yaml" />
    <!-- hello world launch file -->
    <node pkg="hello_world" type="hello_world.py" name="hello_world"  output="screen">
    </node>
</launch>

```bash
hello_world.py # Updated version 
```

```python
#! /usr/bin/env python3 
# interpreter used
import rospy # Python client lib for ROS

rospy.init_node("hello_world")     # Initiate a node called hello_world
rate = rospy.Rate(1)               # We create a Rate object of 1Hz
display = rospy.get_param("/text")
count = rospy.get_param("/count")
while not rospy.is_shutdown():     # Continous loop
   global count
   print("Display: {} count:{} \n".format(display, count))
   count += 1
   rate.sleep()                    # We sleep the needed time to maintain the above Rate
```

### RUN
```bash
roslaunch hello_world hello_world.launch
```

#### output
![alt text](./images/hw_output_log2.png "directory")
## Debugging Tools

```bash
rosnode list #list all the runnig nodes
rosnode info <node_name> #Info about particular node
```

