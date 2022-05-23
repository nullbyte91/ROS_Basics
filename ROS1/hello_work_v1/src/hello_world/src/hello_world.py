#! /usr/bin/env python3 
# interpreter used
import rospy # Python client lib for ROS

rospy.init_node("hello_world")     # Initiate a node called hello_world
rate = rospy.Rate(1)               # We create a Rate object of 1Hz
display = rospy.get_param("/text")
count = rospy.get_param("/count")
while not rospy.is_shutdown():     # Continous loop
   print("Display: {} count:{}".format(display, count))
   count += 1
   rate.sleep()                    # We sleep the needed time to maintain the above Rate