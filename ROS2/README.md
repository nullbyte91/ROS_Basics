# ROS 2 Basics
## Create a First package
1. create a package
2. create a node [Python]
3. Create a launch file
4. Compile
5. Run 

### Create a Package 
```bash
source /opt/ros/foxy/setup.bash # Source the ROS2 Foxy 
mkdir -p ~/ros2_ws/src/  # Create a directory
cd ~/ros2_ws/src/     # Move to src directory
ros2 pkg create --build-type ament_python hello_world --dependencies rclpy # Create a package
# rospy is python clinet lib for ROS
```

```bash
Console output
```
```python
going to create a new package
package name: hello_world
destination directory: /home/nullbyte/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['nullbyte <nullbyte.in@gmail.com>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: ['rclpy']
creating folder ./hello_world
creating ./hello_world/package.xml
creating source folder
creating folder ./hello_world/hello_world
creating ./hello_world/setup.py
creating ./hello_world/setup.cfg
creating folder ./hello_world/resource
creating ./hello_world/resource/hello_world
creating ./hello_world/hello_world/__init__.py
creating folder ./hello_world/test
creating ./hello_world/test/test_copyright.py
creating ./hello_world/test/test_flake8.py
creating ./hello_world/test/test_pep257.py

```

### Create a node
```bash
cd hello_world/ && touch hello_world/helloworld.py # Create a empty file
chmod a+x hello_world/helloworld.py # Execution permission
```

```bash
helloworld.py
```

```python
import rclpy
from rclpy.node import Node

class Hello_World(Node):
    def __init__(self):
        super().__init__('hello_world') 
        self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info("My first ROS package Hello world")
        print("debug")

def main(args=None):
    rclpy.init()  # initialize the ROS communication
    node = Hello_World() # create object
    rclpy.spin(node) # Spin the node
    rclpy.shutdown() # shutdown the ROS communication

if __name__ == '__main__':
    main()
```

### Create a Launch file
```bash
mkdir launch && touch launch/hello_world_launch.py # Create a empty lauch file
chmod a+x launch/hello_world_launch.py
```
```bash
hello_world_launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_world',
            executable='helloworld',
            output='screen'),
    ])
```

### Update setup.py file
```python
from setuptools import setup
import os 
import glob
package_name = 'hello_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/hello_world_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nullbyte',
    maintainer_email='nullbyte.in@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld = hello_world.helloworld:main'
        ],
    },
)

```
### Compile
```bash
cd ~/ros2_ws/ # go to root directory of the packages
colcon build # Compile a ROS package
source install/setup.bash # Source the ROS env variable
```

### RUN
```bash
ros2 launch hello_world hello_world_launch.py
``````bash
hello_world.py # Updated version 
```

#### output
```bash
ros2 launch hello_world hello_world_launch.py
[INFO] [launch]: All log files can be found below /home/nullbyte/.ros/log/2022-05-31-00-02-35-637827-edgeai-13827
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [helloworld-1]: process started with pid [13829]
[helloworld-1] [INFO] [1653935556.007625856] [hello_world]: My first ROS package Hello world
[helloworld-1] [INFO] [1653935556.075156617] [hello_world]: My first ROS package Hello world
[helloworld-1] [INFO] [1653935556.174747624] [hello_world]: My first ROS package Hello world
```

## Basic ROS Concepts:
### Param Server:
A parameter server is a shared, multi-variate dictionary that is accessible via network APIs. Nodes use this server to store and retrieve parameters at runtime. 

### Create a Param file
```bash
cd ~/ros2_ws/src/hello_world/ && mkdir config/ && touch config/param.yaml
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

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
    get_package_share_directory('hello_world'),
    'config',
    'parameters.yaml'
    )
    node1 = Node(
            package='hello_world',
            executable='helloworld',
            output='screen',
            parameters = [config]
    )
    ld.add_action(node1)
    return ld
```

### Update setup.py file
```python
from setuptools import setup
import os 
import glob
package_name = 'hello_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/hello_world_launch.py']),
        ('share/' + package_name, ['config/param.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nullbyte',
    maintainer_email='nullbyte.in@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld = hello_world.helloworld:main'
        ],
    },
)
```

```bash
hello_world.py # Updated version 
```

```python
import rclpy
from rclpy.node import Node

class Hello_World(Node):
    def __init__(self):
        super().__init__('hello_world') 
        self.create_timer(0.1, self.timer_callback)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('text', "hello world")
            ])


    def timer_callback(self):
        text = self.get_parameter('text')
        self.get_logger().info('My first ROS package:"%s"' % text.value)


def main(args=None):
    rclpy.init()  # initialize the ROS communication
    node = Hello_World() # create object
    rclpy.spin(node) # Spin the node
    rclpy.shutdown() # shutdown the ROS communication

if __name__ == '__main__':
    main()
```

## ROS Topics
Topics are called buses over which nodes exchange messages. Topic communication is unidirectional and streaming.

## Create a Publisher with ROS perception package
### Create a Package 
```bash
source /opt/ros/foxy/setup.bash # Source the ROS2 Foxy 
mkdir -p ~/ros2_ws/src/  # Create a directory
cd ~/ros2_ws/src/     # Move to src directory
ros2 pkg create --build-type ament_python image_pipeline --dependencies rclpy image_transport cv_bridge sensor_msgs std_msgs opencv2 # Create a package
# rospy is python clinet lib for ROS
```

### Create a node
```bash
cd image_pipeline/ && touch image_pipeline/publisher.py # Create a empty file
chmod a+x image_pipeline/publisher.py # Execution permission
```

```bash
publisher.py
```

```python
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # lib to convert between ROS and OpenCV Images
import cv2 

class Publisher(Node):
  def __init__(self):
    super().__init__('publisher')
      
    # Create the publisher. This publisher will publish an image data
    self.publisher_ = self.create_publisher(Image, 'camera/image', 100)
      
    # msg will publish a message every 0.1 seconds
    timer_period = 0.1  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
         
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
   
  def timer_callback(self):
    
    ret, frame = self.cap.read()
          
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame))
 
    # Display the message on the console
    self.get_logger().info('Publishing frame data')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  publisher = Publisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(publisher)
  
  publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
```

### Create a Launch file
```bash
mkdir launch && touch launch/publisher_launch.py # Create a empty lauch file
chmod a+x launch/publisher_launch.py
```
```bash
publisher_launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_pipeline',
            executable='img_publisher',
            output='screen'),
    ])
```

### Update setup.py file
```python
from setuptools import setup
import os 
import glob
package_name = 'image_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/publisher_launch.py']),
        ('share/' + package_name, ['image_pipeline/publisher.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nullbyte',
    maintainer_email='nullbyte.in@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = image_pipeline.publisher:main'
        ],
    },
)

```

### Compile
```bash
cd ~/ros2_ws/ # go to root directory of the packages
colcon build # Compile a ROS package
source install/setup.bash # Source the ROS env variable
```

### RUN
```bash
ros2 launch image_pipeline publisher_launch.py
```

## Create a Subscriber with ROS perception package
### Create a node
```bash
cd image_pipeline/ && touch image_pipeline/subscriber.py # Create a empty file
chmod a+x image_pipeline/subscriber.py # Execution permission
```

```bash
subscriber.py
```
```python
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # lib to convert between ROS and OpenCV Images
import cv2 

class Subscriber(Node):
  def __init__(self):
    super().__init__('subscriber')
      
    # Create the publisher. This publisher will publish an image data
    self.subscriber = self.create_subscription(Image, 'camera/image', self.callback, 100)
    
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()

def callback(self, msg):
    self.get_logger().info('Receiving frame Data')

    # Convert ROS Image message to OpenCV image
    frame = self.bridge.imgmsg_to_cv2(data)

    cv2.imshow("window", frame)
    cv2.waitKey(1)

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  subscriber = Subscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(subscriber)
  
  subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
  ```

### Update Launch file
```bash
publisher_launch.py
```

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
    get_package_share_directory('image_pipeline'),
    'config',
    'parameters.yaml'
    )
    node1 = Node(
            package='image_pipeline',
            executable='img_publisher',
            output='screen',
            parameters = [config]
    )
    node2 = Node(
            package='image_pipeline',
            executable='img_subscriber',
            output='screen',
            parameters = [config]
    )
    ld.add_action(node1)
    ld.add_action(node2)
    return ld
```

### Update setup.py file
```python
from setuptools import setup
import os 
import glob
package_name = 'image_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/publisher_launch.py']),
        ('share/' + package_name, ['image_pipeline/publisher.py']),
        ('share/' + package_name, ['image_pipeline/subscriber.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nullbyte',
    maintainer_email='nullbyte.in@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher  = image_pipeline.publisher:main',
            'img_subscriber = image_pipeline.subscriber:main'
        ],
    },
)

```

### Compile
```bash
cd ~/ros2_ws/ # go to root directory of the packages
colcon build # Compile a ROS package
source install/setup.bash # Source the ROS env variable
```

### RUN
```bash
ros2 launch image_pipeline publisher_launch.py
```




