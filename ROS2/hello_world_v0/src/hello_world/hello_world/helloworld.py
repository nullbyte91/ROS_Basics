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