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