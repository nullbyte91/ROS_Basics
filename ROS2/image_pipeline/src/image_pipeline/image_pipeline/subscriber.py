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
      frame = self.bridge.imgmsg_to_cv2(msg)

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