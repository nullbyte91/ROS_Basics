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