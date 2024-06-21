import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import os


INPUT_TOPIC = os.getenv("INPUT_TOPIC", "/robot/front_rgbd_camera/rgb/image_raw/compressed")
OUTPUT_TOPIC = os.getenv("OUTPUT_TOPIC", "/image_raw")


class CompressedImageToImageNode(Node):
    def __init__(self):
        super().__init__('compressed_image_to_image_node')
        
        # Create a CvBridge object
        self.bridge = CvBridge()
        
        # Subscriber for compressed image
        self.subscription = self.create_subscription(
            CompressedImage,
            INPUT_TOPIC,
            self.listener_callback,
            10)
        
        # Publisher for normal image
        self.publisher = self.create_publisher(
            Image,
            OUTPUT_TOPIC,
            10)
        
        self.get_logger().info('Node has been started.')
        self.get_logger().info(f"Input topic: {INPUT_TOPIC}")
        self.get_logger().info(f"Output topic: {OUTPUT_TOPIC}")
        
    
    def listener_callback(self, msg):
        try:
            # Convert compressed image to CV2 image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'rgb8')
            cv_image = cv2.resize(cv_image, (740, 416), interpolation = cv2.INTER_LINEAR)
            
            # Convert CV2 image to ROS2 Image message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'rgb8')
            image_msg.header = msg.header
            
            # Publish the Image message
            self.publisher.publish(image_msg)
            #self.get_logger().info('Image published successfully.')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageToImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
