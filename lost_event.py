import rclpy
import numpy as np
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import requests


def calculate_dispersion(particles):

    # Extract x and y coordinates of the poses
    x_coords = np.array([particle.pose.position.x for particle in particles])
    y_coords = np.array([particle.pose.position.x for particle in particles])
    # Calculate the centroid (mean) of the point cloud
    centroid_x = np.mean(x_coords)
    centroid_y = np.mean(y_coords)
    # Calculate the standard deviation of distances from the centroid
    distances = np.sqrt((x_coords - centroid_x)**2 + (y_coords - centroid_y)**2)
    dispersion = np.std(distances)
    return dispersion

class ParticleCloudSubscriber(Node):
    def __init__(self):
        super().__init__('particlecloud_subscriber')
        #Create quality of service for ParticleCloud
        particle_cloud_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.subscription_particle_cloud = self.create_subscription(ParticleCloud, '/particle_cloud', self.particlecloud_callback, particle_cloud_qos)
        self.publisher_localization_precision = self.create_publisher(String, 'localization_precision', 10)

    def particlecloud_callback(self, msg:ParticleCloud):
        # Extract pose data from the PoseArray
        particles = msg.particles
        # Calculate dispersion within the particle cloud
        dispersion = calculate_dispersion(particles)
        # Print the dispersion value
        self.get_logger().info(f"Dispersion within the particle cloud: {dispersion:.2f}")
        
        signal_strength = "UNKNOWN"
        message = String()
        message.data = signal_strength

        if dispersion < 0.4:
            signal_strength = "GOOD"
        
        elif  dispersion > 2:
            signal_strength = "LOST"

        elif  (dispersion <= 2.0) and (dispersion > 0.4):
            signal_strength = "POOR"
        message.data = str(signal_strength)
        # Replace 'your_api_endpoint' with the actual API endpoint you're targeting.

        
        self.publisher_localization_precision.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()