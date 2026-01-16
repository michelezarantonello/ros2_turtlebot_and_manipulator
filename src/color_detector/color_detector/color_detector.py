import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped

import cv2
import numpy as np

class ColorDetector(Node):

   def __init__(self):
      super().__init__('color_detector')

      rclpy.logging.get_logger('ColorDetector').info('Color Detector node started')   

      # Create subscriber to RGB camera images
      self.subscription = self.create_subscription(
         Image,
         '/rgb_camera/image',
         self.image_callback,
         10
      )
      self.bridge = CvBridge()

      # Create publishers for detected color centroids
      self.centroid_publisher_red = self.create_publisher(
         PointStamped, 
         '/color_detector/red_cube', 
         10)
      
      self.centroid_publisher_blue = self.create_publisher(
         PointStamped, 
         '/color_detector/blue_cube', 
         10)

      # Store latest centroids
      self.red_centroid = None
      self.blue_centroid = None
      self.latest_frame_id = None
      self.latest_timestamp = None

      # Create timers to limit publish rate (Publishing continuously causes problems)
      publish_rate = 1.0  # Hz
      timer_period = 1.0 / publish_rate
      self.create_timer(timer_period, self.publish_centroids)

   def image_callback(self, msg):
      # Convert ROS Image to OpenCV
      frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

      # Convert BGR to HSV color space for easier color detection
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

      # Detect red and blue cubes
      red_mask = self.detect_red(hsv)
      blue_mask = self.detect_blue(hsv)

      # Calculating centroids of detected pixels
      self.red_centroid = self.mask_centroid(red_mask)
      self.blue_centroid = self.mask_centroid(blue_mask)

      # Store frame metadata for publishing
      self.latest_frame_id = msg.header.frame_id
      self.latest_timestamp = self.get_clock().now().to_msg()

   def detect_red(self, hsv):
      # Red color thresholds (tuned)
      lower1 = np.array([0, 128, 50])
      upper1 = np.array([10, 255, 255])
      lower2 = np.array([170, 128, 50])
      upper2 = np.array([180, 255, 255])

      mask1 = cv2.inRange(hsv, lower1, upper1)
      mask2 = cv2.inRange(hsv, lower2, upper2)
      return mask1 + mask2

   def detect_blue(self, hsv):
      # Blue color thresholds (tuned)
      lower = np.array([100, 128, 50])
      upper = np.array([140, 255, 255])

      mask = cv2.inRange(hsv, lower, upper)
      return mask

   def mask_centroid(self, mask):
      # Calculate moments of the binary image
      moments = cv2.moments(mask, binaryImage=True)

      if moments["m00"] == 0:
        return None  # No pixels detected

      # Calculate centroid coordinates
      cx = int(moments["m10"] / moments["m00"])
      cy = int(moments["m01"] / moments["m00"])
      return (cx, cy)

   def publish_centroids(self):
      # This method is called periodically by the timer
      if self.latest_frame_id is None:
         return

      # Publish red centroid if detected
      if self.red_centroid is not None:
         point = PointStamped()
         point.header.stamp = self.latest_timestamp
         point.header.frame_id = self.latest_frame_id
         point.point.x = float(self.red_centroid[0])
         point.point.y = float(self.red_centroid[1])
         point.point.z = 0.0
         self.centroid_publisher_red.publish(point)
         rclpy.logging.get_logger('ColorDetector').info('Published red centroid, time stamp: %s' % str(self.latest_timestamp.sec))   

      # Publish blue centroid if detected
      if self.blue_centroid is not None:
         point = PointStamped()
         point.header.stamp = self.latest_timestamp
         point.header.frame_id = self.latest_frame_id
         point.point.x = float(self.blue_centroid[0])
         point.point.y = float(self.blue_centroid[1])
         point.point.z = 0.0
         self.centroid_publisher_blue.publish(point)
         rclpy.logging.get_logger('ColorDetector').info('Published blue centroid, time stamp: %s' % str(self.latest_timestamp.sec))

def main(args=None):
      rclpy.init()
      node = ColorDetector()
      rclpy.spin(node)
      node.destroy_node()
      rclpy.shutdown()
