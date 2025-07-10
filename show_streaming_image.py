import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import time

class RGBImageSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_image_subscriber')
        self.points = []  # Save current points
        self.max_points_age = 2.0  # time to wait

        #Create subscriber to get camera raw image
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  
            self.listener_callback,
            10
        )

        # Subscriber to get pixels coordinates
        self.point_sub = self.create_subscription(
            Point,
            'detected_pixeles_coordinates',
            self.point_callback,
            10
        )
        
        self.point = None
        self.bridge = CvBridge()
        self.get_logger().info("RGB Image Subscriber initialized")

    def point_callback(self, msg):
        # Save the last received point
        if msg.x == -1.0 and msg.y == -1.0:
            self.point = None  # Delete last point
            self.get_logger().info("No detections. Cleared point.")
        else:
            x_scaled = int(msg.x * 2.0)
            y_scaled = int(msg.y * 1.125)
            timestamp = time.time()
            self.points.append((x_scaled, y_scaled, timestamp))

    def listener_callback(self, msg):
        try:
            # Conert image from ROS to openCv
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            current_time = time.time()
            self.points = [pt for pt in self.points if current_time - pt[2] < self.max_points_age]

            for (x, y, _) in self.points:
                cv2.circle(cv_image, (x, y), radius=5, color=(0, 0, 255), thickness=-1)
            cv2.imshow("RGB Camera View", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RGBImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()