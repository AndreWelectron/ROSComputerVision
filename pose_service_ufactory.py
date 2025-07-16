import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, PointStamped
from xarm_msgs.srv import MoveCartesian
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros


class PoseSubscriberClient(Node):
    def __init__(self):
        super().__init__('raspberry_pose_subscriber_client')

        # Client to move the arm in Cartesian coordinates
        self.cli = self.create_client(MoveCartesian, 'ufactory/set_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ufactory/set_position service...')
        self.req = MoveCartesian.Request()

         # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscription to the topic that a Pose publishes
        self.subscription = self.create_subscription(
            PointStamped,
            'detected_object_coordinates',
            self.pose_callback,
            10
        )

        self.get_logger().info('Node started. Waiting for pose at /detected_object_coordinates...')

    def pose_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                    'realsense_link',               # Frame final
                    msg.header.frame_id,      # Frame original 
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

            # Apply transformation to received point
            transformed_point = do_transform_point(msg, transform)
            x = transformed_point.point.x
            y = transformed_point.point.y
            z = transformed_point.point.z

            self.get_logger().info(f'Transformed pose received: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            self.send_request(x, y, z)

        except Exception as e:
            self.get_logger().warn(f'Error transforming pose: {str(e)}')

    def send_request(self, x, y, z):
        # Send request to cartesian move
        self.get_logger().info(f'Sending pose: [{x}, {y}, {z}, 3.14, 0.0, 0.0]')
        self.req.pose = [x, y, z, 3.14, 0.0, 0.0]  # Position XYZ + fixed orientation 
        self.req.speed = 100.0
        self.req.mvtime = 0.0
        self.req.wait = True
        self.req.timeout = -1.0
        self.req.radius = -1.0

        self.get_logger().info('Calling service...')
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Service answered.')

        if future.result() is not None:
            self.get_logger().info('Movement sent successfully.')
        else:
            self.get_logger().error('Error calling the service.')

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
