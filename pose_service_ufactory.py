import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from xarm_msgs.msg import RobotMsg
from geometry_msgs.msg import Pose, Point, PointStamped
from xarm_msgs.srv import MoveCartesian, MoveJoint, VacuumGripperCtrl, GetInt16
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from array import array
import time

class InitialPoseClient(Node):

    def __init__(self):
        super().__init__('initialpose_ufactory_client')
        self.cli = self.create_client(MoveJoint, '/ufactory/set_servo_angle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveJoint.Request()

    def send_request(self, j1, j2, j3, j4, j5, j6):
        self.req.angles = (float(j1), float(j2), float(j3), float(j4), float(j5), float(j6))
        self.req.speed = 0.80
        self.req.mvtime = 0.0
        self.req.wait = False
        self.req.timeout = -1.0
        self.req.radius = -1.0
        self.req.relative = False
        self.req.acc = 10.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
class RobotStateSubscriber(Node):
    def __init__(self):
        super().__init__('robot_state_subscriber')
        self.subscription = self.create_subscription(
            RobotMsg,              
            '/ufactory/robot_states',
            self.state_callback,
            10
        )
        self.latest_state = None
        self.get_logger().info("Subscribed from /ufactory/robot_states")

    def state_callback(self, msg):
        self.latest_state = msg
        self.get_logger().debug(f"State received: {msg}")

    def get_latest_state(self):
        return self.latest_state

class VacuumGripperUFactoryClient(Node):
    def __init__(self):
        super().__init__('vacuum_gripper_ufactory_client')
        self.cli = self.create_client(VacuumGripperCtrl, '/ufactory/set_vacuum_gripper')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = VacuumGripperCtrl.Request()
        self.running = True

    def send_request(self, is_active):
        self.req.on = is_active
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)


class GetVacuumGripperState(Node):
    def __init__(self):
        super().__init__('get_vacuum_gripper_state')
        self.cli = self.create_client(GetInt16, '/ufactory/get_vacuum_gripper')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetInt16.Request()
        self.running = True

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        resp = self.future.result()
        return resp.data

class PoseSubscriberClient(Node):
    def __init__(self):
        super().__init__('raspberry_pose_subscriber_client')

        # Client to move the arm in Cartesian coordinates
        self.cli = self.create_client(MoveCartesian, '/ufactory/set_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ufactory/set_position service...')
        self.req = MoveCartesian.Request()
        self.initial_pose_client = InitialPoseClient()
        self.ufactory_control_vacuum_gripper = VacuumGripperUFactoryClient()
        self.get_state_vacuum_gripper = GetVacuumGripperState()
        self.robot_state_sub = RobotStateSubscriber()


         # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscription to the topic that a Pose publishes
        self.subscription = self.create_subscription(
            Point,
            'detected_object_coordinates',
            self.pose_callback,
            10
        )

        self.get_logger().info('Node started. Waiting for pose at /detected_object_coordinates...')

    def pose_callback(self, msg):
        try:
            """
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
            z = transformed_point.point.z + 10
            """

            x = msg.x
            y = msg.y
            z = msg.z

            #Move robot arm to initial robot state
            self.initial_pose_client.send_request(0.15, -0.02, 1.52, 0.02, 1.57, 1.60)

            #Move to detected Pose + z offset(2cm)
            z_offset = z + 20
            self.get_logger().info(f'Move to detected Pose + z offset(5m): x={x:.2f}, y={y:.2f}, z={z_offset:.2f}')
            self.send_request(x, y, z_offset, 100)

            #Start Vacuum Gripper
            self.ufactory_control_vacuum_gripper.send_request(True)

            
            #Slow linear move to get raspberry pi
            while True:
                state_gripper = self.get_state_vacuum_gripper.send_request()

                if state_gripper == 1:
                    self.get_logger().info("Vacuum gripper ha detectado succión. Saliendo del bucle.")
                    break

                robot_state = self.robot_state_sub.get_latest_state()
                if robot_state is not None:
                    #pose = robot_state.pose
                    pose_list = robot_state.pose
                    x = pose_list[0]
                    y = pose_list[1]
                    z = pose_list[2] 
                    self.get_logger().info(f"Pose actual: x={x}, y={y}, z={z}")

                    # Calcular nueva posición descendiendo 5mm
                    new_z_pose = z - 5.0

                    # Enviar nuevo movimiento con menor velocidad
                    self.send_request(x, y, new_z_pose, 20)
                else:
                    self.get_logger().warn("robot_state is None, esperando mensaje...")
                    time.sleep(0.1)
                
            #Linear move up
            self.send_request(x, y, z_offset, 100)

            #Move to goal Pose (up)
            self.initial_pose_client.send_request(-1.61, -0.02, 1.52, 0.02, 1.57, 1.60)

            #Move to goal Pose (down)
            self.initial_pose_client.send_request(-1.60, 1.60, 1.33, -0.04, -0.24, 1.64)

            #Stop Vacuum Gripper
            self.ufactory_control_vacuum_gripper.send_request(False)

            #Move robot arm to goal pose (up)(again)
            self.initial_pose_client.send_request(-1.61, -0.02, 1.52, 0.02, 1.57, 1.60)

            #Move robot arm to initial position
            self.initial_pose_client.send_request(0.15, -0.02, 1.52, 0.02, 1.57, 1.60)

        except Exception as e:
            self.get_logger().warn(f'Error transforming pose: {str(e)}')

    def send_request(self, x, y, z, velocity):
        # Send request to cartesian move
        self.req.pose = array('f', [float(x), float(y), float(z),-3.14, 0.03, -0.17])
        self.get_logger().info(f'Sending pose: [{x}, {y}, {z}, -3.14, 0.00, -1.47')
        self.req.speed = float(velocity) #100
        self.req.acc = 0.0 
        self.req.mvtime = 0.0
        self.req.wait = True
        self.req.timeout = -1.0
        self.req.radius = -1.0

        self.get_logger().info('Calling service...')
        future = self.cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        #future.add_done_callback(callback)
        if future.done():
            result = future.result()
            if result is not None:
                self.get_logger().info('Service answered.')
                self.get_logger().info(f'Movement sent successfully. Result: {result}')
            else:
                self.get_logger().error('Future result is None.')
        else:
            self.get_logger().error('Timeout waiting for set_position service response.')

        if future.result() is not None:
            self.get_logger().info('Movement sent successfully.')
        else:
            self.get_logger().error('Error calling the service.')

def callback(future):
    result = future.result()

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberClient()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.robot_state_sub)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.robot_state_sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
