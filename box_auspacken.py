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
        self.req.speed = 0.50 #0.50
        self.req.mvtime = 0.0
        self.req.wait = False
        self.req.timeout = -1.0
        self.req.radius = -1.0
        self.req.relative = False
        self.req.acc = 5.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
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

class MoveCartesianClient(Node):
    def __init__(self):
        super().__init__('raspberry_pose_subscriber_client')

        # Client to move the arm in Cartesian coordinates
        self.cli = self.create_client(MoveCartesian, '/ufactory/set_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ufactory/set_position service...')
        self.req = MoveCartesian.Request()

    def send_request(self, x, y, z, px, py, pz, velocity):
        # Send request to cartesian move
        self.req.pose = array('f', [float(x), float(y), float(z), float(px) , float(py), float(pz)])
        self.get_logger().info(f'Sending pose: [{x}, {y}, {z}, {px}, {py}, {pz}')
        self.req.speed = float(velocity) #100
        self.req.acc = 0.0 
        self.req.mvtime = 0.0
        self.req.wait = True
        self.req.timeout = -1.0
        self.req.radius = -1.0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)


def main(args=None):
    rclpy.init(args=args)
    move_pose_client = InitialPoseClient()
    ufactory_control_vacuum_gripper = VacuumGripperUFactoryClient()
    get_state_vacuum_gripper = GetVacuumGripperState()


    try:
        #Move above container
        #move_pose_client.send_request(0.17, 0.01, 1.29, -3.27, -1.04, 2.13)

        #Move above big box
        #move_pose_client.send_request(-1.04, 0.04, 1.00, -3.31, -0.74, 2.19)
        
        #Move down last layer big box 1
        move_pose_client.send_request(-1.99, -0.36, 1.23, -2.92, -0.84, -1.50)

        #Move down last layer big box 2
        move_pose_client.send_request(-1.95, -0.36, 0.70, -2.67, -0.31, -1.77)

        #Move down last layer big box 3
        move_pose_client.send_request(-1.80, 0.25, 0.79, -0.26, -0.24, -4.13)

        #Move down last layer big box 4
        move_pose_client.send_request(-1.78, 0.70, 1.04, -0.06, -0.44, -4.30)

        #Start vacuum gripper
        ufactory_control_vacuum_gripper.send_request(True)
        time.sleep(1.0)

        #Stop vacuum gripper
        ufactory_control_vacuum_gripper.send_request(False)
        time.sleep(1.0)

        #Move down last layer big box 3
        move_pose_client.send_request(-1.80, 0.25, 0.79, -0.26, -0.24, -4.13)

        #Move down last layer big box 2
        move_pose_client.send_request(-1.95, -0.36, 0.70, -2.67, -0.31, -1.77)

        #Move down last layer big box 1
        move_pose_client.send_request(-1.99, -0.36, 1.23, -2.92, -0.84, -1.50)

        
    except KeyboardInterrupt:
        pass
    finally:
        move_pose_client.destroy_node()
        ufactory_control_vacuum_gripper.destroy_node()
        get_state_vacuum_gripper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()