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
import numpy as np

class InitialPoseClient(Node):
    def __init__(self):
        super().__init__('initialpose_ufactory_client')
        self.cli = self.create_client(MoveJoint, '/ufactory/set_servo_angle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveJoint.Request()

    def send_request(self, j1, j2, j3, j4, j5, j6):
        self.req.angles = (float(j1), float(j2), float(j3), float(j4), float(j5), float(j6))
        self.req.speed = 0.90 #0.50
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
        #self.get_logger().info(f'Sending pose: [{x}, {y}, {z}, {px}, {py}, {pz}')
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
    move_cartesian_client = MoveCartesianClient()


    try:
        #Move above container
        #move_pose_client.send_request(0.17, 0.01, 1.29, -3.27, -1.04, 2.13)

        #Move above big box
        #move_pose_client.send_request(-1.04, 0.04, 1.00, -3.31, -0.74, 2.19)
        
        #Move down last layer big box 1
        move_pose_client.send_request(-1.99, -0.36, 1.23, -2.92, -0.84, -1.50)
        #print("step 1")

        #Move down last layer big box 2
        move_pose_client.send_request(-1.95, -0.36, 0.70, -2.67, -0.31, -1.77)
        #print("step 2")

        #Move Cartesian to box middle point
        move_cartesian_client.send_request(-11.65, -160.75, 350.73, -2.39, 0.27, 2.68, 150)

        x = -11.65
        y = -160.75
        z = 350.73
        for i in range(2):
            punto_original = (x, y, z)
            angulo_x = -133.33 #47
            angulo_y = 15.87 #-15.87 
            angulo_z = 160.56 #-20
            traslacion = (-11.7, -160, 350)  # tx, ty, tz

            nuevo_punto = transform_point(*punto_original, angulo_x, angulo_y, angulo_z, *traslacion)

            if i ==0:
                delta_x = 63
                delta_y = -25

            elif i==1:
                delta_x = -4
                delta_y = 0
            else:
                delta_x = -65


            punto_recalculado = rotate_point_inverse(nuevo_punto[0] + delta_x, nuevo_punto[1] +5, nuevo_punto[2] + 290, angulo_x, angulo_y, angulo_z, *traslacion)
  

            #Moving inside box
            move_cartesian_client.send_request(punto_recalculado[0], punto_recalculado[1], punto_recalculado[2], -2.39, 0.27, 2.68, 150)
            

            #Start vacuum gripper
            ufactory_control_vacuum_gripper.send_request(True)
            time.sleep(1.0)

            while True:
                state_gripper = get_state_vacuum_gripper.send_request()
                if state_gripper == 1:
                    #print("Object was taken!")
                    break

            #Moving outside box
            punto_recalculado_back = rotate_point_inverse(nuevo_punto[0], nuevo_punto[1], nuevo_punto[2], angulo_x, angulo_y, angulo_z, *traslacion)
            move_cartesian_client.send_request(punto_recalculado_back[0], punto_recalculado_back[1], punto_recalculado_back[2], -2.39, 0.27, 2.68, 150)

            #Move middle point container
            move_pose_client.send_request(0.00, -0.68, 0.51, -3.41, -0.42, -0.83)

            #Move above container
            move_pose_client.send_request(0.07, 0.13, 1.10, -3.10, -0.93, 0.30)

            #Move linear down
            move_cartesian_client.send_request(263.2, 21.2, 250.7, 3.10, 0.03, 2.88, 100)

            #Stop vacuum gripper
            ufactory_control_vacuum_gripper.send_request(False)
            time.sleep(1.0)

            while True:
                state_gripper = get_state_vacuum_gripper.send_request()
                if state_gripper == 0:
                    #print("vacuum gripper state was updated")
                    break
                time.sleep(0.5)

            #Move above container
            move_pose_client.send_request(0.07, 0.13, 1.10, -3.10, -0.93, -0.20)

            #Move middle point container
            move_pose_client.send_request(0.00, -0.68, 0.51, -3.41, -0.42, -0.83)

            #Move Cartesian to box middle point
            move_cartesian_client.send_request(-11.65, -160.75, 350.73, -2.39, 0.27, 2.68, 100)
            
            


        """
        y = -300.75
        z = 233.13
        for i in range(15):
            delta = i*10
            y_new = y + delta
            z_new = z + 0.84 * delta
            if i ==14:
                move_cartesian_client.send_request(-11.65, y_new, z_new, -2.39, 0.27, 2.68, 50)
                print("Moving to point:", i)
        """
        #Move down last layer big box 3
        #move_pose_client.send_request(-1.80, 0.25, 0.79, -0.26, -0.24, -4.13)
        #print("step 3")

        #Move down last layer big box 4
        #move_pose_client.send_request(-1.78, 0.70, 1.04, -0.06, -0.44, -4.30)
        #print("step 4")

        #Start vacuum gripper
        #ufactory_control_vacuum_gripper.send_request(True)
        time.sleep(1.0)

        #Stop vacuum gripper
        #ufactory_control_vacuum_gripper.send_request(False)
        time.sleep(1.0)

        #Move down last layer big box 3
        #move_pose_client.send_request(-1.80, 0.25, 0.79, -0.26, -0.24, -4.13)

        #Move down last layer big box 2
        #move_pose_client.send_request(-1.95, -0.36, 0.70, -2.67, -0.31, -1.77)

        #Move down last layer big box 1
        #move_pose_client.send_request(-1.99, -0.36, 1.23, -2.92, -0.84, -1.50)

        
    except KeyboardInterrupt:
        pass
    finally:
        move_pose_client.destroy_node()
        ufactory_control_vacuum_gripper.destroy_node()
        get_state_vacuum_gripper.destroy_node()
        rclpy.shutdown()

def rotate_point(x, y, z, alpha_deg, beta_deg, gamma_deg):

    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx

    P = np.array([x, y, z])

 
    P_rot = R @ P

    return P_rot


def transform_point(x, y, z, alpha_deg, beta_deg, gamma_deg, tx=0, ty=0, tz=0):

    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)
    

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])


    R = Rz @ Ry @ Rx


    P = np.array([x, y, z])


    P_transformado = R @ P + np.array([tx, ty, tz])

    return P_transformado

def rotate_point_inverse(xp, yp, zp, alpha_deg, beta_deg, gamma_deg, tx=0, ty=0, tz=0):
    
    alpha = np.radians(alpha_deg)
    beta = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)
    

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha), np.cos(alpha)]
    ])

    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma), np.cos(gamma), 0],
        [0, 0, 1]
    ])

 
    R = Rz @ Ry @ Rx


    P_rot = np.array([xp, yp, zp])
    T = np.array([tx, ty, tz])

 
    R_inv = R.T


    P_original = R_inv @ (P_rot - T)

    return P_original

if __name__ == '__main__':
    main()