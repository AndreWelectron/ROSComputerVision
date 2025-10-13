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
        self.req.acc = 5.0
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
        self.move_pose_client = InitialPoseClient()
        self.ufactory_control_vacuum_gripper = VacuumGripperUFactoryClient()
        self.get_state_vacuum_gripper = GetVacuumGripperState()
        self.robot_state_sub = RobotStateSubscriber()

        self.already_executed = False  #Flag for executed detection


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

        if self.already_executed:
            self.get_logger().info("Move was executed for this detection, Msg from topic is ignored.")
            return

        self.already_executed = True  # Flag for executed detection

        try:
            num_layers = 5              #Layers in container
            z_layer_offset = 26         #Height z_layer_offset
            flag_layer = 0

            #layer = 4
            #z_layer = msg.z + layer * z_layer_offset

            """
            for col in [1,2]:
                for idx in range(1,5): #(1,2)
                    if col == 1:
                        #self.handle_column_1(idx, msg.x, msg.y, z_layer)
                        print("pass... column1")
                    elif col== 2:
                        if layer > 2:
                            z_layer_new = z_layer -15
                            #z_layer = z_layer
                            flag_layer = 1
                        self.handle_column_2(idx, msg.x, msg.y, z_layer_new, flag_layer, layer)

            """

            #Auskommentieren!!!
            for layer in range(num_layers):
                z_layer = msg.z + layer * z_layer_offset

                for col in [1,2]:
                    for idx in range(1,5):
                        if col == 1:
                            self.handle_column_1(idx, msg.x, msg.y, z_layer)
                            #print("pass... column1")
                        elif col== 2:
                            z_layer_new = z_layer 
                            if layer > 2:
                                z_layer_new = z_layer -15
                                #z_layer = z_layer
                                flag_layer = 1
                            self.handle_column_2(idx, msg.x, msg.y, z_layer_new, flag_layer, layer)
            

            #Place box 9
            #self.handle_box_9(msg.x, msg.y, msg.z)

            #Place box 10
            #self.handle_box_10(msg.x, msg.y, msg.z)

        except Exception as e:
            self.get_logger().warn(f'Error transforming pose: {str(e)}')

    

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

        self.get_logger().info('Calling service...')
        future = self.cli.call_async(self.req)

        #rclpy.spin_until_future_complete(self, future)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)#4.0
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
    
    def handle_column_1(self, idx, x, y, z):
        #Move to pick box (up) state
        self.move_pose_client.send_request(1.03, 0.38, 1.29, -0.03, 0.95, -0.50)

        #Move to pick box (down) state
        #(1.03, 0.96, 1.22, -0.11, 0.30, -0.50)
        self.move_pose_client.send_request(1.03, 1.04, 1.26, -0.13, 0.26, -0.48)

        #Start vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(True)

        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            if state_gripper == 1:
                self.get_logger().info("Vacuum gripper detected object. Break loop!")
                break

        #Move to pick box (down) state
        #self.move_pose_client.send_request(1.03, 0.96, 1.22, -0.11, 0.30, -0.50)

        #Move to pick box (up) state
        self.move_pose_client.send_request(1.03, 0.38, 1.29, -0.03, 0.95, -0.50)

        #Move state to midpoint container
        self.move_pose_client.send_request(0.09, 0.63, 1.64, -0.05, 1.01, -1.43)

        #Move to Anfangspunkt matrix
        self.send_request(x, y, z, 3.14 , 0.04, 1.55, 80)

        y_goal = y - 46.0
        y_intermediate = y_goal +14
        #Move above glide punkt
        self.send_request(x, y_intermediate, z, 3.14 , 0.04, 1.55, 80)#50

        z_new = z - 90 #CHECK!!!
        #Move (down) glide punkt
        self.send_request(x, y_intermediate, z_new, 3.14 , 0.04, 1.55, 80)#50

        if idx in [1, 2]:
            x_offset = x + 124
        elif idx == 3:
            x_offset = x + 93
        else:
            x_offset = x + 31 # For i == 4

        #Move to goal position after glide in x axis
        self.send_request(x_offset, y_intermediate, z_new, 3.14 , 0.04, 1.55, 80)#50

        #Move to goal position after glide in y axis
        self.send_request(x_offset, y_goal, z_new, 3.14 , 0.04, 1.55, 80)#50

        #Stop vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(False)
        time.sleep(1.0)

        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            self.get_logger().info(f"Waiting gripper to change state value: {state_gripper}")
            if state_gripper == 0:
                break
            time.sleep(0.5)

        state_gripper = self.get_state_vacuum_gripper.send_request()
        self.get_logger().info(f"State vacuum gripper: {state_gripper}")

        #Move state to midpoint container
        self.move_pose_client.send_request(0.09, 0.63, 1.64, -0.05, 1.01, -1.43)

        #Move to pick box (up) state
        self.move_pose_client.send_request(1.03, 0.38, 1.29, -0.03, 0.95, -0.50)

    def handle_column_2(self, idx, x, y, z, flag_layer, layer):
        #Move to pick box (up) state for second column
        self.move_pose_client.send_request(0.99, 0.39, 1.31, -3.06, -0.97, -2.43)
        
        #Move to pick box (down) state for second column
        #(1.00, 0.99, 1.27, -2.93, -0.33, -2.57)
        self.move_pose_client.send_request(0.99, 1.04, 1.28, -2.9, -0.29, -2.61)

        #Start vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(True)

        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            if state_gripper == 1:
                self.get_logger().info("Vacuum gripper detected object. Break loop!")
                break
        
        #Move to pick box (up) state for second column
        self.move_pose_client.send_request(0.99, 0.39, 1.31, -3.06, -0.97, -2.43)

        #Move state to midpoint container for second column
        self.move_pose_client.send_request(0.1, 0.23, 0.92, -3.14, -0.77, -3.28)

        #Move to Anfangspunkt matrix
        self.send_request(x, y, z, -3.13, 0.08, 0.23, 80) #vel 50

        if layer == 4:
            y_goal = y + 44.0
        else:
            y_goal = y + 49.0 #46

        #y_goal = y + 49.0 #46
        z_new = z - 40 #CHECK!!!
        
        if idx == 1:
            x_offset = x + 186 #187
        elif idx == 2:
            x_offset = x + 123  #x+124
        elif idx == 3:
            x_offset = x +59  #x+62
        else:
            x_offset = x - 4  # For j == 4 (nur x)
        
        if flag_layer ==1:
            self.send_request(x_offset, y_goal, z, -3.13, 0.08, 0.23, 80) #vel 50
        
        #Move above goal position
        self.send_request(x_offset, y_goal, z_new, -3.13, 0.08, 0.23, 80) #vel 50
        self.get_logger().info(f"Going to the following point layer: {x_offset, y_goal, z_new}")

        #Rotate TCP in RX
        self.send_request(x_offset, y_goal, z_new, -2.96, 0.08, 0.23, 80) #vel 50

        
        z_goal = z_new -55 
        if flag_layer ==1:
            z_goal = z_goal + 12
            self.get_logger().info(f"Going down across z axis: {z_goal}")

        #Move slow to goal position (down)
        self.send_request(x_offset, y_goal, z_goal, -2.96, 0.08, 0.23, 50) #vel 50

        if idx ==4:
            x_push = x_offset + 43  #+37
            #push all boxes to final position
            self.send_request(x_push, y_goal, z_goal, -2.96, 0.08, 0.23, 80) #vel 50     
        
        #Stop vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(False)
        time.sleep(1.0)
        
        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            self.get_logger().info(f"Waiting gripper to change state value: {state_gripper}")
            if state_gripper == 0:
                break
            time.sleep(0.5)

        #Move above goal position
        self.send_request(x_offset, y_goal, z_new, -3.13, 0.08, 0.23, 80) #vel 50

        #Move state to midpoint container for second column
        self.move_pose_client.send_request(0.1, 0.23, 0.92, -3.14, -0.77, -3.28)

        #Move to pick box (up) state for second column
        self.move_pose_client.send_request(0.99, 0.39, 1.31, -3.06, -0.97, -2.43)

        
        

    
    def handle_box_9(self,x, y, z):
        self.move_pose_client.send_request(1.03, 0.38, 1.29, -0.03, 0.95, -0.50)

        #Move to pick box (down) state
        self.move_pose_client.send_request(1.03, 1.04, 1.26, -0.13, 0.26, -0.48)

        #Start vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(True)

        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            if state_gripper == 1:
                self.get_logger().info("Vacuum gripper detected object. Break loop!")
                break

        #Move to pick box (down) state
        #self.move_pose_client.send_request(1.03, 0.96, 1.22, -0.11, 0.30, -0.50)

        #Move to pick box (up) state
        self.move_pose_client.send_request(1.03, 0.38, 1.29, -0.03, 0.95, -0.50)

        #Move state to midpoint container
        self.move_pose_client.send_request(0.09, 0.63, 1.64, -0.05, 1.01, -1.43)

        #Move to Anfangspunkt matrix
        self.send_request(x, y, z, 3.14 , 0.04, 1.55, 80) #vel 50

        x_new_9 = x - 31
        #Move above insertion punkt
        self.send_request(x_new_9, y, z, 3.14 , 0.04, 1.55, 80) #vel 50

        #Rotate TCP to enter into columns
        new_px = -2.76 #3.14 - 5.96
        new_py = 0.01 #0.04 +0.005
        new_pz = 1.64
        self.send_request(x_new_9, y, z,new_px , new_py, new_pz, 80) #vel 50

        #Move in x positive axis to align input position
        x_input_position = 231.77
        self.send_request(x_input_position, y, z,new_px , new_py, new_pz, 50) #vel 50

        #Move in z axis down
        z_down_align = z - 87
        self.send_request(x_input_position, y, z_down_align, new_px , new_py, new_pz, 50) #vel 50

        #Move back in y axis
        y_back_wall = y - 55.8
        self.send_request(x_input_position, y_back_wall, z_down_align, new_px , new_py, new_pz, 50) #vel 50

        #Move down in goal position
        z_goal = z_down_align -14.1
        self.send_request(x_input_position, y_back_wall, z_goal ,new_px , new_py, new_pz, 50) #vel 50

        #Stop vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(False)
        time.sleep(1.0)

        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            self.get_logger().info(f"Waiting gripper to change state value: {state_gripper}")
            if state_gripper == 0:
                break
            time.sleep(0.5)

        state_gripper = self.get_state_vacuum_gripper.send_request()
        self.get_logger().info(f"State vacuum gripper: {state_gripper}")

        #Move state to midpoint container
        self.move_pose_client.send_request(0.09, 0.63, 1.64, -0.05, 1.01, -1.43)

        #Move to pick box (up) state
        self.move_pose_client.send_request(1.03, 0.38, 1.29, -0.03, 0.95, -0.50)

        
    def handle_box_10(self, x, y ,z):
        #Move to pick box (up) state for second column
        self.move_pose_client.send_request(0.99, 0.39, 1.31, -3.06, -0.97, -2.43)

        #Move to pick box (down) state for second column
        self.move_pose_client.send_request(0.99, 1.04, 1.28, -2.9, -0.29, -2.61)

        #Start vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(True)

        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            if state_gripper == 1:
                self.get_logger().info("Vacuum gripper detected object. Break loop!")
                break

        #Move to pick box (down) state for second column
        #self.move_pose_client.send_request(1.03, 0.96, 1.22, -0.11, 0.30, -0.50)

        #Move to pick box (up) state for second column
        self.move_pose_client.send_request(0.99, 0.39, 1.31, -3.06, -0.97, -2.43)

        #Move state to midpoint container for second column
        self.move_pose_client.send_request(0.1, 0.23, 0.92, -3.14, -0.77, -3.28)

        #Move to Anfangspunkt matrix for second column
        self.send_request(x, y, z, -3.13, 0.08, 0.23, 80) #vel 50

        #Rotate TCP to enter into 10 position
        new_py = 0.44 #0.08 + 0.36
        self.send_request(x, y, z, -3.13, new_py, 0.23, 80) #vel 50

        #Move above insertion position (in y-axis)
        x_new = x + 3.60
        y_new = y + 57 #48.68
        self.send_request(x_new, y_new, z, -3.13, new_py, 0.23, 80)

        #Move above insertion position (in z-axis)
        z_new = z -51.83
        self.send_request(x_new, y_new, z_new, -3.13, new_py, 0.23, 80)

        #Move down in negative z_axis 
        z_insertion = z_new - 35
        self.send_request(x_new, y_new, z_insertion, -3.13, new_py, 0.23, 50)

        #Stop vacuum gripper
        self.ufactory_control_vacuum_gripper.send_request(False)
        time.sleep(1.0)

        while True:
            state_gripper = self.get_state_vacuum_gripper.send_request()
            self.get_logger().info(f"Waiting gripper to change state value: {state_gripper}")
            if state_gripper == 0:
                break
            time.sleep(0.5)

        state_gripper = self.get_state_vacuum_gripper.send_request()
        self.get_logger().info(f"State vacuum gripper: {state_gripper}")

        #Move above position for push down function 
        x_push = x_new - 21.81
        y_push = y_new + 1.00
        z_push = z_insertion + 67 #+87

        self.send_request(x_push, y_push, z_push, -3.13, new_py, 0.23, 50)

        #Push down
        z_push_final = z_push - 70 #+90
        self.send_request(x_push, y_push, z_push_final, -3.13, new_py, 0.23, 50)
        #self.get_logger().info(f"Pushing down box 10: {z_push_final}")

        #Move up
        z_up = z_push_final + 10 #+10
        x_middle = x - 31
        y_middle = y + 47.5
        self.send_request(x_middle, y_middle, z_up, -3.13, 0.08, 0.23, 80)

        #Move to middle of box
        z_middle = z_push_final - 10
        self.send_request(x_middle, y_middle, z_middle, -3.13, 0.08, 0.23, 80)

        #Move up
        z_up = z_push_final + 134
        self.send_request(x_middle, y_middle, z_up, -3.13, new_py, 0.23, 80)

        #Move state to midpoint container for second column
        self.move_pose_client.send_request(0.1, 0.23, 0.92, -3.14, -0.77, -3.28)


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
