import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tritonclient.utils import InferenceServerException
import tritonclient.http as httpclient
from geometry_msgs.msg import Point

SAVE_INTERMEDIATE_IMAGES = False

class ImageSubscriberTritonClient(Node):
    def __init__(self):
        super().__init__('image_subscriber_triton_client')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",  
            self.listener_callback,
            10
        )
        self.subscription_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.subscription_info = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self.info_callback, 10)
        self.bridge = CvBridge()
        self.intrinsics = None
        self.depth_image = None

        # Triton client config
        self.triton_url = '192.168.0.180:8000'  
        self.model_name = 'raspberrypi_detection'  # Change model name!
        self.model_input_name = 'images'      # Change input name!
        self.model_output_name = 'output0'    # Change output name!
        self.client = httpclient.InferenceServerClient(url=self.triton_url)
        self.coordinates_publisher = self.create_publisher(Point, 'detected_object_coordinates', 10)
        self.pixeles_publisher = self.create_publisher(Point, 'detected_pixeles_coordinates', 10)



    def info_callback(self, msg: CameraInfo):
        self.intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")


    def preprocess(self, cv_image):
        resized = cv2.resize(cv_image, (640, 640))
        normalized = resized.astype(np.float32)
        normalized = normalized / 255.0  
        img = np.transpose(normalized, (2, 0, 1)) 
        input_data = np.expand_dims(img, axis=0)  
        return input_data
    
    def listener_callback(self, msg: Image):

        if self.depth_image is None or self.intrinsics is None:
            return  

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        input_tensor = self.preprocess(cv_image)

        inputs = [
            httpclient.InferInput(self.model_input_name, input_tensor.shape, "FP32")
        ]
        inputs[0].set_data_from_numpy(input_tensor)

        outputs = [
            httpclient.InferRequestedOutput(self.model_output_name)
        ]

        try:
            response = self.client.infer(
                model_name=self.model_name,
                inputs=inputs,
                outputs=outputs
            )
            result = response.as_numpy(self.model_output_name)
            boxes, scores = self.postprocess_output(result, conf_thresh=0.3)
            self.get_logger().info(f"Detections: {len(boxes)}")
            for i, (x_pixel, y_pixel) in enumerate(boxes):
                x_int = int(round(x_pixel))
                y_int = int(round(y_pixel))

                if y_int >= self.depth_image.shape[0] or x_int >= self.depth_image.shape[1]:
                    self.get_logger().warn(f"Coordinates out of bounds: ({x_int}, {y_int})")
                    continue

                z = self.depth_image[y_int, x_int] / 1000.0  

                if z == 0:
                    self.get_logger().warn("Depth value not accepted")
                    continue

                fx = self.intrinsics['fx']
                fy = self.intrinsics['fy']
                cx = self.intrinsics['cx']
                cy = self.intrinsics['cy']

                x = (x_int - cx) * z / fx
                y = (y_int - cy) * z / fy

                self.get_logger().info(f"Object detected in X: {x:.2f} m, Y: {y:.2f} m, Z: {z:.2f} m")

                if len(boxes) == 0:
                    # Publish point to indicate that there are not detection
                    empty_point = Point()
                    empty_point.x = -1.0
                    empty_point.y = -1.0
                    empty_point.z = 0.0
                    self.pixeles_publisher.publish(empty_point)
                    self.get_logger().info("No detections. Sent empty point.")
                    return

                if len(boxes) > 5:
                    point_msg = Point()
                    point_msg.x = float(x)
                    point_msg.y = float(y)
                    point_msg.z = float(z)
                    pixel_msg = Point()
                    pixel_msg.x = float(x_pixel)
                    pixel_msg.y = float(y_pixel)
                    self.coordinates_publisher.publish(point_msg)
                    self.pixeles_publisher.publish(pixel_msg)
                    self.get_logger().info(f"Published coordinates: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")


        except Exception as e:
            self.get_logger().error(f'Error during inference: {e}')

    def postprocess_output(self, output, conf_thresh=0.3):
        # output shape: [1, 5, 8400]
        output = output[0]  # [5, 8400]
        pose_xywh = output[:4, :]  # x, y, w, h
        scores = output[4, :]       # confidence

        keep = scores > conf_thresh
        pose_xywh = pose_xywh[:, keep]
        scores = scores[keep]

        if pose_xywh.shape[1] == 0:
            return [], []


        x, y, w, h = pose_xywh #x, y are the positions (pixels) in image
        boxes = np.stack([x, y], axis=1)

        return boxes, scores



def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberTritonClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

