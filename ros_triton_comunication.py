import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tritonclient.utils import InferenceServerException
import tritonclient.http as httpclient

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
        self.bridge = CvBridge()

        # Triton client config
        self.triton_url = '192.168.0.180:8000'  
        self.model_name = 'raspberrypi_detection'  # Change model name!
        self.model_input_name = 'images'      # Change input name!
        self.model_output_name = 'output0'    # Change output name!
        self.client = httpclient.InferenceServerClient(url=self.triton_url)

    def preprocess(self, cv_image):
        resized = cv2.resize(cv_image, (640, 640))
        normalized = resized.astype(np.float32)
        normalized = normalized / 255.0  
        img = np.transpose(normalized, (2, 0, 1)) 
        input_data = np.expand_dims(img, axis=0)  
        return input_data
    
    def listener_callback(self, msg: Image):
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
            for i, box in enumerate(boxes):
                self.get_logger().info(f"Bounding Box Coordinates {i}: {box} | Confidence: {scores[i]:.2f}")
        except Exception as e:
            self.get_logger().error(f'Error during inference: {e}')

    def postprocess_output(self, output, conf_thresh=0.3):
        # output shape: [1, 5, 8400]
        output = output[0]  # [5, 8400]
        boxes_xywh = output[:4, :]  # x, y, w, h
        scores = output[4, :]       # confidence

        keep = scores > conf_thresh
        boxes_xywh = boxes_xywh[:, keep]
        scores = scores[keep]

        if boxes_xywh.shape[1] == 0:
            return [], []

        # Convert from (x,y,w,h) to (x1,y1,x2,y2)
        x, y, w, h = boxes_xywh
        x1 = x - w / 2
        y1 = y - h / 2
        x2 = x + w / 2
        y2 = y + h / 2
        boxes = np.stack([x1, y1, x2, y2], axis=1)

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

