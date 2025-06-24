import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

def extract_frames_from_bag(bag_path, output_dir):

    image_topic="/camera/camera/color/image_raw"
    
    bridge = CvBridge()

  
    os.makedirs(output_dir, exist_ok=True)

    
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    
    topic_types = reader.get_all_topics_and_types()
    type_dict = {t.name: t.type for t in topic_types}

    

    count = 0
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == image_topic:
            msg_type = Image
            msg = deserialize_message(data, msg_type)
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(output_dir, f"frame_{count:05d}.png")
            cv2.imwrite(filename, cv_image)
            print(f"Frame {count} guardado en {filename}")
            count += 1

    print(f"Process running: {count} frames saved.")


bag_path1="/home/andre14/ros2_ws/chip_video5"
output_dir1="/home/andre14/dev_ws/src/welectron_deployments/welectron_deployments/recorded_frames"

extract_frames_from_bag(bag_path=bag_path1, output_dir=output_dir1)