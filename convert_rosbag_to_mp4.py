import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def rosbag_to_mp4(bag_path, image_topic, output_file, fps=30):

    rclpy.init()
    bridge = CvBridge()


    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)


    topics = reader.get_all_topics_and_types()
    topic_types = {t.name: t.type for t in topics}

    if image_topic not in topic_types:
        print(f"The topic '{image_topic}' is not in rosbag.")
        return

    #reader.set_filter({'topics': [image_topic]})

    first_frame = True
    video_writer = None
    frame_count = 0

    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == image_topic:
                img_msg = deserialize_message(data, Image)
                cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

                if first_frame:
                    height, width, _ = cv_image.shape
                    video_writer = cv2.VideoWriter(output_file, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
                    first_frame = False

                video_writer.write(cv_image)
                frame_count += 1

        print(f" {frame_count} videos were copied to: {output_file}")

    finally:
        if video_writer:
            video_writer.release()
        rclpy.shutdown()


if __name__ == '__main__':
    bag_path = "/home/andre14/ros2_ws/chips_video4"  
    image_topic = "/camera/camera/color/image_raw" 
    output_file = 'recorded_video.mp4'

    rosbag_to_mp4(bag_path, image_topic, output_file)