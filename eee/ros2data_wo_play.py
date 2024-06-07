import rosbag
import cv2
from cv_bridge import CvBridge
import os

def extract_images_from_bag(bag_file, output_dir, image_topic):
    # Ensure the output directory exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize ROS bag and CvBridge
    bag = rosbag.Bag(bag_file, 'r')
    bridge = CvBridge()

    # Iterate through the messages in the specified image topic
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Construct the image file name
        timestamp = t.to_nsec()
        image_file = os.path.join(output_dir, f"{timestamp}.png")
        
        # Save the image using OpenCV
        cv2.imwrite(image_file, cv_image)

        print(f"Saved image {image_file}")

    # Close the ROS bag
    bag.close()

if __name__ == "__main__":
    bag_file = 'eee_01.bag'
    output_dir = 'result'
    image_topic = '/left/image_raw'

    extract_images_from_bag(bag_file, output_dir, image_topic)
