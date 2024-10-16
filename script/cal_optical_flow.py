#!/usr/bin/env python

import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class OpticalFlowMagnitude:
    def __init__(self, bag_file, image_topic):
        self.bridge = CvBridge()
        self.bag_file = bag_file
        self.image_topic = image_topic
        self.previous_frame = None
        self.max_magnitude = 0  # Initialize the maximum optical flow magnitude
        self.image_diagonal = None  # Store the image diagonal in pixels

    def process_bag(self, frame_skip=10):
        with rosbag.Bag(self.bag_file, 'r') as bag:
            frame_count = 0
            for topic, msg, t in bag.read_messages(topics=[self.image_topic]):
                if frame_count % frame_skip == 0:
                    self.process_image(msg)
                frame_count += 1

    def process_image(self, img_msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # Get image dimensions and calculate diagonal in pixels
        if self.image_diagonal is None:
            height, width = gray_frame.shape
            self.image_diagonal = np.sqrt(width**2 + height**2)
            print(f"Image diagonal (in pixels): {self.image_diagonal}")

        if self.previous_frame is not None and self.previous_frame.shape == gray_frame.shape:
            # Calculate optical flow using the Farneback algorithm
            flow = cv2.calcOpticalFlowFarneback(self.previous_frame, gray_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)
            magnitude = self.compute_flow_magnitude(flow)

            # Normalize the optical flow magnitude
            normalized_magnitude = magnitude / self.image_diagonal

            # Update the maximum normalized magnitude
            self.max_magnitude = max(self.max_magnitude, np.max(normalized_magnitude))
            print(f"Current Optical Flow Magnitude (normalized):\n{normalized_magnitude}")
            print(f"Max Optical Flow Magnitude (normalized) so far: {self.max_magnitude}")

        # # Calculate optical flow using the Lucas-Kanade method
        # p0 = cv2.goodFeaturesToTrack(self.previous_frame, mask=None, maxCorners=100, qualityLevel=0.3, minDistance=7)
        # p1, st, err = cv2.calcOpticalFlowPyrLK(self.previous_frame, gray_frame, p0, None)
        # if p1 is not None:
        #     flow = p1 - p0
        #     magnitude = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
        #     normalized_magnitude = magnitude / self.image_diagonal
        #     self.max_magnitude = max(self.max_magnitude, np.max(normalized_magnitude))
        #     print(f"Max Optical Flow Magnitude (normalized) so far: {self.max_magnitude}")

        # Update the previous frame
        self.previous_frame = gray_frame

    def compute_flow_magnitude(self, flow):
        # Compute the magnitude of the optical flow vectors
        magnitude = np.sqrt(flow[..., 0]**2 + flow[..., 1]**2)
        return magnitude

def main():
    bag_file = "/home/chill/桌面/FWAV_DATASET/Outdoor/dataset/compare_dataset/Zurich_Urban_AGZ.bag"  # Path to your bag file
    image_topic = "camera/image"  # Name of your image topic

    optical_flow_processor = OpticalFlowMagnitude(bag_file, image_topic)
    optical_flow_processor.process_bag()

if __name__ == '__main__':
    main()
