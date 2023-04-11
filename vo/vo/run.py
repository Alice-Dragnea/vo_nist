# Visual Odometry Node
#
# This file runs the entire visual odometry algorithm

import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from voLib import *

# Debug flags
stream_raw = False
stream_keypoints = False
stream_flann = False
skip_match = False
skip_estimate = False
print_perturb = True
print_pose = False
save_pose = False

class Odometry(Node):
    def __init__(self):
        super().__init__('odom_root')

        # Subscribers
        self.subscription = self.create_subscription(
                Image,
                '/camera/color/image_raw',
                self.callback_rgb,
                10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.callback_depth,
            10)

        # Defining Data
        self.trajectory = np.zeros((3, 1))
        self.br = CvBridge()
        self.pointCloudFrame = None
        self.imageFrame = None
        self.kp_last = None
        self.des_last = None
        self.imageFrame_last = None
        self.pose = np.eye(4)
        self.k = np.array([[611.2769775390625, 0, 434.48028564453125],
                           [0, 609.7720336914062, 237.57313537597656],
                           [0, 0, 1]], dtype=np.float32)

    def callback_rgb(self, image_msg):
        self.imageFrame = self.br.imgmsg_to_cv2(image_msg)

        # Calculate Pose
        try: # don't update if something fails (likely not enough feature points)
            self.calc_pose()
        except:
            pass
            

    def callback_depth(self, pc_msg):
        self.pointCloudFrame = pc_msg

    def calc_pose(self):
        # Stream RGB Video
        if stream_raw:
            stream_rgb(self.imageFrame)

        # Detect Features
        kp, des = detect_features(self.imageFrame, 1000)

        # Stream Detected Features
        if stream_keypoints:
            stream_features(self.imageFrame, kp)
        # Do Not Continue If First Frame
        if not skip_match and self.kp_last is not None and self.des_last is not None:
            # Detect Matches
            matches = detect_matches(self.des_last, des)

            # Filter Matches
            matches = filter_matches(matches, 0.7)

            # Stream Matches
            if stream_flann:
                stream_matches(self.imageFrame_last, self.kp_last, self.imageFrame, kp, matches)

            # Estimate Motion
            if not skip_estimate and self.pointCloudFrame is not None:
                # Estimate Change in Pose
                pose_perturb = estimate_motion(matches, self.kp_last, kp, self.k, self.pointCloudFrame)

                # Update Current Position
                self.pose = self.pose @ np.linalg.inv(pose_perturb)

                # Build Trajectory
                coordinates = np.array([[self.pose[0, 3], self.pose[1, 3], self.pose[2, 3]]])
                self.trajectory = np.concatenate((self.trajectory, coordinates.T), axis=1)
                
                # Print Perturb
                if print_perturb:
                    print(pose_perturb)

        # Print Pose
        if print_pose:
            print(self.pose)

        # Save Frame Data for Next Frame
        self.kp_last = kp
        self.des_last = des
        self.imageFrame_last = self.imageFrame

    def save_data(self):
        np.save("out", self.trajectory)

def main(args=None):
    rclpy.init(args=args)
    odom_node = Odometry()

    try:
        rclpy.spin(odom_node)
    except:
        # Save Data
        # This is such a terrible way of doing this, but it works
        # TODO: maybe try something more sensible one day
        odom_node.save_data()
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
