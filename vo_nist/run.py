# Visual Odometry Node
#
# This file runs the entire visual odometry algorithm

import rclpy
import cv2
import numpy as np
import sys
import time

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from cv_bridge import CvBridge
from voLib import *

import tf_transformations
from tf2_ros import TransformBroadcaster

# Debug flags
stream_raw = False
stream_keypoints = False
stream_flann = False
skip_match = False
skip_estimate = False
print_perturb = False
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

        # Publishers
        self.publish_pose = self.create_publisher(PoseStamped, 'camera_pose_vo', 10)
        self.publish_tf_broadcaster = TransformBroadcaster(self)

        # Defining Data
        self.trajectory = np.zeros((3, 1))
        self.br = CvBridge()
        self.pointCloudFrame = None
        self.imageFrame = None
        self.kp_last = None
        self.des_last = None
        self.imageFrame_last = None
        self.pose = np.eye(4)
        self.k = np.array([[615.8260498046875, 0, 327.4524230957031],
                           [0, 616.07373046875, 242.58750915527344],
                           [0, 0, 1]], dtype=np.float32)

    def callback_rgb(self, image_msg):
        self.imageFrame = self.br.imgmsg_to_cv2(image_msg)

        # Calculate Pose
        try: # don't update if something fails (likely not enough feature points)
            self.calc_pose()
        except:
            pass

        # Publish Pose
        self.pub_pose()

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
        if save_pose:
            np.save("out", self.trajectory)

    def pub_pose(self):
        pose = self.pose
        q = tf_transformations.quaternion_from_matrix(pose)

        msg = PoseStamped()
        msg.pose.position.x = pose[0, 3]
        msg.pose.position.y = pose[2, 3]
        msg.pose.position.z = pose[1, 3]
        msg.header.frame_id = "map"
        msg.pose.orientation.x = -q[0]
        msg.pose.orientation.y = -q[2]
        msg.pose.orientation.z = -q[1]
        msg.pose.orientation.w = q[3]
        self.publish_pose.publish(msg)

        t = TransformStamped()
        t.transform.translation.x = pose[0, 3]
        t.transform.translation.y = pose[2, 3]
        t.transform.translation.z = pose[1, 3]
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "vehicle_frame"
        t.transform.rotation.x = -q[0]
        t.transform.rotation.y = -q[2]
        t.transform.rotation.z = -q[1]
        t.transform.rotation.w = q[3]
        self.publish_tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    odom_node = Odometry()

    try:
        rclpy.spin(odom_node)
    except:
        # Save Data
        # This is such a terrible way of doing this, but it works
        # TODO: maybe try something more sensible one day
        if save_pose:
            odom_node.save_data()
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
