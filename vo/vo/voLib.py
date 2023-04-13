# Helper functions for run.py

import cv2
import numpy as np
import array

from sensor_msgs_py import point_cloud2


def rotate_x(rmat, tvec):
    rx = np.array([[1, 0, 0],
                    [0, np.cos(np.deg2rad(rmat[1, 1])), -1*np.sin(np.deg2rad(rmat[1, 2]))],
                    [0, np.sin(np.deg2rad(rmat[2, 1])), np.cos(np.deg2rad([2, 2]))]])
    rmat = rx @ rmat
    tvec = rx @ tvec

    return rmat, tvec

def estimate_motion(matches, kp_last, kp, k, pointCloud):
    # Declare lists and arrays
    rmat = np.eye(3)
    tvec = np.zeros((3, 1))
    pxl_list_last = []
    pxl_list = []
    points = []
    valid_points = []
    valid_pxl_list = []

    # Collect feature pixel locations
    for match in matches:
        x1, y1 = kp_last[match[0].queryIdx].pt
        x2, y2 = kp[match[0].trainIdx].pt
        pxl_list_last.append([int(x1), int(y1)])
        pxl_list.append([int(x2), int(y2)])

    # Read point cloud data
    points = read_point_efficient(pointCloud, pxl_list_last)

    # Remove pixel locations if there is no corresponding point
    for point in range(len(points)):
        if np.linalg.norm(points[point]) > 0:
            valid_points.append(points[point])
            valid_pxl_list.append(pxl_list[point])

    # Solve for relative motion
    if len(valid_points) > 5:

        # Ransac and solve
        _, rvec, tvec, _ = cv2.solvePnPRansac(np.vstack(valid_points), np.array(valid_pxl_list).astype(np.float32), k, None)

        # Convert to Rodrigues
        rmat, _ = cv2.Rodrigues(rvec)

        # Correct rotation
        rmat, tvec = rotate_x(rmat, tvec)

    # Pack
    pose_perturb = np.eye(4)
    pose_perturb[0:3, 0:3] = rmat
    pose_perturb[0:3, 3] = tvec.T

    return pose_perturb

def read_point_efficient(pointCloud, pxl_list):
    # Define points to use
    select_data = array.array('B', [])
    for x, y in pxl_list:
        start_idx = x * pointCloud.point_step + y * pointCloud.row_step
        select_data += pointCloud.data[start_idx:start_idx + pointCloud.point_step]
    points_len = len(pxl_list)

    points = np.ndarray(
        shape=(points_len, ),
        dtype=point_cloud2.dtype_from_fields(pointCloud.fields, point_step=pointCloud.point_step),
        buffer=select_data)
    points_out = np.vstack([points['x'], points['y'], points['z']]).T
    return points_out

def filter_matches(matches, threshold):
    filtered = []
    for match in matches:
        if len(match) == 2 and match[0].distance < (threshold * match[1].distance):
            filtered.append(match)

    return filtered

def stream_matches(imageFrame_last, kp_last, imageFrame, kp, matches):
    # Define draw parameters
    draw_params = dict(matchColor=(255, 0, 0),
                       singlePointColor=(0, 255, 0),
                       flags=cv2.DrawMatchesFlags_DEFAULT)

    # Create Images
    visual = cv2.drawMatchesKnn(imageFrame, kp, imageFrame_last, kp_last, matches, None, **draw_params)

    # Draw Images
    cv2.imshow("Matches", visual)
    cv2.waitKey(1)

def detect_matches(des_last, des):
    # Define FLANN parameters
    index_params = dict(algorithm=6,
                        table_number=6,
                        key_size=12,
                        multi_probe_level=1)
    search_params = dict(checks=50)

    # Initialize FLANN Matcher
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    # Detect Matches
    match = flann.knnMatch(des_last, des, k=2)

    return match

def stream_rgb(image):
    cv2.imshow("RGB Stream", image)
    cv2.waitKey(1)

def detect_features(image, num):
    # Initialize ORB Dectector
    orb = cv2.ORB_create(nfeatures=num)

    # Detect and Compute
    kp, des = orb.detectAndCompute(image, None)

    return kp, des

def stream_features(image, kp):
    # Create Image
    visual = cv2.drawKeypoints(image, kp, None, color=(0, 255, 0), flags=0)

    # Draw Images
    cv2.imshow("Features", visual)
    cv2.waitKey(1)
