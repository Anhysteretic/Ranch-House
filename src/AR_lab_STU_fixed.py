import cv2 as cv
import numpy as np
from std_msgs.msg import Int32MultiArray
import csv
import os


def find_relative_pose(pic):
    ids = Int32MultiArray()  # Each AR tag has a number written on it and a unique color combo, this will contain info for each 
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
    
    # Replace these with your actual calibration results
    mtx = np.array([[800, 0, 320],    # fx=800, cx=320
                [0, 800, 240],    # fy=800, cy=240
                [0,   0,   1]])   # typical for 640x480 camera

    distortion = np.array([0.1, -0.05, 0.0, 0.0, 0.0])  # k1, k2, p1, p2, k3

    # Find and process the image
    tag_info = grab_tag(pic, arucoDict, mtx, distortion)
    # grab_tag should accept the image, dictionary, matrix, and distortion

    if tag_info and len(tag_info) == 2:  # if AR tag is found
        rvec, tvec = tag_info
        trans, orien = find_pos(rvec, tvec)
        print(f"Orientation: {orien}, Position: {trans}")
    else:
        print('not found')
        
def grab_tag(image, arucoDict, mtx, distortion):
    """This function extracts the corners of the AR tag from the image"""

    # Preprocess the image
    if image is None:
        print("Error: Image is None.")
        return []
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)  # load in grayscale

    # Find corners of the AR tag
    corners, ids, rejects = cv.aruco.detectMarkers(gray, arucoDict)

    if corners is None or len(corners) == 0:
        print('No corners found')
        return []

    # Estimate pose of the first detected marker
    rvec, tvec = my_estimatePoseSingleMarkers2(corners, mtx, distortion)

    # Optional: draw axes for visualization
    cv.drawFrameAxes(image, mtx, distortion, rvec, tvec, length=10)

    print(f"rvecs: {rvec}, tvecs: {tvec}\n")
    return rvec, tvec

def find_pos(rvec, tvec):
        """rvec is a rodriguez vector and tvec is the position of the tag relative to
        the camera in camera frame. Use rvec to create a rotation matrix and find
        position of the drone relative to the tag; return the position of the drone. 
        Additionally, it may be helpful to have the orientation of the drone so you can could
        also return euler angles, the full rotation matrix, or some other information. This is just
        some math so I'll let you do the whole thing yourselves"""
        return drone_from_ar, orientation

def my_estimatePoseSingleMarkers2(corners, mtx, distortion, marker_size=26.6): # 26.6cm is side length of AR tag
    """This function gets the rvec and tvec """

    c = np.array(corners[0])[0]  # corners of the AR tag in a numpy array in camera frame

    marker_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],   # top-left
        [marker_size / 2, marker_size / 2, 0],    # top-right
        [marker_size / 2, -marker_size / 2, 0],   # bottom-right
        [-marker_size / 2, -marker_size / 2, 0]   # bottom-left
    ], dtype=np.float32)

    # SolvePnP expects object points and image points
    success, rvec, tvec = cv.solvePnP(marker_points, c, mtx, distortion)
    return rvec, tvec

#Read image into CV2
path = os.path.expanduser('ADD PATHNAME')
image = cv.imread(path, cv.IMREAD_COLOR)
if image is None:
    print("Failed to load image. Check the path and file.")

find_relative_pose(image)#compute world frame position and orientation

#optional debugging
cv.imshow("Pose Debug", image)
cv.waitKey(0)
cv.destroyAllWindows()