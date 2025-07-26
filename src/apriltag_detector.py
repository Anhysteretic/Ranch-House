#importing packages
import apriltag
import argparse 
import cv2 as cv
import os

# constructing the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, 
    help="path to the input image containing AprilTag")
args = vars(ap.parse_args())

STANDARD_WIDTH = 600
STANDARD_HEIGHT = 600

# load the input image and convert it to grayscale
print("[INFO] loading image...")
image = cv.imread(args["image"])
print(f"[INFO] Resizing image to {STANDARD_WIDTH}x{STANDARD_HEIGHT}...")
image = cv.resize(image, (STANDARD_WIDTH, STANDARD_HEIGHT))
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

# define the AprilTag detector options and then detect the AprilTags
#in the input image
print("[INFO] detecting AprilTags...")
options = apriltag.DetectorOptions(
    families="tag25h9", # Or "tag36h11" if you're testing that family
    border=1,           # Standard border size for AprilTags
    nthreads=1,         # Use a single thread for simplicity initially
    quad_decimate=1.0,  # Process at full resolution (since your image is clean and large)
    quad_sigma=0.0,     # No Gaussian blur, as your image is clean
    decode_cp_bits=0,   # Use default or 0 for common tags
    debug=True          # Crucial for seeing the debug image
)
apriltag_lib_search_path = [os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build', 'lib')]
detector = apriltag.Detector(options, searchpath=apriltag_lib_search_path)
results, dimg = detector.detect(gray, return_image=True)
print("[INFO] {} total AprilTags detected".format(len(results)))

# loop over the detected AprilTags
for r in results:
    # compute the bounding box of the AprilTag and then draw the
    # bounding box on the image
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))
    
    #draw the bounding box on the image
    cv.line(image, ptA, ptB, (0, 255, 0), 2)
    cv.line(image, ptB, ptC, (0, 255, 0), 2)
    cv.line(image, ptC, ptD, (0, 255, 0), 2)
    cv.line(image, ptD, ptA, (0, 255, 0), 2)

    #draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv.circle(image, (cX, cY), 5, (0, 0, 255), -1)

    # draw the tag family on the image
    tag_family = r.tag_family.decode("utf-8")
    cv.putText(image, tag_family, (ptA[0], ptA[1] - 15),
        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    print("[INFO] tag family: {}".format(tag_family))

    #show the output image after AprilTag detection
cv.imshow("Image", image)
cv.waitKey(0)
