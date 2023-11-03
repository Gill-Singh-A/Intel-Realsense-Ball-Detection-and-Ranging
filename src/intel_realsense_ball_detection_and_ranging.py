#!/usr/bin/env python3

import rospy, cv2, numpy, math, image_geometry, statistics
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Polygon

bridge = CvBridge()
camera_model = image_geometry.PinholeCameraModel()

point_3d_publisher = rospy.Publisher("/ball_3d_point", Polygon, queue_size=10)

cameraInfo = None
image = None
depth_image = None

def getImage(ros_image):
    global image
    image = cv2.cvtColor(bridge.imgmsg_to_cv2(ros_image), cv2.COLOR_RGB2BGR)
def getDepthImage(ros_depth_image):
    global depth
    depth_image = bridge.imgmsg_to_cv2(ros_depth_image, ros_depth_image.encoding)
def getPointCloud(center, radius, index):
    if (center == None and radius == None) or depth == None:
        return
    ray = numpy.array(camera_model.projectPixelTo3dRay(center))
    point_3d = ray * depth
    point_3d_ros_msg = Point32()
    point_3d_ros_msg.x = point_3d[0]
    point_3d_ros_msg.y = point_3d[1]
    point_3d_ros_msg.z = point_3d[2]
    print(index, point_3d)
    return point_3d_ros_msg
def getCameraInfo(camera_info):
    global cameraInfo
    cameraInfo = camera_info

def dist(c_1, c_2):
    return math.sqrt((c_1[0]-c_2[0])**2+(c_1[1]-c_2[1])**2)
def getCircleDepth(ball):
    global depth, depths
    depths = []
    center, radius = (ball[0], ball[1]), ball[2]
    for x in range(center[0]-radius, center[0]+radius-1):
        for y in range(center[1]-round(math.sqrt(radius**2-(center[0]-x)**2)), center[1]+round(math.sqrt(radius**2-(center[0]-x)**2))-1):
            depths.append(depth_image[y][x])
    total_depth = sum(depths)
    total_depths = len(depths)
    depth = total_depth/total_depths

def get_masked_image():
    if image == None:
        return
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellowLower = (30, 50, 50)
    yellowUpper = (60, 255, 255)
    mask = cv2.inRange(hsv, yellowLower, yellowUpper)
    return mask
def get_contour_center(contour):
    M = cv2.moments(contour)
    cx = -1
    cy = -1
    if (M['m00'] != 0):
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    return cx, cy
def draw_ball_contours(rgb_img, contours):
    black_img = numpy.zeros(rgb_img.shape, 'uint8')
    ros_publisher_points = Polygon()
    for index, c in enumerate(contours):
        area = cv2.contourArea(c)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        if (area > 5000):
            cv2.drawContours(rgb_img, [c], -1, (255,0,255), 2)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_img, (cx,cy), (int)(radius), (0,255,255), 3)
            cv2.circle(black_img, (cx,cy), (int)(radius), (0,255,255), 3)
            cv2.circle(black_img, (cx,cy), 5, (150,0,255), -1)
            ros_publisher_points.points.append(getPointCloud((cx,cy),  (int)(radius), index))
    point_3d_publisher.publish(ros_publisher_points)

if __name__ == "__main__":
    rospy.init_node("intel_realsense_ball_detection_and_ranging")
    rate = rospy.Rate(10)
    image_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, getImage)
    image_depth_subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, getDepthImage)
    camera_info_subscriber = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, getCameraInfo)
    while cameraInfo == None:
        rate.sleep()
    camera_model.fromCameraInfo(cameraInfo)
    while not rospy.is_shutdown():
        mask = get_masked_image()
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        draw_ball_contours(image, contours)
        cv2.imshow("Ball Tracking", image)
        cv2.imshow("Mask", mask)