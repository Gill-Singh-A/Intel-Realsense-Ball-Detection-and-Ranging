#!/usr/bin/env python3

import rospy, cv2, numpy, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

sample_space = 5
tolerence = 10
score_threshold = 0

images = []
center, radius = None, None

def getImage(ros_image):
    images.append(cv2.cvtColor(bridge.imgmsg_to_cv2(ros_image), cv2.COLOR_RGB2BGR))
def getDepthImage(ros_depth_image):
    depth_image = bridge.imgmsg_to_cv2(ros_depth_image, ros_depth_image.encoding)
    if center == None and radius == None:
        return
    depths = []
    for x in range(center[0]-radius, center[0]+radius):
        for y in range(center[1]-round(math.sqrt(radius**2-(center[0]-x)**2)), center[1]+round(math.sqrt(radius**2-(center[0]-x)**2))-1):
            depths.append(depth_image[y][x])
    total_depth = sum(depths)
    total_depths = len(depths)
    print(total_depth/total_depths)

def dist(c_1, c_2):
    return math.sqrt((c_1[0]-c_2[0])**2+(c_1[1]-c_2[1])**2)
def detectBalls(frames):
    frames_circles = []
    for frame in frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hist = cv2.equalizeHist(gray)
        blur = cv2.GaussianBlur(hist, (13, 13), cv2.BORDER_DEFAULT)
        height, width = blur.shape[:2]
        minR = round(width/40)
        maxR = round(width/20)
        minDis = round(width/7)
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, minDis, param1=14, param2=25, minRadius=minR, maxRadius=maxR)
        if circles is not None:
            circles = numpy.round(circles[0, :]).astype("int")
            frame_circles = []
            for (x, y, r) in circles:
                frame_circles.append((x, y, r))
            frames_circles.append(frame_circles)
    if len(frames_circles) > 1:
        reference_circles = frames_circles[1]
    else:
        return None
    prev_min_dist, score = None, {i:0 for i in range(len(reference_circles))}
    for circles in frames_circles[1:]:
        index = None
        for i, circle in enumerate(reference_circles):
            dist_circles = {(circle, compare_circle): dist(circle, compare_circle) for compare_circle in circles}
            min_dist = min(list(dist_circles.values()))
            if min_dist > tolerence:
                break
            if prev_min_dist == None or min_dist < prev_min_dist:
                prev_min_dist = min_dist
                index = i
        if index != None:
            score[index] += 1
    score = {j: score[j] for j in reversed(sorted(score, key=lambda i: score[i]))}
    if len(score) > 0:
        best_circle = list(score.keys())[0]
        if score[best_circle] > score_threshold:
            return reference_circles[best_circle]
        else:
            return None
    else:
        return None

if __name__ == "__main__":
    rospy.init_node("intel_realsense_ball_detection_and_ranging")
    rate = rospy.Rate(10)
    image_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, getImage)
    image_depth_subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, getDepthImage)
    while not rospy.is_shutdown():
        if len(images) == sample_space:
            ball = detectBalls(images)
            if ball != None:
                center, radius = (ball[0], ball[1]), ball[2]
                cv2.circle(images[0], center, radius, (0, 255, 0), 2)
                cv2.rectangle(images[0], (ball[0]-1, ball[1]-1), (ball[0]+1, ball[1]+1), (255, 0, 0), -1)
            else:
                center, radius = None, None
            cv2.imshow("Ball Detection", images[0])
            cv2.waitKey(1)
            images.clear()