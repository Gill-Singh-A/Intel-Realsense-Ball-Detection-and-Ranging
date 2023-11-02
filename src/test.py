#! /usr/bin/env python3

import cv2, numpy

def get_masked_image(rgb_img):
    hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
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
    for c in contours:
        area = cv2.contourArea(c)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        if (area > 5000):
            cv2.drawContours(rgb_img, [c], -1, (255,0,255), 2)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_img, (cx,cy), (int)(radius), (0,255,255), 3)
            cv2.circle(black_img, (cx,cy), (int)(radius), (0,255,255), 3)
            cv2.circle(black_img, (cx,cy), 5, (150,0,255), -1)

if __name__ == "__main__":
    video_capture = cv2.VideoCapture(0)
    while cv2.waitKey(1) != 113:
        ret, frame = video_capture.read()
        if not ret:
            print("Error in Reading the Frame from the Camera")
            continue
        mask = get_masked_image(frame)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        draw_ball_contours(frame, contours)
        cv2.imshow("Ball Tracking", frame)
        cv2.imshow("Mask", mask)