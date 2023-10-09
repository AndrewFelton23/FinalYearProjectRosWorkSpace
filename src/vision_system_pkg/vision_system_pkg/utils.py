import cv2
import numpy as np
from math import atan2, cos, sin, degrees, radians

def stackImages(scale,imgArray):
    '''A method used to stack many images into a single frame'''
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def empty(a):
    pass

def getContours(img,imgContour,minArea,maxArea, imgOrientation):
    '''A method used to get the contours of the image'''
    contours, hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    center = None
    angle_deg = None
    x = 0
    y = 0
    w = 0
    h = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if minArea < area < maxArea:
            cv2.drawContours(imgContour,cnt, -1, (255, 0, 255), 7)
            peri = cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,0.02*peri,True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x, y),(x+w,y+h), (0, 255, 0), 5)

            cv2.putText(imgContour, "Points: " + str(len(approx)),(x+w+20, y+20),cv2.FONT_HERSHEY_COMPLEX,.7,(0,255,0),2)

            cv2.putText(imgContour, "Area: " + str(int(area)),(x+w+20, y+45),cv2.FONT_HERSHEY_COMPLEX,.7,(0,255,0),2)
            if 6>=len(approx) >= 4:  # Check if the contour is a quadrilateral (square)
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)

                center = (x + w // 2, y + h // 2)
                
                # Calculate the rotation angle
                side1 = approx[1][0] - approx[0][0]
                side2 = approx[2][0] - approx[1][0]
                if side1[1] > side2[1]:
                    long_side = side1
                    short_side = side2
                else:
                    long_side = side2
                    short_side = side1
                
                angle_rad = atan2(long_side[1], long_side[0])
                angle_deg = degrees(angle_rad)
                if angle_deg < 0:
                    angle_deg += 180  # Adjust the angle range if needed
                cv2.putText(imgOrientation, "Points: " + str(len(approx)), (x + w + 20, y + 20), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
                cv2.putText(imgOrientation, "Area: " + str(int(area)), (x + w + 20, y + 45), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
                cv2.putText(imgOrientation, "Angle: " + str(round(angle_deg, 2)), (x + w + 20, y + 70), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
                cv2.putText(imgOrientation, "Center: " + str(center), (x + w + 20, y + 95), cv2.FONT_HERSHEY_COMPLEX, .7, (0, 255, 0), 2)
                draw_axes(imgOrientation, center, angle_deg, max(w, h))
    return center, angle_deg, (x + w // 2), ( y + h // 2)

##UNDER DEVELOPMENT
def getHoughTranform(image, edges, rho = 1,theta= np.pi / 180, thresh = 100):
    lines = cv2.HoughLines(edges, rho, theta, threshold=thresh)
    for line in lines:
        rho0, theta0 = line[0]
        angle = theta0 * 180 / np.pi

        # Filter lines based on angle
        if 45 <= angle <= 135 or 225 <= angle <= 315:
            a = np.cos(theta0)
            b = np.sin(theta0)
            x0 = a * rho0
            y0 = b * rho0

            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))

            # Draw lines on the image
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
    print(angle)

def draw_axes(img, center, angle_deg, size):
    angle_rad = radians(angle_deg)
    cos_val = cos(angle_rad)
    sin_val = sin(angle_rad)
    cos_val_perp = cos(angle_rad+np.pi/2)
    sin_val_perp = sin(angle_rad+np.pi/2)
    
    length = size // 2
    
    x1 = int(center[0] - length * cos_val)
    y1 = int(center[1] - length * sin_val)
    x2 = int(center[0] + length * cos_val)
    y2 = int(center[1] + length * sin_val)

    x3 = int(center[0] - length * cos_val_perp)
    y3 = int(center[1] - length * sin_val_perp)
    x4 = int(center[0] + length * cos_val_perp)
    y4 = int(center[1] + length * sin_val_perp)
    
    cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 5)
    cv2.line(img, (x3, y3), (x4, y4), (0, 255, 0), 5)