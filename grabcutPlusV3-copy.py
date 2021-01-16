#!/usr/bin/env python
'''
===============================================================================
!! Update code documentation (as of Jan16,2020: Incomplete and not well edited) !!
Interactive Image Segmentation using GrabCut algorithm.

README FIRST:
    Two windows will show up, one for input and one for output.

    At first, in input window, draw a rectangle around the object using the
right mouse button. Then press 'n' to segment the object (once or a few times)
For any finer touch-ups, you can press any of the keys below and draw lines on
the areas you want. Then again press 'n' to update the output.

Key '0' - To select areas of sure background
Key '1' - To select areas of sure foreground
Key '2' - To select areas of probable background
Key '3' - To select areas of probable foreground

Key 'n' - To update the segmentation
Key 'r' - To reset the setup
Key 's' - To save the results
===============================================================================
'''


# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv
# import random as rng
import sys
import math

####################### FUNCTION DEFINITIONS #######################
# Auto selects the canny upper and lower boundaries
def autoCanny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv.Canny(image, lower, upper)
    # return the edged image
    return edged

def slope(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    if x2 != x1:
        return ((y2-y1)/(x2-x1))
    else:
        return 'NA'

def drawLine(image, p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    m = slope(p1, p2)
    h, w = image.shape[:2]
    if m != 'NA':
        #  here we are essentially extending the line to x=0 and x=width
        #  and calculating the y associated with it
        px = 0            # starting point
        py = -(x1-0)*m+y1
        qx = w     # ending point
        qy = -(x2-w)*m+y2
    else:
        #  if slope is zero, draw a line with x=x1 and y=0 and y=height
        px, py = x1, 0
        qx, qy = x1, h
    cv.line(image, (int(px), int(py)), (int(qx), int(qy)), (70, 255, 0), 2)
    return image

def contactPoints(ellipseParams, horizon):
    """ Returns: cPoints, points, angle (soon)
    - cPoints: [(row1, col1), (row2, col2)] (i think) for drawing contact points
    - points: [(row1, col1),...,(row4, col4)] for drawing tangential lines
    - need to add functionality for rotated ellipses (major not always major)
    - fix the mainEllipse (name) repetition problem through code
    """
    cx = ellipseParams[0]
    cy = -1*ellipseParams[1]        # cy value from fitEllipse, opposite sign
    major = ellipseParams[2]
    minor = ellipseParams[3]
    angle = math.radians(360-ellipseParams[4])  # (360-angleFitEllipse),convert radians
    sHorizon = (-1*horizon)-cy      # shifted horizon: -1 * horizon value - modified cy

    cPoints = []                    # list holding cpoint values
    cos = math.cos(angle)
    sin = math.sin(angle)

    a = (minor**2)*(cos**2) + (major**2)*(sin**2)
    b = 2*(sHorizon)*cos*sin*((minor**2)-(major**2))
    c = (sHorizon**2)*((minor**2)*(sin**2) + (major**2)*(cos**2))-((major**2)*(minor**2))

    discriminant = (b**2)-(4*a*c)
    if discriminant > 0:
        x1 = ((-1*b + math.sqrt(discriminant)) / (2*a)) + cx
        x2 = ((-1*b - math.sqrt(discriminant)) / (2*a)) + cx
        if x1 > x2:  # save leftmost points first in list
            cPoints.append((round(x2), horizon))
            cPoints.append((round(x1), horizon))
        else:
            cPoints.append((round(x1), horizon))
            cPoints.append((round(x2), horizon))

        return cPoints
    else:
        print("No intersection with horizon")
        return -1

def EllipseTangentLines(ellipseParams, cPoints):
    cx = ellipseParams[0]
    cy = -1 * ellipseParams[1]     # opposite sign since ellipse is bottom left cartesian quadrant
    major = ellipseParams[2]
    minor = ellipseParams[3]
    angle = math.radians(360-ellipseParams[4]) # (360-angleFitEllipse) spins wrong way,convert radians

    tangPoints = []
    cos = math.cos(angle)
    sin = math.sin(angle)
    # Calculate the slope of the tangent to the un-rotated ellipse
    # in terms of the rotated points (x2, y2) (contact point on original)
    leftContactPoint = cPoints[0]
    rightContactPoint = cPoints[1]

    def slope(contactPoint):
        x2 = contactPoint[0]
        y2 = contactPoint[1]  * (-1)
        m1 = -1 * ( (((x2-cx)*cos)+((y2-cy)*sin)) / (((y2-cy)*cos)-((x2-cx)*sin)) ) * ((minor**2)/(major**2))
        m2 = (m1*cos + sin) / (cos - m1*sin)
        return m2
        # # check if infinite:
        # denominator = (((contactPoint[1]-cy)*cos)-((contactPoint[0]-cx)*sin))*(major**2)
        # if (denominator != 0):
        #     m1 = -1*( (((contactPoint[0]-cx)*cos)+((contactPoint[1]-cy)*sin))*(minor**2)/denominator )
        #     # check if m2 will be infinite
        #     denominator2 = cos - m1*sin
        #     if (denominator2 != 0):
        #         m2 = (m1*cos + sin)/denominator2
        #         return m2
        #         # DEAL WITH THE INFINITE CASES

    leftSlope = slope(leftContactPoint)
    tempYL = ((-1*leftContactPoint[1])+50)
    leftX = (1/leftSlope)*(tempYL-(-1*leftContactPoint[1])) + leftContactPoint[0]
    leftLinePoint = (round(leftX), round(-1*tempYL))
    tangPoints.append(leftLinePoint)

    rightSlope = slope(rightContactPoint)
    tempYR = ((-1*rightContactPoint[1])+50)
    rightX = (1/rightSlope)*(tempYR-(-1*rightContactPoint[1])) + rightContactPoint[0]
    rightLinePoint = (round(rightX), round(-1*tempYR))
    tangPoints.append(rightLinePoint)

    return tangPoints


def contactAngle(cPoints, tangPoints):
    lvector_1 = [(tangPoints[0][0]) - (cPoints[0][0]), (tangPoints[0][1]) - (cPoints[0][1])]
    avgLengthInwards = (cPoints[0][0]+cPoints[1][0])/4
    ltemp = cPoints[0][0] + avgLengthInwards
    lvector_2 = [(ltemp) - (cPoints[0][0]), (cPoints[0][1]) - (cPoints[0][1])]

    rvector_1 = [(tangPoints[1][0]) - (cPoints[1][0]), (tangPoints[1][1]) - (cPoints[1][1])]
    rtemp = cPoints[1][0] - avgLengthInwards
    rvector_2 = [(rtemp) - (cPoints[0][0]), (cPoints[0][1]) - (cPoints[0][1])]

    def angle(vector_1, vector_2):
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.degrees(np.arccos(dot_product))
        return angle

    cAngles = []
    langle = angle(lvector_1, lvector_2)
    rangle = 180-angle(rvector_1, rvector_2)
    avgAngle = (langle+rangle)/2
    cAngles.append(round(langle, 2))
    cAngles.append(round(rangle, 2))
    cAngles.append(round(avgAngle, 2))

    return cAngles

##################### END OF FUNCTION DEFINITIONS #####################

class App():
    BLUE = [255, 0, 0]        # rectangle color
    RED = [0, 0, 255]         # PR BG
    GREEN = [0, 255, 0]       # PR FG
    BLACK = [0, 0, 0]         # sure BG
    WHITE = [255, 255, 255]   # sure FG

    DRAW_BG = {'color': BLACK, 'val': 0}
    DRAW_FG = {'color': WHITE, 'val': 1}
    DRAW_PR_BG = {'color': RED, 'val': 2}
    DRAW_PR_FG = {'color': GREEN, 'val': 3}

    # setting up flags
    rect = (0, 0, 1, 1)
    drawing = False         # flag for drawing curves
    rectangle = False       # flag for drawing rect
    rect_over = False       # flag to check if rect drawn
    rect_or_mask = 100      # flag for selecting rect or mask mode
    value = DRAW_FG         # drawing initialized to FG
    thickness = 3           # brush thickness

    def onmouse(self, event, x, y, flags, param):
        # Draw Rectangle
        if event == cv.EVENT_RBUTTONDOWN:
            self.rectangle = True
            self.ix, self.iy = x, y

        elif event == cv.EVENT_MOUSEMOVE:
            if self.rectangle == True:
                self.img = self.img2.copy()
                cv.rectangle(self.img, (self.ix, self.iy), (x, y), self.BLUE, 3)
                self.rect = (min(self.ix, x), min(self.iy, y), abs(self.ix - x), abs(self.iy - y))
                self.rect_or_mask = 0

        elif event == cv.EVENT_RBUTTONUP:
            self.rectangle = False
            self.rect_over = True
            cv.rectangle(self.img, (self.ix, self.iy), (x, y), self.BLUE, 3)
            self.rect = (min(self.ix, x), min(self.iy, y), abs(self.ix - x), abs(self.iy - y))
            self.rect_or_mask = 0
            print(" Now press the key 'n' a few times until no further change \n")

        # draw touchup curves

        if event == cv.EVENT_LBUTTONDOWN:
            if self.rect_over == False:
                print("first draw rectangle \n")
            else:
                self.drawing = True
                cv.circle(self.img, (x, y), self.thickness, self.value['color'], -1)
                cv.circle(self.mask, (x, y), self.thickness, self.value['val'], -1)

        elif event == cv.EVENT_MOUSEMOVE:
            if self.drawing == True:
                cv.circle(self.img, (x, y), self.thickness, self.value['color'], -1)
                cv.circle(self.mask, (x, y), self.thickness, self.value['val'], -1)

        elif event == cv.EVENT_LBUTTONUP:
            if self.drawing == True:
                self.drawing = False
                cv.circle(self.img, (x, y), self.thickness, self.value['color'], -1)
                cv.circle(self.mask, (x, y), self.thickness, self.value['val'], -1)

    def run(self):
        # Loading images
        if len(sys.argv) == 2:
            filename = sys.argv[1]  # for drawing purposes
        else:
            # print("No input image given, so loading default image, images/test1.jpg \n")
            # print("Correct Usage: python grabcut.py <filename> \n")
            filename = 'images/test1.jpg'

        self.img = cv.imread(filename)
        self.imgToDrawOn = self.img.copy()
        self.unsegmented = self.img.copy()
        self.img2 = self.img.copy()                               # a copy of original image
        self.mask = np.zeros(self.img.shape[:2], dtype=np.uint8)  # mask initialized to PR_BG
        self.output = np.zeros(self.img.shape, np.uint8)          # output image to be shown

        # input and output windows
        cv.namedWindow('output', cv.WINDOW_GUI_NORMAL)
        cv.namedWindow('input', cv.WINDOW_GUI_NORMAL)
        cv.setMouseCallback('input', self.onmouse)
        # cv.moveWindow('input', self.img.shape[1]+10, 90)

        print(" Instructions: \n")
        print(" Draw a rectangle around the object using right mouse button \n")

        while(1):

            cv.imshow('output', self.output)
            cv.imshow('input', self.img)
            k = cv.waitKey(1)

            # key bindings
            if k == 27:         # esc to exit
                break
            elif k == ord('0'):  # BG drawing
                print(" mark background regions with left mouse button \n")
                self.value = self.DRAW_BG
            elif k == ord('1'):  # FG drawing
                print(" mark foreground regions with left mouse button \n")
                self.value = self.DRAW_FG
            elif k == ord('2'):  # PR_BG drawing
                self.value = self.DRAW_PR_BG
            elif k == ord('3'):  # PR_FG drawing
                self.value = self.DRAW_PR_FG
            elif k == ord('s'):  # save image
                bar = np.zeros((self.img.shape[0], 5, 3), np.uint8)
                res = np.hstack((self.img2, bar, self.img, bar, self.output))
                cv.imwrite('outputImages/grabcut_output.png', res)
                print(" Result saved as image \n")

                self.unsegmented = cv.cvtColor(self.unsegmented, cv.COLOR_BGR2GRAY)
                self.unsegmented = cv.blur(self.unsegmented, (4, 4), 0)
                self.unsegmented = cv.adaptiveThreshold(self.unsegmented, 255, cv.ADAPTIVE_THRESH_MEAN_C, \
                            cv.THRESH_BINARY, 11, -2)

                totalHeight, totalWidth = (self.unsegmented).shape

                edged = autoCanny(self.unsegmented)
                start_x=self.rect[0]
                start_y=self.rect[1]
                width=self.rect[2]
                height=self.rect[3]
                # remove 10% from the bottom between left and right margins (1/6 of width inwards on each side)
                #  could simplify the int(round(..)) into one fxn 

                # ##@@!!!!!! CUT THE CONTOURS ALONG THIS LINE CHECK POLYFITTESTING
                edged[(start_y+int(round(height*0.78))):(start_y+int(round(height*1.1))), start_x+int(round(width*0.1)):(start_x+int(round(width*0.88)))] = 0
                edged[ 0:start_y, 0:totalWidth] = 0
                edged[ 0:totalHeight, (start_x+width):totalWidth] = 0
                edged[ (start_y+height):totalHeight, 0:totalWidth] = 0
                edged[ 0:totalHeight, 0:start_x] = 0

                cv.imwrite('outputImages/preEdging.jpg', edged)
                contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

                mainEllipse = [((0, 0), (0, 0), 0)]  # may have redundant parentheses
                horizonLevel = int(round((self.rect[3]+self.rect[1])*1)*0.984)  # fix code repetition


                #  Code can run faster if this only gets outer contours farther from center
                minRect = [None]*len(contours)
                for i, c in enumerate(contours):
                    minRect[i] = cv.minAreaRect(c)
                    if c.shape[0] > 5:
                        ellipse = cv.fitEllipse(c)  # stores ellipse data
                        # Fit Result:
                        a_min = ellipse[1][0]  # minor axis
                        a_max = ellipse[1][1]  # major axis
                        # if largest axes ellipse, save
                        if (a_max*a_min > ((mainEllipse[0])[1][1])*((mainEllipse[0])[1][0])):
                            mainEllipse[0] = ellipse
                        else:
                            pass

                # Set up Ellipse parameters
                ellipseParams = [mainEllipse[0][0][0],
                                 mainEllipse[0][0][1],
                                 mainEllipse[0][1][0]/2,
                                 mainEllipse[0][1][1]/2,
                                 mainEllipse[0][2]]
                # Find Contact Points
                cPoints = contactPoints(ellipseParams, horizonLevel)
                if cPoints != -1:   # if discriminant was not negative
                    # Find Points to draw tangets
                    tangPoints = EllipseTangentLines(ellipseParams, cPoints)
                    # Find Contact Angles
                    cAngles = contactAngle(cPoints, tangPoints)


                # Drawing: Here its possible that some ellipses arent shown or too many
                # shown because its using 155 pixels but really it should be an
                # percentage of the image size1 (ratio of pixels in ellipse vs image)
                # Draw contours + rotated rects + ellipses
                for i, c in enumerate(contours):
                    color = (128, 128, 128)  # (rng.randint(0, 256), rng.randint(0, 256), rng.randint(0, 256))
                    if c.shape[0] > 255:
                        # contour
                        cv.drawContours(self.output, contours, i, color)
                        # ellipse
                        cv.ellipse(self.output, ellipse, color, 2)
                        mainEllipse.append(ellipse)
                        # rotated rectangle
                        box = cv.boxPoints(minRect[i])
                        box = np.intp(box)  # np.intp: Integer used for indexing (same as C ssize_t; normally either int32 or int64)

                        cv.drawContours(self.output, [box], 0, color)

                color = (70, 255, 0)
                # draw horizon and contact po  ints
                cv.line(self.imgToDrawOn, (0, self.rect[3]+self.rect[1]), (self.img.shape[1], self.rect[3]+self.rect[1]), color, 2)
                if cPoints != -1:
                    cv.circle(self.imgToDrawOn, cPoints[0], 4, color, 2)
                    cv.circle(self.imgToDrawOn, cPoints[1], 4, color, 2)
                    drawLine(self.imgToDrawOn, cPoints[0], tangPoints[0])
                    drawLine(self.imgToDrawOn, cPoints[1], tangPoints[1])
                    cv.putText(self.imgToDrawOn, str(cAngles[0]), (cPoints[0][0]+10,cPoints[0][1]-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv.putText(self.imgToDrawOn, str(cAngles[1]), (cPoints[1][0]-60,cPoints[1][1]-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv.putText(self.imgToDrawOn, 'Avg. CAngle: '+str(cAngles[2]), (50,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    print("[cx,cy,major,minor,rotation]: ", ellipseParams)
                    print("Horizon row from top:", horizonLevel)
                    print("Left Contact Angle:", cAngles[0])
                    print("Right Contact Angle:", cAngles[1])
                    print("Avg. Contact Angle:", cAngles[2], "\n")

                cv.imwrite('outputImages/edging.png', self.output)
                cv.imwrite('outputImages/contactAngle.png', self.imgToDrawOn)

            elif k == ord('r'):  # reset everything
                print("resetting \n")
                self.rect = (0, 0, 1, 1)
                self.drawing = False
                self.rectangle = False
                self.rect_or_mask = 100
                self.rect_over = False
                self.value = self.DRAW_FG
                self.img = self.img2.copy()
                self.imgToDrawOn = self.img2.copy()
                self.unsegmented = self.img2.copy()
                self.mask = np.zeros(self.img.shape[:2], dtype=np.uint8)  #mask initialized to PR_BG
                self.output = np.zeros(self.img.shape, np.uint8)           #output image to be shown
            elif k == ord('n'):  # segment the image
                self.imgToDrawOn = self.img2.copy()
                print(""" For finer touchups, mark foreground and background after pressing keys 0-3
                and again press 'n' \n""")
                print("Rect:", self.rect)
                try:
                    bgdmodel = np.zeros((1, 65), np.float64)
                    fgdmodel = np.zeros((1, 65), np.float64)
                    if (self.rect_or_mask == 0):         # grabcut with rect
                        cv.grabCut(self.img2, self.mask, self.rect, bgdmodel, fgdmodel, 1, cv.GC_INIT_WITH_RECT)
                        self.rect_or_mask = 1
                    elif (self.rect_or_mask == 1):       # grabcut with mask
                        cv.grabCut(self.img2, self.mask, self.rect, bgdmodel, fgdmodel, 1, cv.GC_INIT_WITH_MASK)
                except:
                    import traceback
                    traceback.print_exc()

            mask2 = np.where((self.mask == 1) + (self.mask == 3), 255, 0).astype('uint8')
            self.output = cv.bitwise_and(self.img2, self.img2, mask=mask2)

        print('Done')


if __name__ == '__main__':
    print(__doc__)
    App().run()
    cv.destroyAllWindows()
