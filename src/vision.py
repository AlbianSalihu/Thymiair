"""
vision.py — HSV-based computer vision pipeline for detecting obstacles, goal, and Thymio.

Functions:
    image_filter       — applies HSV saturation mask + blur/morphology to produce a binary image
    obstacle_detection — detects rectangular obstacles (4-corner polygons) in a snapshot
    goal_detection     — detects the triangular goal marker and returns its centroid
    find_thymio        — detects Thymio via two colored dots and computes its heading angle
"""

import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

DEBUG = False  # set True to pop visualisation windows during development


# -------- Value definitions ------------------------------------------------

# Values for image filter on HSV images
hueLow   = 0
hueHigh  = 180
satLow   = 100
satHigh  = 255
valLow   = 0
valHigh  = 255
blurInt  = 15
morphIt  = 2

# Object criteria
obst_size_min    = 10000
obst_size_max    = 50000
obst_corner_number = 4

goal_size_min    = 10000
goal_size_max    = 70000
goal_corner_number = 3

thym_smallDot_size_min = 800
thym_smallDot_size_max = 5000
thym_bigDot_size_min   = 5000
thym_bigDot_size_max   = 18000
dot_corner_number_min  = 5
thym_dotsMaxDist       = 200

polygon_devi_max = 0.04
# ---------------------------------------------------------------------------


def image_filter(snap_HSV):
    """Apply HSV saturation threshold + median blur + erode/dilate to produce a binary mask.

    :param snap_HSV: HSV-transformed camera frame (numpy array)
    :returns: binary morphed image ready for contour detection
    """
    treshhold_low  = np.array([hueLow,  satLow,  valLow])
    treshhold_high = np.array([hueHigh, satHigh, valHigh])

    mask   = cv2.inRange(snap_HSV, treshhold_low, treshhold_high)
    median = cv2.medianBlur(mask, blurInt)
    erode  = cv2.erode(median, None, iterations=morphIt)
    morph  = cv2.dilate(erode,  None, iterations=morphIt)

    return morph


def obstacle_detection(snap_HSV):
    """Detect rectangular obstacles (4-corner polygons within size bounds) in the HSV snapshot.

    :param snap_HSV: HSV-transformed camera snapshot
    :returns: list of polygon corner arrays, one per obstacle; empty list if none found
    """
    poly_obst = []
    morph_obst = image_filter(snap_HSV)

    if DEBUG:
        plt.figure()
        plt.imshow(morph_obst)

    contours, _ = cv2.findContours(morph_obst, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for count in contours:
        epsilon = polygon_devi_max * cv2.arcLength(count, True)
        polygon = cv2.approxPolyDP(count, epsilon, True)
        if obst_size_min < cv2.contourArea(polygon) < obst_size_max and len(polygon) == obst_corner_number:
            poly_obst.append(polygon)

    return poly_obst


def goal_detection(snap_HSV):
    """Detect the triangular goal marker and return its centroid [x, y].

    :param snap_HSV: HSV-transformed camera snapshot
    :returns: [x, y] centroid of the goal triangle, or None if not found
    """
    poly_goal  = []
    morph_goal = image_filter(snap_HSV)

    contours, _ = cv2.findContours(morph_goal, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for count in contours:
        epsilon = polygon_devi_max * cv2.arcLength(count, True)
        polygon = cv2.approxPolyDP(count, epsilon, True)
        if goal_size_min < cv2.contourArea(polygon) < goal_size_max and len(polygon) == goal_corner_number:
            poly_goal = polygon

    if len(poly_goal) != 0:
        mome_goal  = cv2.moments(poly_goal)
        momX_goal  = int(mome_goal["m10"] / mome_goal["m00"])
        momY_goal  = int(mome_goal["m01"] / mome_goal["m00"])
        cent_goal  = [momX_goal, momY_goal]
        return cent_goal
    else:
        print('no goal found')
        return None


def find_thymio(snap_HSV):
    """Detect Thymio via its two colored dots and compute heading angle w.r.t. the x-axis.

    :param snap_HSV: HSV-transformed camera frame
    :returns: (flag, angle_rad, cent_thym, cent_thymS)
              flag=1 if successfully detected, angle in [-pi, pi], centers as [x, y]
    """
    flag_thym  = 0
    cent_thym  = []
    cent_thymS = []
    morph_thym = image_filter(snap_HSV)

    contours, _ = cv2.findContours(morph_thym, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    poly_thym_l = []
    poly_thym_s = []

    for count in contours:
        epsilon = polygon_devi_max * cv2.arcLength(count, True)
        polygon = cv2.approxPolyDP(count, epsilon, True)

        if thym_smallDot_size_min < cv2.contourArea(polygon) < thym_smallDot_size_max and len(polygon) >= dot_corner_number_min:
            poly_thym_s = polygon
        elif thym_bigDot_size_min < cv2.contourArea(polygon) < thym_bigDot_size_max and len(polygon) >= dot_corner_number_min:
            poly_thym_l = polygon

    if len(poly_thym_l) == 0:
        print('No Thymio found')
        return flag_thym, 0, [0, 0], [0, 0]

    elif len(poly_thym_l) != 0 and len(poly_thym_s) == 0:
        mome_thym  = cv2.moments(poly_thym_l)
        momX_thym  = int(mome_thym["m10"] / mome_thym["m00"])
        momY_thym  = int(mome_thym["m01"] / mome_thym["m00"])
        cent_thym  = [momX_thym, momY_thym]
        print('Only big circle detected')
        return flag_thym, 0, [0, 0], [0, 0]

    else:
        mome_thym  = cv2.moments(poly_thym_l)
        momX_thym  = int(mome_thym["m10"] / mome_thym["m00"])
        momY_thym  = int(mome_thym["m01"] / mome_thym["m00"])
        cent_thym  = [momX_thym, momY_thym]

        mome_thymS = cv2.moments(poly_thym_s)
        momX_thymS = int(mome_thymS["m10"] / mome_thymS["m00"])
        momY_thymS = int(mome_thymS["m01"] / mome_thymS["m00"])
        cent_thymS = [momX_thymS, momY_thymS]

        if math.dist(cent_thym, cent_thymS) > thym_dotsMaxDist:
            print('Detecting strange things which are not (only) Thymio')
            return flag_thym, 0, [0, 0], [0, 0]
        else:
            vecL       = np.array(cent_thym)
            vecS       = np.array(cent_thymS)
            vect_thym  = vecS - vecL

            a          = vect_thym[0]
            b          = vect_thym[1]
            angle_rad  = np.arctan2(b, a) - np.arctan2(0, 1)

            flag_thym  = 1
            return flag_thym, angle_rad, cent_thym, cent_thymS
