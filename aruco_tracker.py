#!python2
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library

DESIRED_ARUCO_DICTIONARY = "DICT_5X5_50"
ARUCO_SIZE = 0.05

# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

def check_aruco_validity():
    if ARUCO_DICT.get(DESIRED_ARUCO_DICTIONARY, None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
        sys.exit(0)

def load_aruco_dictionary():
    print("[INFO] detecting '{}' markers...".format(DESIRED_ARUCO_DICTIONARY))
    this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[DESIRED_ARUCO_DICTIONARY])
    this_aruco_parameters = cv2.aruco.DetectorParameters_create()
    return this_aruco_dictionary, this_aruco_parameters