from mpl_toolkits import mplot3d
import argparse
import json
import math
import time
import numpy as np
import matplotlib.pyplot as plt

import numpy as np
import pyrealsense2 as rs

import cv2
import csv
class Point3D(object):
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

     # object to string
    def __repr__(self):
        return "x: " + str(self.x) + ", y: " + str(self.y) + ", z: " + str(self.z)

########################################################################################

class AlphaSkeleton(object):
    def __init__(self, alpha):
        i = 0;
        self.nose = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lEye = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rEye = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lEar = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rEar = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lShoulder = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rShoulder = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lElbow = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rElbow = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lWrist = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rWrist = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lHip = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rHip = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lKnee = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rKnee = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lAnkle = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rAnkle = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.head = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.neck = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.hip = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lBigToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rBigToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lSmallToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rSmallToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lHeel = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rHeel = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3

###############################################################################################

def takeClosest(num,collection):
   return min(collection,key=lambda x:abs(x-num))


def closest(lst, K):
    lst = np.asarray(lst)
    idx = (np.abs(lst - K)).argmin()
    return lst[idx]


def getDistance(pointA, pointB):
    return math.sqrt((math.pow((pointA.x - pointB.x), 2) + math.pow((pointA.y - pointB.y), 2) + math.pow((pointA.z - pointB.z), 2)))

def getNorm(p):
    return math.sqrt((math.pow(p.x,2) + math.pow(p.y,2) + math.pow(p.z,2)))


def getNormalizeVector(pointA, pointB = Point3D(0,0,0)):

    p = Point3D((pointA.x - pointB.x), (pointA.y - pointB.y), (pointA.z - pointB.z))
    norm = getNorm(p)
    if norm != 0:
        if p.x != 0:
            p.x = p.x / norm
        else:
            pass
        if p.y != 0:
            p.y = p.y / norm
        else:
            pass
        if p.z != 0:
            p.z = p.z / norm
        else:
            pass
    return p



def getAngle(normalizedVectorA, normalizedVectorB):

    # dot product
    angle = math.acos((normalizedVectorA.x * normalizedVectorB.x) + (normalizedVectorA.y * normalizedVectorB.y) + (normalizedVectorA.z * normalizedVectorB.z))

    # transform to radians to degrees
    angle = angle * (180 / np.pi)
    if angle == 90:
        print("error")
    return angle

def isZero(p):
    if math.fabs(p.x) == 0 and math.fabs(p.y) == 0 and math.fabs(p.z == 0):
        return True
    else:
        return False

def outOfBoundries(x,y):
    if x>1279 or y> 719:
        return True
    else:
        return False

# testing~
p1 = Point3D(1,0,0)
p2 = Point3D(3,3,0)
p3 = Point3D(0,0,0)

print (getNormalizeVector(p1,p2))


# print (getNorm(p1))
# testAngle = getAngle(getNormalizeVector(p2,None,1),getNormalizeVector(p1,None,1))
# print(testAngle)

