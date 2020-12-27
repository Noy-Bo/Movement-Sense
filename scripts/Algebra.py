from mpl_toolkits import mplot3d
import argparse
import json
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import json
import glob

import numpy as np
import pyrealsense2 as rs
from scipy.interpolate import make_interp_spline, BSpline
from scipy.ndimage.filters import gaussian_filter1d
from collections import OrderedDict

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

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return (self.x, self.y).__hash__()
        # return hash(self.x,self.y)

################################################
class OpenPoseSkeleton(object):
    def __init__(self,keypoints):
        i = 0
        self.nose = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.neck = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rShoulder = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rElbow = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rWrist = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lShoulder = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lElbow = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lWrist = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.midHip = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rHip = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rKnee = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rAnkle = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lHip = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lKnee = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lAnkle = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rEye = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lEye = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rEar = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lEar = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lBigToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lSmallToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lHeel = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rBigToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rSmallToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rHeel = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])


class OpenPoseObject(object):
    def __init__(self, image_id, keypoints):
        self.image_id = image_id + ".png"
        self.keypoints = keypoints

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.image_id == other.image_id

    def __hash__(self):
        return self.image_id.__hash__()

    # object to string
    def __repr__(self):
        return "image id: " + str(self.image_id)

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.image_id == other.image_id

    def __hash__(self):
        return (self.image_id).__hash__()


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
max_height_resulotion = 0
max_width_resulotion = 0

plotCount = 0


################################################################################################
def takeClosest(num, collection):
    return min(collection, key=lambda x: abs(x - num))

def takeClosestByZ(zValue, collection):         # returns closest of collection of Point3D by zValue
    return min(collection, key=lambda x: abs(x.z - zValue))

def eroding(x, y, height, width,isRotated):               # takes x,y coordinates of pixel, height,width of frame, returns set of Point3D points eroded from the original pixel (can change size of erosion)

    if isRotated == True: #important in case we rotat
        tmp = height
        height = width
        width = tmp

    size = 6
    kernel = np.ones((size, size), np.uint8)
    img = np.zeros([height, width, 3], dtype=np.uint8)
    img.fill(255)  # or img[:] = 255
    img[x, y] = 0
    erosion = cv2.erode(img, kernel, iterations=1)
    indices = np.where(erosion == [0])
    coordinates = zip(indices[0], indices[1])
    collection = []
    for item in coordinates:
        collection.append(Point3D(item[0], item[1], 0))
    set1 = set(collection)
    return set1

def rounded(value, collection):                 # take a value (z) and collection of points in space, returns point the average of group size half of original collection, closest to value
    half = len(collection) / 2
    collect = []
    for i in range(half):
        point = takeClosestByZ(value, collection)
        collection.remove(point)
        collect.append(point)
    x, y, z = 0, 0, 0
    for item in collect:
        x = x + item.x
        y = y + item.y
        z = z + item.z
    x = x / len(collect)
    y = y / len(collect)
    z = z / len(collect)
    return Point3D(x, y, z)

def roundGraph(x_collection,y_collection,graph,xlabel,ylabel,limit):      # outputs a graph with scattered points and smoothened line
    if len(x_collection) < 3:
         return
    # resets the graph - TODO: perhaps find a way to only update the latest point
    graph.clear()
    # casts the data to np.array type for the plot
    x = np.array(x_collection)
    y = np.array(y_collection)
    # if i remember correctly, splits the range between first and second argument to 'third argument' slices (used to be 100, dont need it for 1d line, but need np.linespace)
    x_smooth = np.linspace(x.min(),x.max(),200)
    # no idea
    spl = make_interp_spline(x, y, k=1)
    # no idea
    y_smooth = spl(x_smooth)
    # sets graph names again, because we did ax.clear at the beginning
    graph.set_ylabel(ylabel)
    graph.set_xlabel(xlabel)
    graph.set_ylim([0, limit])
    #graph.plot(x_smooth, y_smooth)
    # if we want to add the scattered points
    graph.scatter(x, y, c='b', label='data',s=2)





def getDistance(pointA, pointB):
    return math.sqrt(
        (math.pow((pointA.x - pointB.x), 2) + math.pow((pointA.y - pointB.y), 2) + math.pow((pointA.z - pointB.z), 2)))


def getNorm(p):
    return math.sqrt((math.pow(p.x, 2) + math.pow(p.y, 2) + math.pow(p.z, 2)))
def getNorm2D(p):
    return math.sqrt((math.pow(p.x, 2) + math.pow(p.y, 2)))

def getNormalizeVector(pointA, pointB=Point3D(0, 0, 0)):
    p = Point3D((pointA.x - pointB.x), (pointA.y - pointB.y), (pointA.z - pointB.z))
    norm = getNorm(p)
    # if norm != 0:
            # p.x = p.x / norm
            # p.y = p.y / norm
            # p.z = p.z / norm
    #print (p.z+p.y+p.x)
    return p
def getVectorFrom2Points(pointA,pointB):
    return np.subtract(pointA,pointB)

def getAngle(pointA,pointB):
    dotProduct = np.dot(pointA, pointB)
    norms = np.linalg.norm(pointA) * np.linalg.norm(pointB)
    angleRadians = np.arccos(np.divide(dotProduct, norms))
    angleDegrees = angleRadians * (180 / np.pi)
    return angleDegrees


def getNormalizeVector2D(pointA, pointB=Point3D(0, 0, 0)):
    p = Point3D((pointA.x - pointB.x), (pointA.y - pointB.y), (pointA.z - pointB.z))
    norm = getNorm2D(p)
    if norm != 0:
        if p.x != 0:
            p.x = p.x / norm
        else:
            pass
        if p.y != 0:
            p.y = p.y / norm
        else:
            pass
    return p

def getAngle2D(normalizedVectorA, normalizedVectorB):
    # dot product
    angle = -1
    res = (normalizedVectorA.x * normalizedVectorB.x) + (normalizedVectorA.y * normalizedVectorB.y)
    if res >= -1 and res <= 1:
        angle = math.acos((normalizedVectorA.x * normalizedVectorB.x) + (normalizedVectorA.y * normalizedVectorB.y))
    else:
        pass
    # transform to radians to degrees
    if (angle != -1):
        angle = angle * (180 / np.pi)
    if angle == 90:
        print("error")
    return angle

# def getAngle(normalizedVectorA, normalizedVectorB):
#     # dot product
#     angle = -1
#     res = (normalizedVectorA.x * normalizedVectorB.x) + (normalizedVectorA.y * normalizedVectorB.y) + (
#             normalizedVectorA.z * normalizedVectorB.z)
#     norms = getNorm(normalizedVectorA) * getNorm((normalizedVectorB))
#     res /= norms
#     if res >= -1 and res <= 1:
#         angle = math.acos((normalizedVectorA.x * normalizedVectorB.x) + (normalizedVectorA.y * normalizedVectorB.y) + (
#                 normalizedVectorA.z * normalizedVectorB.z))
#     else:
#         pass
#
#     # transform to radians to degrees
#     if (angle != -1):
#         angle = angle * (180 / np.pi)
#     if angle == 90:
#         print("error")
#     return angle


def isZero(p):
    if math.fabs(p.x) == 0 and math.fabs(p.y) == 0 and math.fabs(p.z == 0):
        return True
    else:
        return False


def outOfBoundries(x, y,isRotated):
    if isRotated is False:
        if x > (max_width_resulotion - 1) or y > (max_height_resulotion - 1):
            return True
        else:
            return False
    else:
        if y > (max_width_resulotion - 1) or x > (max_height_resulotion - 1):
            return True
        else:
            return False

def getRotatedPixel(x,y,center_x,center_y,angle):
    xp = (x - center_x) * np.cos(angle) - (y - center_y) * np.sin(angle) + center_x
    yp = (x - center_x) * np.sin(angle) + (y - center_y) * np.cos(angle) + center_y

    return xp,yp

def OpenPoseReader():
    # final objects list
    openpose_list = []
    # original path to files
    raw_filenames = glob.glob('*openpose/*.json')
    # constructing filenames
    for item in raw_filenames:
        # open json file
        with open(item) as json_file:
            data = json.load(json_file)
        name_temp = item.replace('openpose\\','')
        # get image id name
        image_id = name_temp.replace('_keypoints.json','')
        # get keypoints
        keypoints = data['people'][0]['pose_keypoints_2d']
        openpose_list.append(OpenPoseObject(image_id,keypoints))
    return openpose_list