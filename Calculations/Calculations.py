import numpy as np
from scipy import ndimage

from Calculations.Algebra import getVectorFrom2Points, getAngle, isZero, getDistance
from Entities.Point3D import Point3D


def CalculateMeasurement(skeletons, calculations, timestamps=None):

    # knee angle
    if calculations is "Knees":
        points = ('rHip', 'rKnee', 'rAnkle')
        if timestamps is None:
            points = ('RASI','RKNE','RANK')
        return CalculateAngles(skeletons, CalculateAngle3D, points, timestamps)

    # shoulder's kyphosis
    elif calculations is "Kyphosis":
        points = ('rShoulder','neck','lShoulder')
        if timestamps is None:
            points = ('RSHO','LSHO','CLAV')
        return CalculateAngles(skeletons, CalculateAngle2D, points, timestamps)

    # center mass
    elif calculations is "Mass":
        points = ('midHip','_')
        if timestamps is None:
            points = ('RPSI','LPSI')
    return CalculteCenterOfMass(skeletons,points,timestamps)

    return

def CalculteCenterOfMass(skeletons, points, timestamps = None):
    centerOfMassPoints = []
    correspondingTimestamps = []
    varianceDistance = []

    for i in range(len(skeletons)):

        if timestamps is None: # vicon
            if (isZero(getattr(skeletons[i], points[0])) is not None or isZero(getattr(skeletons[i], points[1])) is not None):
                point = AverageOfPoints([getattr(skeletons[i], points[0]),getattr(skeletons[i], points[1])])
                point.x /= 1000; point.y /= 1000; point.z /= 1000
                centerOfMassPoints.append(point)

        elif timestamps is not None: # openpose
            if (getattr(skeletons[i],points[0]) is not None):
                point = getattr(skeletons[i],points[0])
                centerOfMassPoints.append(point)
                correspondingTimestamps.append(timestamps[i])

    avgPoint = AverageOfPoints(centerOfMassPoints)

    for i in range(len(centerOfMassPoints)):
        dist = getDistance(centerOfMassPoints[0],centerOfMassPoints[i])
        dist *= 100 # from meter to cm
        if dist > 100:
            dist = 0; # outliers elimination
        varianceDistance.append(dist)


    if timestamps is None:
        return varianceDistance
    else:
        #apply gaus smoothing
        return ndimage.gaussian_filter1d(varianceDistance, 5).tolist(), correspondingTimestamps


def CalculateAngles(skeletons, CalcAngle, points,timestamps=None):
    angles = []
    correspondingTimestamps = []

    for i in range(len(skeletons)):
        angle = CalcAngle(getattr(skeletons[i], points[0]), getattr(skeletons[i], points[1]),
                                 getattr(skeletons[i], points[2]))
        angles.append(angle)
        if timestamps is not None:
            correspondingTimestamps.append(timestamps[i])

    if timestamps is None:
        return angles
    else:
        return angles, correspondingTimestamps





def AverageOfPoints(points):
    x=y=z=count=0
    for p in points:
        x+=p.x
        y+=p.y
        z+=p.z
        count+=1
    x /= count
    y /= count
    z /= count
    avgPoint =  Point3D(x,y,z)
    return avgPoint


# # Calculate list of angles
# def CalculateAngles(skeletons, angleType, timestamps=None):
#     angles = []
#     correspondingTimestamps = []
#     AngleCalculation = None
#     pointA = None
#     pointB = None
#     pointC = None
#
#     if angleType is 'Knees':
#         AngleCalculation = CalculateAngle3D
#         pointA = 'rHip'
#         pointB = 'rKnee'
#         pointC = 'rAnkle'
#         if timestamps is None:
#             pointA = 'RPSI'
#             pointB = 'RKNE'
#             pointC = 'RANK'
#
#     elif angleType is 'Kyphosis':
#         AngleCalculation = CalculateAngle2D
#         pointA = 'rShoulder'
#         pointB = 'neck'
#         pointC = 'lShoulder'
#         if timestamps is None:
#             pointA = 'RSHO'
#             pointB = 'LSHO'
#             pointC = 'CLAV'
#
#     for i in range(len(skeletons)):
#         angle = AngleCalculation(getattr(skeletons[i], pointA), getattr(skeletons[i], pointB),
#                                  getattr(skeletons[i], pointC))
#         if angle != -1 or angle != -2:
#             angles.append(angle)
#             if timestamps is not None:
#                 correspondingTimestamps.append(timestamps[i])
#
#     if timestamps is None:
#         return angles
#     else:
#         return angles, correspondingTimestamps


# # Calculate list of angles - OpenPose
# def CalculateAnglesVicon(skeletons, type):
#     angles = []
#     AngleCalculation = None
#     pointA = None
#     pointB = None
#     pointC = None
#     if type is 'Knees':
#         # pointA = getattr(cls, var)
#         pointA = 'rHip'
#         pointB = 'rKnee'
#         pointC = 'rAnkle'
#         AngleCalculation = CalculateAngle3D
#     elif type is 'Kyphosis':
#         pointA = 'rShoulder'
#         pointB = 'neck'
#         pointC = 'lShoulder'
#         AngleCalculation = CalculateAngle2D
#     for i in range(len(skeletons)):
#         angle = AngleCalculation(getattr(skeletons(i), pointA), getattr(skeletons(i), pointB),
#                                  getattr(skeletons(i), pointC))
#         if angle != -1 or angle != -2:
#             angles.append(angle)
#     return angles


# Calculate angle of pointB. point_x is of Point3D class
def CalculateAngle3D(pointA, pointB, pointC):
    # TODO: throw exception- zero or None
    if pointA is None or pointB is None or pointC is None:
        return -2
    if isZero(pointA) == True or isZero(pointB) == True or isZero(pointC) == True:
        return -1
    vector_a = getVectorFrom2Points(np.array([pointA.x, pointA.y, pointA.z]),
                                    np.array([pointB.x, pointB.y, pointB.z]))
    vector_b = getVectorFrom2Points(np.array([pointB.x, pointB.y, pointB.z]),
                                    np.array([pointC.x, pointC.y, pointC.z]))
    # angle = 180 - angle
    angle = 180 - getAngle(vector_a, vector_b)
    return angle


# Calculate angle of pointB in 2D space, where y fixed as pointB.y
def CalculateAngle2D(pointA, pointB, pointC):
    pointA.y = pointC.y = pointB.y
    return CalculateAngle3D(pointA, pointB, pointC)
