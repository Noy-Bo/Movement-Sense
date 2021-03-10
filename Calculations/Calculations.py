import numpy as np

from Calculations.Algebra import getVectorFrom2Points, getAngle, isZero


# Calculate list of angles
def CalculateAngles(skeletons, angleType, timestamps=None):
    angles = []
    correspondingTimestamps = []
    AngleCalculation = None
    pointA = None
    pointB = None
    pointC = None

    if angleType is 'Knees':
        AngleCalculation = CalculateAngle3D
        pointA = 'rHip'
        pointB = 'rKnee'
        pointC = 'rAnkle'
        if timestamps is None:
            pointA = 'RPSI'
            pointB = 'RKNE'
            pointC = 'RANK'

    elif angleType is 'Kyphosis':
        AngleCalculation = CalculateAngle2D
        pointA = 'rShoulder'
        pointB = 'neck'
        pointC = 'lShoulder'
        if timestamps is None:
            pointA = 'RSHO'
            pointB = 'LSHO'
            pointC = 'CLAV'

    for i in range(len(skeletons)):
        angle = AngleCalculation(getattr(skeletons[i], pointA), getattr(skeletons[i], pointB),
                                 getattr(skeletons[i], pointC))
        if angle != -1 or angle != -2:
            angles.append(angle)
            if timestamps is not None:
                correspondingTimestamps.append(timestamps[i])

    if timestamps is None:
        return angles
    else:
        return angles, correspondingTimestamps


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
