## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

############################################################################################
## this script uses bag file, the log, skeleton-key points, to extract and calculate data ##
############################################################################################

import matplotlib
from mpl_toolkits import mplot3d
import argparse
import json
import math
import time
import numpy as np
from scipy import ndimage, misc
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
#import xlsxwriter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

#from Excel import addToExcel
import sys

sys.path.append('C:\Python27\Lib\site-packages')

import numpy as np
import pyrealsense2 as rs
import Algebra
import cv2


# class for timestamps
class Timestamp(object):
    def __init__(self, color_timestamp, depth_timestamp):
        self.color_timestamp = color_timestamp
        self.depth_timestamp = depth_timestamp


# class for alphapose cloud
class AlphaPoseObject(object):
    def __init__(self, image_id, score, box, keypoints):
        self.image_id = image_id
        self.score = score
        self.box = box
        self.keypoints = keypoints

    def __hash__(self):
        return hash(self.image_id)

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.image_id == other.image_id

    # object to string
    def __repr__(self):
        return self.image_id + " " + str(self.score)

    # function to add to global table
    def add(self):
        global skeletonsTable  # uses the global table declared in main, no idea why we need this
        if self in skeletonsTable:
            index = skeletonsTable.index(self)
            if self.score > skeletonsTable[index].score:
                skeletonsTable.pop(index)
                skeletonsTable.append(self)
        else:
            skeletonsTable.append(self)

# =========== ALPHAPOSE
# open's the alphapose results and sets them in json array
# skeletonsTable = []  # contains all json's, with the bigger score
# alphapose_file = open('alphapose-results.json')
# alphapose_array = json.load(alphapose_file)
#
# for item in alphapose_array:
#     x = AlphaPoseObject(item['image_id'], item['score'], item['box'], item['keypoints'])
#     x.add()

# =========== OPENPOSE
skeletonsTable = Algebra.OpenPoseReader()

json_log_name = "log_color.json"  # choose color or depth
excelFilename = "Hanna_Squat_Front_depth.xlsx"  # change
heading = "Hanna_Squat_Front_depth"  # change    &     line 101
headingX = "Time (sec)"
headingY = "Distance butt-to-floor (meters)"
headingY2 = "Knees angle (degrees)"
dataX = []
dataX2 = []
dataY = []
dataY2 = []
startingTimeStamp = 0
currentTimeStamp = 0
lastTimeStamp = -1

# open's the log as json array.
log_file = open(json_log_name)
log_array = json.load(log_file)
log_list = []

# generating the log_list
for item in log_array:
    log_list.append(
        Timestamp(item['color_timestamp'], item['depth_timestamp']))  # json_list contains all pairs of depth & color

# Setup:
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("C:\Age_Estimation_Project\\bag_files\sub02_squat_side.bag", False)

profile = pipeline.start(cfg)
device = profile.get_device()
playback = device.as_playback()  # this allows the use of pause and resume.

# pasuing playback to let the pipe warm up
playback.pause

first = True
firstFloor = True
isRotated =False

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 2.2  # 1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)
numOfFrames = 0
# field names
all_depth_timestamps = []

color_depth = []
timestamps = []

# Streaming loop
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
parser.add_argument("-i", "--input", type=str, help="Bag file to read")
args = parser.parse_args()

# setup plot - two graphs and two images
fig = plt.figure(num=None, figsize=(60, 10), dpi=80)

ax = fig.add_subplot(221)
# ax.set_ylabel("Butt to floor distance (Meter)")
# ax.set_xlabel("Time (Sec)")

bx = fig.add_subplot(223)
# bx.set_ylabel("Angle")
# bx.set_xlabel("Time (Sec)")

ay = fig.add_subplot(222)
ay.set_title("Distance")
ay.axis('off')

by = fig.add_subplot(224)
by.set_title("Angle")
by.axis('off')

# Heuristic preperations
top = 1.2;
bottom = 0.8;
top_angle = 1.2
bottom_angle = 0.8
loop = 0

#plot count for saving frames
plotCount = 1;

# letting the pipe warm up for couple of seconds then starting to capture frames
playback.resume()
time.sleep(1)
try:
    while True:
    #while loop < 60:
        loop = loop + 1
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # pausing the playback to do heavy calculations
        playback.pause()

        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Extracting the width \ height from first frame
        if (first == True):
            first = False
            startingTimeStamp = aligned_depth_frame.timestamp
            Algebra.max_height_resulotion = aligned_depth_frame.height
            Algebra.max_width_resulotion = aligned_depth_frame.width

        # TimeStamps
        depth_timestamp = aligned_depth_frame.timestamp
        # checking if we already read this frame
        if depth_timestamp not in all_depth_timestamps:
            all_depth_timestamps.append(depth_timestamp)

            # marks code to valid the frame in log file
            color_frame_name = None
            for item in log_list:  # searching for color timestamp of corresponding color frame
                if item.depth_timestamp == str(depth_timestamp):  # python 2.7: change to item.depth_timestamp == str(depth_timestamp)
                #if float(item.depth_timestamp) == float(depth_timestamp):
                    color_frame_name = item.color_timestamp + ".png"
                    break

            # finding color's skeletonObject
            skeletonObject = None
            for item in skeletonsTable:  # searching for the full corresponding alphapose cloud object
                if item.image_id == color_frame_name:
                    skeletonObject = item
                    currentTimeStamp = float(item.image_id[:-4])
                    break

            # color_frame = open(color_frame_name)  # open the correct .png file

            # skeletonObject
            # aligned_depth_frame
            # color_frame_name.png

            # Generating 3d points from aligned_depth
            if skeletonObject != None and lastTimeStamp != currentTimeStamp:

                # Get height / width configurations.
                w, h = aligned_depth_frame.width, aligned_depth_frame.height

                pc = rs.pointcloud()
                points = pc.calculate(aligned_depth_frame)

                # Create a VertexList to hold pointcloud data
                # Will pre-allocates memory according to the attributes below
                # vertex_list = pyglet.graphics.vertex_list(
                #     w * h, 'v3f/stream', 't2f/stream', 'n3f/stream')

                points = pc.calculate(aligned_depth_frame)
                verts = np.asarray(points.get_vertices(2)).reshape(h, w, 3)

                # rotation
                verts = np.rot90(verts) # this is for images that were rotated 90 degrees
                isRotated = True # dont forget to disable this in cases u dont rotate

                # print("rawsize: {}, vertsSize: {}".format(vertsRaw.size, verts.size))
                texcoords = np.asarray(points.get_texture_coordinates(2))

                # writing 3d points inside alphapose object
                # ==================== AlphaPose
                #skeleton2 = Algebra.AlphaSkeleton(skeletonObject)
                for i in range(0, 75, 3):
                    # incase pose estimation didnt return a valid point.
                    if math.fabs(skeletonObject.keypoints[i]) == 0 and math.fabs(
                            skeletonObject.keypoints[i + 1]) == 0:
                        skeletonObject.keypoints[i] = 0
                        skeletonObject.keypoints[i + 1] = 0
                        skeletonObject.keypoints[i + 2] = 0
                    else:
                        pixel_x = int(math.floor(skeletonObject.keypoints[i]))
                        pixel_y = int(math.floor(skeletonObject.keypoints[i + 1]))
                        if Algebra.outOfBoundries(pixel_x, pixel_y,isRotated) == True:
                            skeletonObject.keypoints[i] = 0
                            skeletonObject.keypoints[i + 1] = 0
                            skeletonObject.keypoints[i + 2] = 0
                        else:
                            set1 = Algebra.eroding(pixel_y, pixel_x, aligned_depth_frame.height,
                                                   aligned_depth_frame.width,isRotated)
                            collect = []
                            for item in set1:
                                xyz = verts[item.x, item.y]
                                collect.append(Algebra.Point3D(xyz[0], xyz[1], xyz[2]))
                            xyz = verts[pixel_y, pixel_x]
                            point = Algebra.rounded(xyz[2], collect)
                            # xyz = verts[pixel_y, pixel_x]
                            skeletonObject.keypoints[i] = point.x
                            skeletonObject.keypoints[i + 1] = point.y
                            skeletonObject.keypoints[i + 2] = point.z

                # ALPHA POSE
                #skeleton = Algebra.AlphaSkeleton(skeletonObject)
                # OPEN POSE
                skeleton = Algebra.OpenPoseSkeleton(skeletonObject.keypoints)


                # butt distance from floor
                # floor_point_x = None
                # if firstFloor == True and Algebra.isZero(skeleton.rKnee) != True and Algebra.isZero(
                #         skeleton.lKnee) != True:
                #     firstFloor = False
                #     floor_point_x = (skeleton.lAnkle.x + skeleton.rAnkle.x) / 2
                #     floor_point_y = (skeleton.lAnkle.y + skeleton.rAnkle.y) / 2
                # if not Algebra.isZero(skeleton.hip):
                #     if len(dataX) < 3:
                #         dataX.append((currentTimeStamp - startingTimeStamp) / 1000)
                #         dataY.append(abs(skeleton.hip.x - floor_point_x))
                #     elif np.median(np.array(dataY[-3:])) * top > abs(skeleton.hip.x - floor_point_x) and np.median(
                #             np.array(dataY[-3:])) * bottom < abs(skeleton.hip.x - floor_point_x):
                #         dataX.append((currentTimeStamp - startingTimeStamp) / 1000)
                #         dataY.append(abs(skeleton.hip.x - floor_point_x))
                #         top = 1.3;
                #         bottom = 0.7;
                #     else:
                #         # print("top = " + str(top) + " ~ " + str(
                #         #     np.median(np.array(dataY[-3:])) * top) + " bottom = " + str(bottom) + " ~ " + str(
                #         #     np.median(np.array(dataY[-3:])) * bottom))
                #         top *= 1.3
                #         bottom *= 0.7
                #         # print(abs(skeleton.hip.x - floor_point_x))

                # # yaron's right hand from body side andle
                # if Algebra.isZero(skeleton.rShoulder) == False and Algebra.isZero(skeleton.rHip) == False and Algebra.isZero(skeleton.rElbow) == False:
                #     normalizeShoulderToElbow = Algebra.getNormalizeVector(skeleton.rShoulder,skeleton.rElbow)
                #     normalizeShoulderToHip = Algebra.getNormalizeVector(skeleton.rShoulder,skeleton.rHip)
                #     if (Algebra.isZero(normalizeShoulderToHip) == False and Algebra.isZero(normalizeShoulderToElbow) == False):
                #         angle = Algebra.getAngle(normalizeShoulderToElbow,normalizeShoulderToHip)
                #     else:
                #         angle = -1
                #     print(angle)
                # skeleton.rKnee.z += 0.045
                # skeleton.rAnkle.z += 0.01
                # skeleton.rHip.z += 0.
                # knee angle on squatting
                if (Algebra.isZero(skeleton.rHip) == False and Algebra.isZero(
                        skeleton.rKnee) == False and Algebra.isZero(skeleton.rAnkle) == False):
                    normalizeHipToKnee = Algebra.getNormalizeVector2D(skeleton.rHip, skeleton.rKnee)
                    normalizeKneeToAnkle = Algebra.getNormalizeVector2D(skeleton.rAnkle, skeleton.rKnee)
                    if (Algebra.isZero(normalizeHipToKnee) == False and Algebra.isZero(normalizeKneeToAnkle) == False):
                        angle = Algebra.getAngle2D(normalizeHipToKnee, normalizeKneeToAnkle)
                        dataX2.append((currentTimeStamp - startingTimeStamp) / 1000)
                        dataY2.append(angle)
                        print(angle)
                    else:
                        angle = -1

                    # if len(dataX2) < 3:
                    #     dataX2.append((currentTimeStamp - startingTimeStamp) / 1000)
                    #     dataY2.append(angle)
                    # elif np.median(np.array(dataY2[-3:])) * top_angle > angle and np.median(
                    #         np.array(dataY2[-3:])) * bottom_angle < angle:
                    #     dataX2.append((currentTimeStamp - startingTimeStamp) / 1000)
                    #     dataY2.append(angle)
                    #     top_angle = 1.4;
                    #     bottom_angle = 0.6;
                    # else:
                    #     print("top = " + str(top_angle) + " ~ " + str(
                    #         np.median(np.array(dataY2[-3:])) * top_angle) + " bottom = " + str(
                    #         bottom_angle) + " ~ " + str(np.median(np.array(dataY2[-3:])) * bottom_angle))
                    #     top_angle *= 1.3
                    #     bottom_angle *= 0.7
                    #     print(angle)
                    #     print("missed angle")
                # # # angle between base-spine to neck and base-spine to ankle
                # if (Algebra.isZero(skeleton.neck) == False and Algebra.isZero(skeleton.rKnee) == False and Algebra.isZero(skeleton.hip) == False):
                #     normalizeHipToNeck = Algebra.getNormalizeVector(skeleton.neck,skeleton.hip)
                #     normalizeHipToAnkle = Algebra.getNormalizeVector(skeleton.rAnkle, skeleton.hip)
                #     if (Algebra.isZero(normalizeHipToAnkle) == False and Algebra.isZero(normalizeHipToNeck) == False):
                #         angle = Algebra.getAngle(normalizeHipToAnkle,normalizeHipToNeck)
                #     else:
                #         angle = -1
                #     print(180-angle)

                # #front angle of body curve (C) from estimated crotch point to hip, and from hip to neck. Anterior Neck_Crotch angle
                # if Algebra.isZero(skeleton.neck) == False and Algebra.isZero(skeleton.hip) == False and Algebra.isZero(skeleton.rHip) == False and Algebra.isZero(skeleton.lHip) == False:
                #     normalizeHipToNeck = Algebra.getNormalizeVector(skeleton.hip,skeleton.neck)
                #
                #     # generating estimation of crotch point.
                #     estimatedCrotchPoint = Algebra.Point3D(((skeleton.rHip.x + skeleton.lHip.x)/2),((skeleton.rHip.y + skeleton.lHip.y)/2),skeleton.hip.z)
                #
                #     normalizeHipToCrotch = Algebra.getNormalizeVector(skeleton.hip,estimatedCrotchPoint)
                #
                #     if (Algebra.isZero(normalizeHipToCrotch) == False and Algebra.isZero(normalizeHipToNeck) == False):
                #         angle = Algebra.getAngle(normalizeHipToCrotch,normalizeHipToNeck)
                #     else:
                #         angle = -1
                #     print(180-angle)

                # front angle of Anterior Neck_Vertex. (head to neck \\ neck to crotch )
                # if Algebra.isZero(skeleton.neck) == False and Algebra.isZero(skeleton.head) == False and Algebra.isZero(
                #     skeleton.hip) == False and Algebra.isZero(skeleton.rHip) == False and Algebra.isZero(skeleton.lHip) == False:
                #
                #     estimatedCrotchPoint = Algebra.Point3D(((skeleton.rHip.x + skeleton.lHip.x) / 2), ((skeleton.rHip.y + skeleton.lHip.y) / 2), skeleton.hip.z)
                #     normalizeHeadToNeck = Algebra.getNormalizeVector(skeleton.neck,skeleton.head)
                #     normalizeNeckToCrotch = Algebra.getNormalizeVector(skeleton.neck,estimatedCrotchPoint)
                #     if (Algebra.isZero(normalizeNeckToCrotch) == False and Algebra.isZero(normalizeHeadToNeck) == False):
                #         angle = Algebra.getAngle(normalizeHeadToNeck,normalizeNeckToCrotch)
                #         print(180 - angle)
                #     else:
                #         angle = -1

                # # front angle of Anterior Neck_Waist
                # if Algebra.isZero(skeleton.neck) == False and Algebra.isZero(skeleton.head) == False and Algebra.isZero(
                #         skeleton.hip) == False:
                #     # skeleton.neck.z = 0
                #     # skeleton.hip.z = 0
                #     # skeleton.head.z = 0
                #     normalizeHeadToNeck = Algebra.getNormalizeVector(skeleton.neck, skeleton.head)
                #     normalizeNeckToHip = Algebra.getNormalizeVector(skeleton.neck, skeleton.hip)
                #     if (Algebra.isZero(normalizeNeckToHip) == False and Algebra.isZero(normalizeHeadToNeck) == False):
                #         angle = Algebra.getAngle(normalizeHeadToNeck, normalizeNeckToHip)
                #         print(180 - angle)
                #     else:
                #         angle = -1

                # generating points for sanity checkout

                # front angle of Anterior Neck_Waist
                # if Algebra.isZero(skeleton.rElbow) == False and Algebra.isZero(
                #         skeleton.rWrist) == False and Algebra.isZero(
                #         skeleton.rShoulder) == False:
                #     skeleton.rElbow.z = 0
                #     skeleton.rShoulder.z = 0
                #     skeleton.rWrist.z = 0
                #     normalizeShoulderToElbow = Algebra.getNormalizeVector(skeleton.rElbow, skeleton.rShoulder)
                #     normalizeElbowToWrist = Algebra.getNormalizeVector(skeleton.rElbow, skeleton.rWrist)
                #     if (Algebra.isZero(normalizeShoulderToElbow) == False and Algebra.isZero(
                #             normalizeElbowToWrist) == False):
                #         angle = Algebra.getAngle(normalizeShoulderToElbow, normalizeElbowToWrist)
                #         #print(180 - angle)
                #     else:
                #         angle = -1
                #     dataX2.append((currentTimeStamp - startingTimeStamp)/1000)
                #     dataY2.append(angle)

                # # get relevant points to draw
                # floor_point = Algebra.Point3D((skeleton2.rAnkle.x + skeleton2.lAnkle.x) / 2,
                #                               (skeleton2.rAnkle.y + skeleton2.lAnkle.y) / 2,
                #                               0)  # floor point, between ankles
                # yarech = Algebra.Point3D(0.5 * (skeleton2.rHip.x - skeleton2.rKnee.x) + skeleton2.rKnee.x,
                #                          0.5 * (skeleton2.rHip.y - skeleton2.rKnee.y) + skeleton2.rKnee.y,
                #                          0)  # yarech point
                # shock = Algebra.Point3D(0.5 * (skeleton2.rKnee.x - skeleton2.rAnkle.x) + skeleton2.rAnkle.x,
                #                         0.5 * (skeleton2.rKnee.y - skeleton2.rAnkle.y) + skeleton2.rAnkle.y,
                #                         0)  # shock point
                #
                # # deal with angle image
                # image_angle = color_image.copy()
                # cv2.line(image_angle, (int(yarech.x), int(yarech.y)), (int(skeleton2.rKnee.x), int(skeleton2.rKnee.y)),
                #          (255, 0, 0), thickness=4)  # draw yarech line
                # cv2.line(image_angle, (int(skeleton2.rKnee.x), int(skeleton2.rKnee.y)), (int(shock.x), int(shock.y)),
                #          (255, 0, 0), thickness=4)  # draw shock line
                #
                # # higher version of cv2
                # image_angle = cv2.rotate(image_angle, cv2.ROTATE_90_CLOCKWISE)  # rotate angle image
                #
                # # cv2 version 2.4
                # # (h, w) = image_angle.shape[:2]
                # # center = (w / 2, h / 2)
                # # M = cv2.getRotationMatrix2D(center, angle, 1.0)
                # # image_angle = cv2.warpAffine(image_angle, M, (h, w))
                #
                # # deal with distance image
                # image_distance = color_image.copy()
                # cv2.line(image_distance, (int(skeleton2.hip.x), int(skeleton2.hip.y)),
                #          (int(floor_point.x), int(floor_point.y)), (255, 0, 0),
                #          thickness=4)  # draw distance from butt to floor
                #
                # # higher version of cv2
                # image_distance = cv2.rotate(image_distance, cv2.ROTATE_90_CLOCKWISE)  # rotate distance image
                #
                # # cv2 version 2.4
                # # (h,w) = image_distance.shape[:2]
                # # center = (w / 2, h / 2)
                # # M = cv2.getRotationMatrix2D(center, angle, 1.0)
                # # image_distance = cv2.warpAffine(image_distance, M, (h, w))


                # update images with red lines
                # by.imshow(image_angle)
                # ay.imshow(image_distance)

                # plot graph
                # Algebra.roundGraph(dataX, dataY, ax, "Time (sec)", "Butt to floor distance (meter)", 1.5)
                # Algebra.roundGraph(dataX2, dataY2, bx, "Time (sec)", "Knee angle (degrees)", 200)

                # saving frame
                # plt.savefig('plot{}.png'.format(plotCount))
                # plotCount += 1

                # need this to continue running
                #plt.pause(0.01)

            numOfFrames += 1
            #print(numOfFrames)
            lastTimeStamp = currentTimeStamp
            # Algebra.roundGraph(dataX,dataY,ax)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        # resuming after heavy calculations.
        playback.resume()


finally:
    pipeline.stop()
    # save last state of plot
    #Algebra.roundGraph(dataX, dataY, ax, "Time (sec)", "Butt to floor distance (meter)", 1.5)
    Algebra.roundGraph(dataX2, dataY2, bx, "Time (sec)", "Knee angle (degrees)", 200)
    plt.savefig("plotNormal.pdf")
    # save last state of gaussian plot
    #Algebra.roundGraph(dataX, ndimage.gaussian_filter1d(dataY, 1), ax,"Time (sec)", "Butt to floor distance (meter)", 1.5)
    Algebra.roundGraph(dataX2, ndimage.gaussian_filter1d(dataY2, 1), bx,"Time (sec)", "Knee angle (degrees)", 200)
    plt.savefig("plotSmoothGauss1.pdf")
    #Algebra.roundGraph(dataX, ndimage.gaussian_filter1d(dataY, 2), ax,"Time (sec)", "Butt to floor distance (meter)", 1.5)
    Algebra.roundGraph(dataX2, ndimage.gaussian_filter1d(dataY2, 2), bx,"Time (sec)", "Knee angle (degrees)", 200)
    plt.savefig("plotSmoothGauss2.pdf")

    # call addToExcel(filename,dataX,dataY,heading,headingX,headingY), dataX=[...], dataY=[....]
    # addToExcel(excelFilename, dataX, dataX2, dataY, dataY2, heading, headingX, headingY, headingY2)
    print("successfully finished, number of frames: {}".format(numOfFrames))
