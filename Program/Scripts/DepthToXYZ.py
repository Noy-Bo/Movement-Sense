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
import scipy.stats
#from Excel import addToExcel
import sys

import Scripts.ViconCalculations
from Calculations.Algebra import OpenPoseReader
from Scripts import ViconCalculations

sys.path.append('C:\Python27\Lib\site-packages')

import numpy as np
import pyrealsense2 as rs
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
skeletonsTable = OpenPoseReader()


# ===========
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
fig = plt.figure(num=None, figsize=(18, 10))

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
    #while loop < 5:
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
                #if str(math.floor(float(item.depth_timestamp))) == str(round(float(depth_timestamp))):  # python 2.7: change to item.depth_timestamp == str(depth_timestamp)

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
                #pixelSkeleton = Algebra.AlphaSkeleton(skeletonObject)
                #==================== OpenPose
                pixelSkeleton = Algebra.OpenPoseSkeleton(skeletonObject.keypoints)

               #  #printing points test
               #  color_image_copy = color_image.copy()
               #  color_image_copy = ndimage.rotate(color_image, 90)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.rShoulder.x),int(pixelSkeleton.rShoulder.y)),4,(255,0,0),-1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.lShoulder.x),int(pixelSkeleton.lShoulder.y)),4,(0,255,0),-1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.rKnee.x), int(pixelSkeleton.rKnee.y)), 4,
               #             (255, 0, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.lKnee.x), int(pixelSkeleton.lKnee.y)), 4,
               #             (0, 255, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.lHip.x), int(pixelSkeleton.lHip.y)), 4,
               #             (0, 255, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.rHip.x), int(pixelSkeleton.rHip.y)), 4,
               #             (255, 0, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.lAnkle.x), int(pixelSkeleton.lAnkle.y)), 4,
               #             (0, 255, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.rAnkle.x), int(pixelSkeleton.rAnkle.y)), 4,
               #             (255, 0, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.lElbow.x), int(pixelSkeleton.lElbow.y)), 4,
               #             (0, 255, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.rElbow.x), int(pixelSkeleton.rElbow.y)), 4,
               #             (255, 0, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.neck.x), int(pixelSkeleton.neck.y)), 4,
               #             (0, 0, 255), -1)
               # # cv2.circle(color_image_copy, (int(pixelSkeleton.midHip.x), int(pixelSkeleton.midHip.y)), 4,
               #  #           (0, 0, 255), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.lShoulder.x), int(pixelSkeleton.lShoulder.y)), 4,
               #             (0, 255, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.rWrist.x), int(pixelSkeleton.rWrist.y)), 4,
               #             (255, 0, 0), -1)
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.lWrist.x), int(pixelSkeleton.lWrist.y)), 4,
               #             (0, 255, 0), -1)
               #
               #
               #  cv2.circle(color_image_copy, (int(pixelSkeleton.neck.x),int(pixelSkeleton.neck.y)),4,(0,0,255),-1)
               #  print(int(pixelSkeleton.lShoulder.x),int(pixelSkeleton.lShoulder.y))
               #  #cv2.circle(color_image_copy, (10,200),4,(255,0,0),-1)
               #  cv2.imshow("sanity check",color_image_copy)

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

                # #ALPHA POSE
                #skeleton = Algebra.AlphaSkeleton(skeletonObject)
                # OPEN POSE
                skeleton = Algebra.OpenPoseSkeleton(skeletonObject.keypoints)


                # #butt distance from floor
                # if firstFloor == True and Algebra.isZero(skeleton.rKnee) != True and Algebra.isZero(
                #         skeleton.lKnee) != True:
                #     firstFloor = False
                #     floor_point_x = (skeleton.lAnkle.x + skeleton.rAnkle.x) / 2
                #     floor_point_y = (skeleton.lAnkle.y + skeleton.rAnkle.y) / 2
                # if not Algebra.isZero(skeleton.rHip):
                #     if floor_point_x is not None and floor_point_y is not None:
                #         dataX.append((currentTimeStamp - startingTimeStamp) / 1000)
                #         dataY.append(abs(skeleton.rHip.x - floor_point_x))

                        #
                        # if len(dataX) < 3 :
                        #     dataX.append((currentTimeStamp - startingTimeStamp) / 1000)
                        #     dataY.append(abs(skeleton.rHip.x - floor_point_x))
                        # elif np.median(np.array(dataY[-3:])) * top > abs(skeleton.rHip.x - floor_point_x) and np.median(
                        #         np.array(dataY[-3:])) * bottom < abs(skeleton.rHip.x - floor_point_x):
                        #     dataX.append((currentTimeStamp - startingTimeStamp) / 1000)
                        #     dataY.append(abs(skeleton.rHip.x - floor_point_x))
                        #     top = 1.3;
                        #     bottom = 0.7;
                        # else:
                        #     # print("top = " + str(top) + " ~ " + str(
                        #     #     np.median(np.array(dataY[-3:])) * top) + " bottom = " + str(bottom) + " ~ " + str(
                        #     #     np.median(np.array(dataY[-3:])) * bottom))
                        #     top *= 1.3
                        #     bottom *= 0.7
                        #     # print(abs(skeleton.hip.x - floor_point_x))



                # # kyphosis
                # if (Algebra.isZero(skeleton.rShoulder) == False and Algebra.isZero(
                #         skeleton.lShoulder) == False and Algebra.isZero(skeleton.neck) == False):
                #
                #     skeleton.rShoulder.y = skeleton.lShoulder.y = skeleton.neck.y
                #     rshoulder = np.array([skeleton.rShoulder.x,skeleton.rShoulder.y,skeleton.rShoulder.z])
                #     lshoulder = np.array([skeleton.lShoulder.x,skeleton.lShoulder.y,skeleton.lShoulder.z])
                #     neck = np.array([skeleton.neck.x,skeleton.neck.y,skeleton.neck.z])
                #
                #     rightShoulderToNeck = Algebra.getVectorFrom2Points(rshoulder,neck)
                #     leftShoulderToNeck = Algebra.getVectorFrom2Points(lshoulder,neck)
                #
                #     angle = Algebra.getAngle(rightShoulderToNeck,leftShoulderToNeck)
                #     angle = 180 - angle
                #
                #     dataX2.append((currentTimeStamp - startingTimeStamp) / 1000)
                #     dataY2.append(angle)
                #     print(angle)

                # knee angle (right)
                if (Algebra.isZero(skeleton.rHip) == False and Algebra.isZero(
                        skeleton.rKnee) == False and Algebra.isZero(skeleton.rAnkle) == False):
                    rHip = np.array([skeleton.rHip.x, skeleton.rHip.y, skeleton.rHip.z])
                    rKnee = np.array([skeleton.rKnee.x, skeleton.rKnee.y, skeleton.rKnee.z])
                    rAnkle = np.array([skeleton.rAnkle.x, skeleton.rAnkle.y, skeleton.rAnkle.z])

                    hipToKnee = Algebra.getVectorFrom2Points(rHip, rKnee)
                    ankleToKnee = Algebra.getVectorFrom2Points(rAnkle, rKnee)

                    angle = Algebra.getAngle(hipToKnee, ankleToKnee)
                    # angle = 180 - angle

                    dataX2.append((currentTimeStamp - startingTimeStamp) / 1000)
                    dataY2.append(angle)
                    print(angle)


                # # get relevant points to draw
                # floor_point = Algebra.Point3D(pixelSkeleton.rHip.x,
                #                               (pixelSkeleton.rAnkle.y + pixelSkeleton.lAnkle.y) / 2,
                #                               0)  # floor point, between ankles
                # yarech = Algebra.Point3D(0.5 * (pixelSkeleton.rHip.x - pixelSkeleton.rKnee.x) + pixelSkeleton.rKnee.x,
                #                          0.5 * (pixelSkeleton.rHip.y - pixelSkeleton.rKnee.y) + pixelSkeleton.rKnee.y,
                #                          0)  # yarech point
                # shock = Algebra.Point3D(0.5 * (pixelSkeleton.rKnee.x - pixelSkeleton.rAnkle.x) + pixelSkeleton.rAnkle.x,
                #                         0.5 * (pixelSkeleton.rKnee.y - pixelSkeleton.rAnkle.y) + pixelSkeleton.rAnkle.y,
                #                         0)  # shock point
                #
                # # deal with angle image
                # image_angle = color_image.copy()
                # #cv2.imshow("sanity check", image_angle)
                #
                # # higher version of cv2
                # image_angle = cv2.rotate(image_angle, cv2.ROTATE_90_COUNTERCLOCKWISE)  # rotate angle image
                # cv2.line(image_angle, (int(yarech.x), int(yarech.y)), (int(pixelSkeleton.rKnee.x), int(pixelSkeleton.rKnee.y)),
                #          (255, 0, 0), thickness=4)  # draw yarech line
                # cv2.line(image_angle, (int(pixelSkeleton.rKnee.x), int(pixelSkeleton.rKnee.y)), (int(shock.x), int(shock.y)),
                #          (255, 0, 0), thickness=4)  # draw shock line
                # cv2.circle(image_angle, (int(yarech.x), int(yarech.y)), 4,
                #              (0, 255, 0), -1)
                # cv2.circle(image_angle, (int(pixelSkeleton.rKnee.x), int(pixelSkeleton.rKnee.y)), 4,
                #            (0, 255, 0), -1)
                # cv2.circle(image_angle, (int(shock.x), int(shock.y)), 4,
                #            (0, 255, 0), -1)
                #
                #
                # # cv2 version 2.4
                # # (h, w) = image_angle.shape[:2]
                # # center = (w / 2, h / 2)
                # # M = cv2.getRotationMatrix2D(center, angle, 1.0)
                # # image_angle = cv2.warpAffine(image_angle, M, (h, w))
                #
                # # deal with distance image
                # image_distance = color_image.copy()
                # # higher version of cv2
                # image_distance = cv2.rotate(image_distance, cv2.ROTATE_90_COUNTERCLOCKWISE)  # rotate distance image
                #
                # cv2.line(image_distance, (int(pixelSkeleton.rHip.x), int(pixelSkeleton.rHip.y)),
                #          (int(floor_point.x), int(floor_point.y)), (255, 0, 0),
                #          thickness=4)  # draw distance from butt to floor
                # cv2.circle(image_distance, (int(pixelSkeleton.rHip.x), int(pixelSkeleton.rHip.y)), 4,
                #            (0, 255, 0), -1)
                # cv2.circle(image_distance, (int(floor_point.x), int(floor_point.y)), 4,
                #            (0, 255, 0), -1)
                #
                #
                #
                # # cv2 version 2.4
                # # (h,w) = image_distance.shape[:2]
                # # center = (w / 2, h / 2)
                # # M = cv2.getRotationMatrix2D(center, angle, 1.0)
                # # image_distance = cv2.warpAffine(image_distance, M, (h, w))
                #
                #
                # # update images with red lines
                # by.imshow(image_angle)
                # ay.imshow(image_distance)
                #
                # #plot graph
                # Algebra.roundGraph(dataX, dataY, ax, "Time (sec)", "Butt to floor distance (meter)", 1.5)
                # Algebra.roundGraph(dataX2, dataY2, bx, "Time (sec)", "Knee angle (degrees)", 200)
                #
                #
                # #saving frame
                # plt.savefig('plot{}.png'.format(plotCount))
                # plotCount += 1
                #
                # #need this to continue running
                # plt.pause(0.01)

            numOfFrames += 1
            #print(numOfFrames)
            lastTimeStamp = currentTimeStamp
            #Algebra.roundGraph(dataX,dataY,ax)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        # resuming after heavy calculations.
        playback.resume()


finally:
    pipeline.stop()
    # save last state of plot
    cut_dataX_vicon = []
    cut_dataY_vicon = []
    dataX_vicon,dataY_vicon = ViconCalculations.calculate()

    # cleaning vicon ASI (specific bag)
    for idx in range(0,len(dataY_vicon)):
        if dataY_vicon[idx] > 68:
            cut_dataX_vicon.append(dataX_vicon[idx])
            cut_dataY_vicon.append(dataY_vicon[idx])
    dataX_vicon = cut_dataX_vicon
    dataY_vicon = cut_dataY_vicon
    cut_dataY_vicon = []
    cut_dataX_vicon = []

    # matching our shooting to vicon's via timestamps
    for idx in range(0,len(dataX2)):
        vicon_timestamp = Algebra.takeClosest(dataX2[idx],dataX_vicon)
        for vIdx in range(0,len(dataX_vicon)):
            if vicon_timestamp == dataX_vicon[vIdx]:
                cut_dataX_vicon.append(dataX_vicon[vIdx])
                cut_dataY_vicon.append(dataY_vicon[vIdx])


    #Algebra.roundGraph(dataX, dataY, ax, "Time (sec)", "Butt to floor distance (meter)", 1.5)
    Algebra.roundGraph(cut_dataX_vicon, cut_dataY_vicon, bx, "Time (sec)", "Knee angle (degrees)", 200,'r')
    Algebra.roundGraph(dataX2, dataY2, bx, "Time (sec)", "Knee angle (degrees)", 200)
    plt.savefig("plotNormal.pdf")
    bx.clear()
    # save last state of gaussian plot
    #Algebra.roundGraph(dataX, ndimage.gaussian_filter1d(dataY, 1), ax,"Time (sec)", "Butt to floor distance (meter)", 1.5)
    Algebra.roundGraph(cut_dataX_vicon, cut_dataY_vicon, bx, "Time (sec)", "Knee angle (degrees)", 200, 'r')
    Algebra.roundGraph(dataX2, ndimage.gaussian_filter1d(dataY2, 1), bx,"Time (sec)", "Knee angle (degrees)", 200)
    plt.savefig("plotSmoothGauss1.pdf")
    bx.clear()
    #Algebra.roundGraph(dataX, ndimage.gaussian_filter1d(dataY, 2), ax,"Time (sec)", "Butt to floor distance (meter)", 1.5)
    Algebra.roundGraph(cut_dataX_vicon, cut_dataY_vicon, bx, "Time (sec)", "Knee angle (degrees)", 200, 'r')
    Algebra.roundGraph(dataX2, ndimage.gaussian_filter1d(dataY2, 2), bx,"Time (sec)", "Knee angle (degrees)", 200)
    plt.savefig("plotSmoothGauss2.pdf")
    bx.clear()


    # CORRELATION

    # correlation parameters
    peterson = scipy.stats.pearsonr(cut_dataY_vicon, dataY2)  # Pearson's r
    spearman = scipy.stats.spearmanr(cut_dataY_vicon, dataY2)  # Spearman's rho
    kendall = scipy.stats.kendalltau(cut_dataY_vicon, dataY2)  # Kendall's tau


    # graph

    fig = plt.figure(num=None, figsize=(20, 12))
    ax = fig.add_subplot(111)
    Algebra.roundGraph(cut_dataY_vicon, dataY2, ax, "Vicon", "Realsense", 200, 'r')
    ax.text(1, 1,("Pearson's r: {} \n Spearman's rho: {} \n Kendall's tau: {} \n".format(str(peterson[0])[:-12], str(spearman[0])[:-12],
                                                                                      str(kendall[0])[:-12])),
     horizontalalignment='right',
     verticalalignment='top',
     transform = ax.transAxes)
    plt.savefig("correlation.pdf")
    bx.clear()





    # call addToExcel(filename,dataX,dataY,heading,headingX,headingY), dataX=[...], dataY=[....]
    # addToExcel(excelFilename, dataX, dataX2, dataY, dataY2, heading, headingX, headingY, headingY2)
    print("successfully finished, number of frames: {}".format(numOfFrames))
