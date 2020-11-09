## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

############################################################################################
## this script uses bag file, the log, skeleton-key points, to extract and calculate data ##
############################################################################################
from mpl_toolkits import mplot3d
import argparse
import json
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import xlsxwriter

import sys

from Excel import addToExcel

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
            if self.score > skeletonsTable[index]:
                skeletonsTable.pop(index)
                skeletonsTable.append(self)
        else:
            skeletonsTable.append(self)

#

# open's the alphapose results and sets them in json array
skeletonsTable = []  # contains all json's, with the bigger score
alphapose_file = open('alphapose-results.json')
alphapose_array = json.load(alphapose_file)

for item in alphapose_array:
    x = AlphaPoseObject(item['image_id'], item['score'], item['box'], item['keypoints'])
    x.add()

heading = "Dana_Squat_1.5_Front_color"
headingX = "Time (sec)"
headingY = "Distance butt-to-floor (cm)"
headingY2 = "Knees angle (cm)"
excelFilename = "chart_scatter.xlsx"
dataX = []
dataX2 = []
dataY = []
dataY2 = []
time = []
startingTimeStamp = 0
currentTimeStamp = 0

json_log_name ="log_color.json"

# open's the log as json array.
log_file = open(json_log_name)
log_array = json.load(log_file)
log_list = []

#generating the log_list
for item in log_array:
    log_list.append(Timestamp(item['color_timestamp'], item['depth_timestamp']))      # json_list contains all pairs of depth & color

# Setup:
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("C:\Users\markf\Downloads\Project\BAG files\Second\Dana_Squat_1.5_Front.bag", False)

profile = pipeline.start(cfg)
device = profile.get_device()
playback = device.as_playback()  # this allows the use of pause and resume.

#pasuing playback to let the pipe warm up
playback.pause

first = True
firstFloor = True

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

# letting the pipe warm up for couple of seconds then starting to capture frames
playback.resume
#time.sleep(10)
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # pausing the playback to do heavy calculations
        playback.pause()


        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()


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
                if item.depth_timestamp == str(depth_timestamp):
                    color_frame_name = item.color_timestamp + ".png"
                    break

            # finding color's alphaposeObject
            alphaposeObject = None
            for item in skeletonsTable:  # searching for the full corresponding alphapose cloud object
                if item.image_id == color_frame_name:
                    alphaposeObject = item
                    currentTimeStamp = float(item.image_id[:-4])
                    break

            # color_frame = open(color_frame_name)  # open the correct .png file




            # alphaposeObject
            # aligned_depth_frame
            #color_frame_name.png


            # Generating 3d points from aligned_depth
            if alphaposeObject != None:
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
                # print("rawsize: {}, vertsSize: {}".format(vertsRaw.size, verts.size))
                texcoords = np.asarray(points.get_texture_coordinates(2))


                # writing 3d points inside alphapose object
                for i in range(0,78,3):
                    # incase pose estimation didnt return a valid point.
                    if math.fabs(alphaposeObject.keypoints[i]) == 0 and math.fabs(alphaposeObject.keypoints[i+1]) == 0:
                        alphaposeObject.keypoints[i] = 0
                        alphaposeObject.keypoints[i + 1] = 0
                        alphaposeObject.keypoints[i + 2] = 0
                    else:
                        pixel_x = int(math.floor(alphaposeObject.keypoints[i]))
                        pixel_y = int(math.floor(alphaposeObject.keypoints[i+1]))
                        if Algebra.outOfBoundries(pixel_x,pixel_y) == True:
                            alphaposeObject.keypoints[i] = 0
                            alphaposeObject.keypoints[i + 1] = 0
                            alphaposeObject.keypoints[i + 2] = 0
                        else:
                            xyz = verts[pixel_y,pixel_x]
                            alphaposeObject.keypoints[i] = xyz[0]
                            alphaposeObject.keypoints[i+1] = xyz[1]
                            alphaposeObject.keypoints[i+2] = xyz[2]

                skeleton = Algebra.AlphaSkeleton(alphaposeObject)


                # butt distance from floor
                floor_point_x = None
                if firstFloor == True and Algebra.isZero(skeleton.rKnee) != True and Algebra.isZero(skeleton.lKnee) != True:
                    first = False
                    floor_point_x = (skeleton.lAnkle.x + skeleton.rAnkle.x) / 2
                if not Algebra.isZero(skeleton.head):
                    dataX.append((currentTimeStamp - startingTimeStamp)/1000)
                    dataY.append((skeleton.hip.x - floor_point_x))
                    # head_x = skeleton.hip.x
                    # if hip_x - floor_point_x > 2:
                    #     print("here")
    #                print(skeleton.head.x - floor_point_x)


                # # yaron's right hand from body side andle
                # if Algebra.isZero(skeleton.rShoulder) == False and Algebra.isZero(skeleton.rHip) == False and Algebra.isZero(skeleton.rElbow) == False:
                #     normalizeShoulderToElbow = Algebra.getNormalizeVector(skeleton.rShoulder,skeleton.rElbow)
                #     normalizeShoulderToHip = Algebra.getNormalizeVector(skeleton.rShoulder,skeleton.rHip)
                #     if (Algebra.isZero(normalizeShoulderToHip) == False and Algebra.isZero(normalizeShoulderToElbow) == False):
                #         angle = Algebra.getAngle(normalizeShoulderToElbow,normalizeShoulderToHip)
                #     else:
                #         angle = -1
                #     print(angle)

                # # knee angle on squatting
                # if (Algebra.isZero(skeleton.rHip) == False and Algebra.isZero(skeleton.rKnee) == False and Algebra.isZero(skeleton.rAnkle) == False):
                #     normalizeHipToKnee = Algebra.getNormalizeVector(skeleton.rHip,skeleton.rKnee)
                #     normalizeKneeToAnkle = Algebra.getNormalizeVector(skeleton.rAnkle, skeleton.rKnee)
                #     if (Algebra.isZero(normalizeHipToKnee) == False and Algebra.isZero(normalizeKneeToAnkle) == False):
                #         angle = Algebra.getAngle(normalizeHipToKnee,normalizeKneeToAnkle)
                #     else:
                #         angle = -1
                #     print(angle)



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
                if Algebra.isZero(skeleton.rElbow) == False and Algebra.isZero(
                        skeleton.rWrist) == False and Algebra.isZero(
                        skeleton.rShoulder) == False:
                    skeleton.rElbow.z = 0
                    skeleton.rShoulder.z = 0
                    skeleton.rWrist.z = 0
                    normalizeShoulderToElbow = Algebra.getNormalizeVector(skeleton.rElbow, skeleton.rShoulder)
                    normalizeElbowToWrist = Algebra.getNormalizeVector(skeleton.rElbow, skeleton.rWrist)
                    if (Algebra.isZero(normalizeShoulderToElbow) == False and Algebra.isZero(
                            normalizeElbowToWrist) == False):
                        angle = Algebra.getAngle(normalizeShoulderToElbow, normalizeElbowToWrist)
                        #print(180 - angle)
                    else:
                        angle = -1
                    dataX2.append((currentTimeStamp - startingTimeStamp)/1000)
                    dataY2.append(angle)







                    # fig = plt.figure()
                    # ax = plt.axes(projection='3d')
                    # ax.scatter(skeleton.hip.x, skeleton.hip.y, skeleton.hip.z, cmap='blue', linewidth=0.5);
                    # ax.scatter(skeleton.rHip.x,skeleton.rHip.y,skeleton.rHip.z, cmap='blue', linewidth=0.5);
                    # ax.scatter(skeleton.lHip.x,skeleton.lHip.y,skeleton.lHip.z, cmap='blue', linewidth=0.5);
                    #
                    # ax.scatter(estimatedCrotchPoint.x,estimatedCrotchPoint.y,estimatedCrotchPoint.z, cmap='green', linewidth=0.5);







                # visualisation test
                # fig = plt.figure()
                # ax = plt.axes(projection='3d')
                # for i in range(0,78,3):
                #     ax.scatter( alphaposeObject.keypoints[i], alphaposeObject.keypoints[i+1], alphaposeObject.keypoints[i+2], cmap='viridis', linewidth=0.5);




            numOfFrames += 1

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
        # resuming after heavy calculations.
        playback.resume()



finally:
    pipeline.stop()
    # call addToExcel(filename,dataX,dataY,heading,headingX,headingY), dataX=[...], dataY=[....]
    addToExcel(excelFilename, dataX, dataX2, dataY, dataY2, heading, headingX, headingY, headingY2)
    print("successfully finished, number of frames: {}".format(numOfFrames))









