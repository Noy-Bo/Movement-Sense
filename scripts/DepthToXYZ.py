## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

################################################################################
##                                                                            ##
################################################################################
from mpl_toolkits import mplot3d
import argparse
import json
import math
import time
import numpy as np
import matplotlib.pyplot as plt

import numpy as np
import pyrealsense2 as rs
import Algebra
import cv2
import csv


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




# Setup:
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("C:\Age_Estimation_Project\\bag_files\Eliran_Squat_1.5_Front.bag", False)
logfile_name = "LOG_Eliran_Squat_150_Front.txt"



profile = pipeline.start(cfg)
device = profile.get_device()
playback = device.as_playback()  # this allows the use of pause and resume.

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

# marks code to load the skeleton keys object

# open's the alphapose results and sets them in json array
skeletonsTable = []  # contains all json's, with the bigger score
alphapose_file = open('alphapose-results.json')
alphapose_array = json.load(alphapose_file)

for item in alphapose_array:
    x = AlphaPoseObject(item['image_id'], item['score'], item['box'], item['keypoints'])
    x.add()


# open's the log as json array.
log_file = open('log.json')
log_array = json.load(log_file)
log_list = []

#generating the log_list
for item in log_array:
    log_list.append(Timestamp(item['color_timestamp'], item['depth_timestamp']))      # json_list contains all pairs of depth & color

try:
    while True:

        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # pausing the playback to do heavy calculations
        playback.pause()

        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()

        # TimeStamps
        depth_timestamp = aligned_depth_frame.timestamp

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
                pixel_x = int(math.floor(alphaposeObject.keypoints[i]))
                pixel_y = int(math.floor(alphaposeObject.keypoints[i+1]))
                xyz = verts[pixel_y,pixel_x]
                alphaposeObject.keypoints[i] = xyz[0]
                alphaposeObject.keypoints[i+1] = xyz[1]
                alphaposeObject.keypoints[i+2] = xyz[2]

            skeleton = Algebra.AlphaSkeleton(alphaposeObject)





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









