## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

################################################################################
##       script to generate log file with aligned to color depths frames      ##
##           saves the color frame as png with its timestamp as title         ##
##             generates list with png names sorted by timestamps             ##
################################################################################

import argparse
import time

import numpy as np
import pyrealsense2 as rs

import cv2
import csv

# Setup:
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("C:\Age_Estimation_Project\\bag_files\Eliran_Squat_1.5_Front.bag", True)
logfile_name = "LOG_Eliran_Squat_150_Front.txt"
listfile_name = "list.txt"



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
all_color_timestamps = []

color_depth = []
timestamps = []


# Streaming loop
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
parser.add_argument("-i", "--input", type=str, help="Bag file to read")
args = parser.parse_args()
t_end = time.time() + 60*60*2
try:
    while time.time() < t_end:

        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        # pausing the playback to do heavy calculations
        playback.pause()

        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        # TimeStamps
        color_timestamp = aligned_color_frame.timestamp
        depth_timestamp = aligned_depth_frame.timestamp

        color_timestamp_str = str(color_timestamp)
        depth_timestamp_str = str(depth_timestamp)

        color_depth.append(color_timestamp_str)
        color_depth.append(depth_timestamp_str)

        # checking foor new captured frame
        if (color_timestamp_str not in all_color_timestamps):
            print("new frame captured!")

            #adding the new frame to the bank
            all_color_timestamps.append(color_timestamp_str)
            timestamps.append(color_depth)

            #writing the png to dir
            # writing image & list
            color_image = np.asanyarray(aligned_color_frame.get_data())
            cv2.imwrite("{}.png".format(color_timestamp_str), color_image)

            numOfFrames += 1

        # reseting the data
        color_depth = []


        numOfFrames += 1

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

        # resuming after heavy calculations.
        playback.resume()



finally:
    pipeline.stop()


    #turning back to float and sorting
    log = open(logfile_name, "a")
    list = open(listfile_name, "a")
    log.write("\taligned to color:\n\n")

    #converting to float
    for stamp in timestamps: # iterating over timestamps
        stamp[0] = float(stamp[0])
        stamp[1] = float(stamp[1])

    #sort stamps after converting to float
    timestamps.sort

    # ~~WARNING!~~ DELETES THE CURRENT FILES!!
    # opening files to write into
    log = open(logfile_name, "w")
    list = open(listfile_name, "w")

    for stamp in timestamps:

        #writing to log file
        log.write("color timestamp: {}\n".format(stamp[0]))
        log.write("depth timestamp: {}\n\n".format(stamp[1]))

        #writing to list file
        list = open(listfile_name, "a")
        list.write("{}.png\n".format(stamp[0]))







