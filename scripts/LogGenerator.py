## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

################################################################################
##       script to generate log file with aligned to color depths frames      ##
##           saves the color frame as png with its timestamp as title         ##
##             generates list with png names sorted by timestamps             ##
################################################################################
from scipy import ndimage
import argparse
import json
import time

import numpy as np
import pyrealsense2 as rs

import cv2

takeClosest = lambda num,collection:min(collection,key=lambda x:abs(x-num))
# Setup:
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("C:\Age_Estimation_Project\\bag_files\Sub002_Stand_Front.bag", True)
logfile_color_name = "Sub002_Stand_Front_color.txt"
logfile_depth_name = "Sub002_Stand_Front_depth.txt"
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
clipping_distance_in_meters = 2.5  #  meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)
numOfFrames = 0
numOfColorFrames = 0
numOfDepthFrames = 0
# field names
all_depth_timestamps = []
all_color_timestamps = []

color_depth = []
timestamps = []
timestampsColorJson = []
timestampsDepthJson = []


# Streaming loop
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
parser.add_argument("-i", "--input", type=str, help="Bag file to read")
args = parser.parse_args()
t_end = time.time() + 60 * 7
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

        # object with depth/color mapping
        color_depth.append(color_timestamp_str)
        color_depth.append(depth_timestamp_str)

        #NEW
        # catching new color frame
        if (color_timestamp_str not in all_color_timestamps):
            all_color_timestamps.append(color_timestamp_str)

            # writing the png to dir
            color_image = np.asanyarray(aligned_color_frame.get_data())

            # remove background
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            grey_color = 153
            depth_image_3d = np.dstack(
                (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
            color_image = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)


            # rotation angle in degree
            rotated = ndimage.rotate(color_image, 90)

            # writing the png to dir
            cv2.imwrite("{}.png".format(color_timestamp_str), rotated)

            numOfColorFrames += 1

        # catching new color frame
        if (depth_timestamp_str not in all_depth_timestamps):
            all_depth_timestamps.append(depth_timestamp_str)
            numOfDepthFrames += 1

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




    # ~~WARNING!~~ DELETES THE CURRENT FILES!!
    # opening files to write into
    log_color = open(logfile_color_name, "w")
    log_depth = open(logfile_depth_name, "w")
    list = open(listfile_name, "w")


    #converting to float
    for i in range(0,len(all_depth_timestamps)):
       all_depth_timestamps[i] = float(all_depth_timestamps[i])

    for i in range(0, len(all_color_timestamps)):
        all_color_timestamps[i] = float(all_color_timestamps[i])

    # sorting arrays
    all_color_timestamps.sort
    all_depth_timestamps.sort

    # building log by color
    logfile_by_color = []
    for stamp in all_color_timestamps:
        color_depth.append(stamp)
        color_depth.append(takeClosest(stamp,all_depth_timestamps))
        logfile_by_color.append(color_depth)
        color_depth = []

    # writing to color_log and list
    log_color = open(logfile_color_name, "a")
    list = open(listfile_name, "a")

    for cd in logfile_by_color:

        # writing to log file
        log_color.write("depth timestamp: {}\n".format(cd[1]))
        log_color.write("color timestamp: {}\n\n".format(cd[0]))

        # writing to json
        # sorry futre Noy and Mark, im just too lazy~
        for stamp in all_color_timestamps:
            if stamp == cd[0]:
                color_str = str(stamp)
                break;


        for stamp in all_depth_timestamps:
            if stamp == cd[1]:
                depth_str = str(stamp)
                break;

        timestampsColorJson.append({'color_timestamp': color_str, 'depth_timestamp': depth_str})

        # writing to list file
        list.write("{}.png\n".format(cd[0]))

    list.close()
    log_color.close()


    # building log by depth

    logfile_by_depth = []
    for stamp in all_depth_timestamps:
        color_depth.append(takeClosest(stamp, all_color_timestamps))
        color_depth.append(stamp)
        logfile_by_depth.append(color_depth)
        color_depth = []

    # writing to depth_log
    log_depth = open(logfile_depth_name, "a")

    for cd in logfile_by_depth:
        # writing to log file
        log_depth.write("depth timestamp: {}\n".format(cd[1]))
        log_depth.write("color timestamp: {}\n\n".format(cd[0]))

        # writing to json
        # sorry future Noy and Mark, im just too lazy~
        for stamp in all_color_timestamps:
            if stamp == cd[0]:
                color_str = str(stamp)
                break;

        for stamp in all_depth_timestamps:
            if stamp == cd[1]:
                depth_str = str(stamp)
                break;

        timestampsDepthJson.append({'color_timestamp': color_str, 'depth_timestamp': depth_str})


    log_depth.close()

    with open('log_color.json', "w") as f:
        json.dump(timestampsColorJson, f)

    with open('log_depth.json', "w") as f:
        json.dump(timestampsDepthJson, f)



    print("succesfully captured {} color frames and {} depth frames".format(numOfColorFrames,numOfDepthFrames));




