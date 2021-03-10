## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

################################################################################
##              script to export csv and aligned png from bag file            ##
################################################################################

# First import the library
import argparse

import imageio
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import csv

# Setup:
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("C:\omer_burpee_side\Omer_Burpee_Side.bag",False)

profile = pipeline.start(cfg)
device = profile.get_device()
playback = device.as_playback() # this allows the use of pause and resume.

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 3  # 1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)
numOfFrames = 0
# field names
row = []
rows = []

# # Streaming loop
# parser = argparse.ArgumentParser()
# parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
# parser.add_argument("-i", "--input", type=str, help="Bag file to read")
# args = parser.parse_args()


try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        playback.pause()  # pasuing the playback to do heavy calculations

        # frames.get_depth_frame() is a 640x360 depth image
        # Align the depth frame to color frame

        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        #Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())


        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack(
            (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))


        cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example', images)

        key = cv2.waitKey(1)


        # ======================================= creating csv file for each frame. =====================================
        # this is done by writing rows. getting each row into 'row' adding it to rows array 'rows' and writing all rows.



        # field names
        row = []
        rows = []
        # filling the row.

        for y in range(480):
            for x in range(640):
                dist = aligned_depth_frame.get_distance(x, y)
                row.append(dist)
            rows.append(row)
            row = []

        # name of csv file
        filename = "csv{}.csv".format(numOfFrames)

        # writing to csv file
        with open(filename, 'w') as csvfile:
            # creating a csv writer object
            csvwriter = csv.writer(csvfile)

            # writing the data rows
            csvwriter.writerows(rows)


        cv2.imwrite("aligned{}.png".format(numOfFrames), bg_removed)

        # cv2.imwrite("depth{}.png".format(numOfFrames), depth_colormap)
        # cv2.imwrite("alignedAndDepth{}.png".format(numOfFrames), images)


        numOfFrames += 1
        playback.resume()  # resuming after heavy calculations.



finally:
    pipeline.stop()





