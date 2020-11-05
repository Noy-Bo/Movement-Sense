## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

################################################################################
##       script to align frames to color and save them with timestamps        ##
##           also generating list with images name for alphapsoe              ##
################################################################################

import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# Setup:
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_device_from_file("C:\Age_Estimation_Project\\bag_files\Eliran_Squat_1.5_Front.bag",False)

profile = pipeline.start(cfg)
device = profile.get_device()
playback = device.as_playback() # this allows the use of pause and resume.

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
timestamps = []
f = open("list.txt", "w")


try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()

        playback.pause()  # pausing the playback to do heavy calculations

        # frames.get_depth_frame() is a 640x360 depth image
        # Align the depth frame to color frame

        aligned_frames = align.process(frames)

        aligned_color_frame = aligned_frames.get_color_frame()

        color_timestamp = aligned_color_frame.timestamp
        color_timestamp_str = str(color_timestamp)

        #checking if we already have this frame.
        if (color_timestamp_str not in timestamps):
            timestamps.append(color_timestamp_str)



            # # Remove background - Set pixels further than clipping_distance to grey
            # grey_color = 153
            # depth_image_3d = np.dstack(
            #     (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
            # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            #
            # # Render images
            #
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # images = np.hstack((bg_removed, depth_colormap))

            key = cv2.waitKey(1)

            #writing image & list
            color_image = np.asanyarray(aligned_color_frame.get_data())
            cv2.imwrite("{}.png".format(color_timestamp_str), color_image)
            f = open("list.txt", "a")
            f.write("{}.png\n".format(color_timestamp_str))
            numOfFrames += 1
        playback.resume()  # resuming after heavy calculations.



finally:

    pipeline.stop()





