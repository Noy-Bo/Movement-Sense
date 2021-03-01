## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

################################################################################
##              script to export csv and aligned png from bag file            ##
################################################################################

import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import csv

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
row = []
rows = []
timestamp = 0
maxTimestamp = 0
firstFrame = True
totalIntervals = 0

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




        # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ testing cloud



        # # Intrinsics & Extrinsics
        # depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        # color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        # depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
        # color_to_depth_extrin = color_frame.profile.get_extrinsics_to(aligned_depth_frame.profile)
        #
        # print "\n Depth intrinsics: " + str(depth_intrin)
        # print "\n Color intrinsics: " + str(color_intrin)
        # print "\n Depth to color extrinsics: " + str(depth_to_color_extrin)
        #
        # # Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
        # depth_sensor = device.first_depth_sensor()
        # depth_scale = depth_sensor.get_depth_scale()
        # print depth_scale
        # # Map depth to color
        # depth_pixel = [200, 200]  # Random pixel
        # print "\n\t depth_pixel: " + str(depth_pixel)
        # depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_scale)
        # color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
        # color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
        # print "\n\t color_pixel: " + str(color_pixel)
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(color_frame.profile)
        depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_scale)
        color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)

        # log interactions
        log = open(logfile_name, "r")
        num_lines = sum(1 for line in open(logfile_name))

        depth_timestamp_string = "depth timestamp: {}\n".format(depth_timestamp)

        for i in range(0, num_lines):
            if (log.readline() == depth_timestamp_string and depth_timestamp not in all_depth_timestamps):

                all_depth_timestamps.append(depth_timestamp)
                correspondent_aligned_color_frame = log.readlines()[i:i + 1]  # this is relative position to i

                # leaving only color timestamp

                if correspondent_aligned_color_frame[0].startswith("color timestamp: "):
                    correspondent_aligned_color_frame[0] = correspondent_aligned_color_frame[0][
                                                           len("color timestamp: "):]

                if correspondent_aligned_color_frame[0].endswith("\n"):
                    correspondent_aligned_color_frame[0] = correspondent_aligned_color_frame[0][:-1]

                correspondent_aligned_color_frame[0] = str(correspondent_aligned_color_frame[0]) + ".png"

        # Generating 3d points from aligned_depth

        # Get height / width configurations.
        w, h = aligned_depth_frame.width, aligned_depth_frame.height

        pc = rs.pointcloud()
        points = pc.calculate(aligned_depth_frame)
        pc.map_to(color_frame)

        # Create a VertexList to hold pointcloud data
        # Will pre-allocates memory according to the attributes below
        # vertex_list = pyglet.graphics.vertex_list(
        #     w * h, 'v3f/stream', 't2f/stream', 'n3f/stream')

        points = pc.calculate(aligned_depth_frame)
        verts = np.asarray(points.get_vertices(2)).reshape(h, w, 3)
        #print("rawsize: {}, vertsSize: {}".format(vertsRaw.size, verts.size))
        texcoords = np.asarray(points.get_texture_coordinates(2))
        # for i in range(400):
        #     print ("Verts: {}".format(verts[i]))
        #     print("texoord: {}".format(texcoords[i]))
        # print(verts)
        # if len(vertex_list.vertices) != verts.size:
        #     vertex_list.resize(verts.size // 3)
        #     # need to reassign after resizing
        #     vertex_list.vertices = verts.ravel()
        #     vertex_list.tex_coords = texcoords.ravel()


        #@@@@@@@@@@@@@@TimeStamps

        # timestamps data to determin fps
        data = color_frame.get_data

        # removing frames that we already have
        if (data.im_self.timestamp != timestamp):

            if (firstFrame == True):
                firstFrame = False
            else:
                maxTimestamp = max(maxTimestamp, data.im_self.timestamp - timestamp)
                interval = data.im_self.timestamp - timestamp
                print("current timestamp interval  {} -  {} = {}".format(data.im_self.timestamp,timestamp,interval))
                print("max interval is {}".format(maxTimestamp))
                if (numOfFrames%50 == 0):
                    print("@@@@@@@@@@@@@@@@@@@@@@@")
                    print("numer of frames {}".format(numOfFrames))
                    print("total timestamps intervals {}".format(totalIntervals))
                    print("avg fps {}".format(totalIntervals / numOfFrames))
                totalIntervals += interval

            timestamp = data.im_self.timestamp




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



            #cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            #cv2.imshow('Align Example', images)

            key = cv2.waitKey(1)


            # ======================================= creating csv file for each frame. =====================================
            # this is done by writing rows. getting each row into 'row' adding it to rows array 'rows' and writing all rows.

            # field names
            row = []
            rows = []
            # filling the row.
            #for y in range(719,-1,-1):
            for y in range (720):
                for x in range(1280):
                    #dist = "x:{} , y:{}".format(x,y)
                    depth_pixel = [x,y]
                    dist = aligned_depth_frame.get_distance(x, y)
                    if dist == 0:
                        row.append("0")
                    else:
                        xyz = verts[y,x]
                        results = "({},{},{})".format(xyz[0],xyz[1],xyz[2])
                        row.append(results)
                rows.append(row)
                row = []

            # name of csv file
            filename = "csv{}.csv".format(numOfFrames)


            # writing to csv file
            with open(filename, 'wb') as csvfile:
                # creating a csv writer object
                csvwriter = csv.writer(csvfile)

                # writing the data rows
                csvwriter.writerows(rows)


            cv2.imwrite("aligned{}.png".format(numOfFrames), bg_removed)

            #cv2.imwrite("depth{}.png".format(numOfFrames), depth_colormap)
            # cv2.imwrite("alignedAndDepth{}.png".format(numOfFrames), images)


            numOfFrames += 1
        playback.resume()  # resuming after heavy calculations.



finally:
    print ("total intervals".format(totalIntervals))
    print("number of frames".format(numOfFrames))
    pipeline.stop()





