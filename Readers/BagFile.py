import argparse
import json
import math

import cv2
import numpy as np
from scipy import ndimage

from Calculations.Algebra import isZero, getDistance
from Entities.OpenPose import OpenPoseSkeleton
from Entities.Point3D import Point3D
from Entities.Timestamp import Timestamp
from Readers.OpenPose import OpenPoseReader
import pyrealsense2 as rs
import time

from Calculations import Algebra


def BagFileSetup(path, orientation,rotationAngle):  # path = '.../', orientation = 'front'
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
    skeletonsTable = OpenPoseReader(path, orientation)
    # ===========

    # open and generate the log as timestamps
    log_list = OpenLogFile(path, orientation)

    # Setup:
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device_from_file(path + orientation + '.bag', False)

    profile = pipeline.start(cfg)
    device = profile.get_device()
    playback = device.as_playback()  # this allows the use of pause and resume.

    # pausing playback to let the pipe warm up
    playback.pause

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 2.2  # 1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # field names
    first = True
    isRotated = False

    data_timestamps = []
    data_skeletons = []
    startingTimeStamp = 0
    currentTimeStamp = 0
    lastTimeStamp = -1

    numOfFrames = 0
    all_depth_timestamps = []

    # Streaming loop
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--directory", type=str, help="Path to save the images")
    parser.add_argument("-i", "--input", type=str, help="Bag file to read")
    args = parser.parse_args()

    # letting the pipe warm up for couple of seconds then starting to capture frames
    playback.resume()
    time.sleep(1)
    loop = 0
    try:
        while True:
            loop += 1
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
            if first:
                first = False
                startingTimeStamp = math.floor(float(aligned_depth_frame.timestamp) / 10)
                Algebra.max_height_resulotion = aligned_depth_frame.height
                Algebra.max_width_resulotion = aligned_depth_frame.width

            # TimeStamps
            depth_timestamp = aligned_depth_frame.timestamp
            # checking if we already read this frame
            frameTimestamp = math.floor(float(depth_timestamp) / 10)
            if frameTimestamp not in all_depth_timestamps:
                all_depth_timestamps.append(frameTimestamp)
                # marks code to valid the frame in log file
                color_frame_name = None
                for item in log_list:  # searching for color timestamp of corresponding color frame
                    # if item.depth_timestamp == str(depth_timestamp):  # python 2.7: change to item.depth_timestamp == str(depth_timestamp)
                    # logTimestamp = str(math.floor(float(item.depth_timestamp)))
                    # frameTimestamp = str(round(float(depth_timestamp)))
                    # if str(math.floor(float(item.depth_timestamp))) == str(round(float(depth_timestamp))):  # python 2.7: change to item.depth_timestamp == str(depth_timestamp)
                    logTimestamp = math.floor(float(item.depth_timestamp) / 10)
                    # frameTimestamp = math.floor(float(depth_timestamp) / 10)
                    #if logTimestamp == frameTimestamp: # for e stamp
                    if float(item.depth_timestamp) == depth_timestamp:
                        color_frame_name = item.color_timestamp + ".png"
                        break

                # finding color's skeletonObject
                skeletonObject = None
                for item in skeletonsTable:  # searching for the full corresponding alphapose cloud object
                    if item.image_id == color_frame_name:
                        skeletonObject = item
                        currentTimeStamp = float(item.image_id[:-4])
                        break
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
                    verts = np.asarray(points.get_vertices(2)).reshape(h,w, 3)
                    verts = ndimage.rotate(verts, rotationAngle)
                    color_image_copy = ndimage.rotate(color_image.copy(), rotationAngle)
                    isRotated = True  # dont forget to disable this in cases u dont rotate

                    # print("rawsize: {}, vertsSize: {}".format(vertsRaw.size, verts.size))
                    texcoords = np.asarray(points.get_texture_coordinates(2))

                    # writing 3d points inside alphapose object
                    # ==================== AlphaPose
                    # pixelSkeleton = AlphaPoseSkeleton(skeletonObject)
                    # ==================== OpenPose
                    pixelSkeleton = OpenPoseSkeleton(skeletonObject.keypoints)


                    #printing points test
                    #color_image_copy = color_image.copy()
                    cv2.circle(color_image_copy, (int(pixelSkeleton.rShoulder.x),int(pixelSkeleton.rShoulder.y)),4,(255,0,0),-1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.lShoulder.x),int(pixelSkeleton.lShoulder.y)),4,(0,255,0),-1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.rKnee.x), int(pixelSkeleton.rKnee.y)), 4,
                            (255, 0, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.lKnee.x), int(pixelSkeleton.lKnee.y)), 4,
                            (0, 255, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.lHip.x), int(pixelSkeleton.lHip.y)), 4,
                            (0, 255, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.rHip.x), int(pixelSkeleton.rHip.y)), 4,
                            (255, 0, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.lAnkle.x), int(pixelSkeleton.lAnkle.y)), 4,
                            (0, 255, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.rAnkle.x), int(pixelSkeleton.rAnkle.y)), 4,
                            (255, 0, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.lElbow.x), int(pixelSkeleton.lElbow.y)), 4,
                            (0, 255, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.rElbow.x), int(pixelSkeleton.rElbow.y)), 4,
                            (255, 0, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.neck.x), int(pixelSkeleton.neck.y)), 4,
                            (0, 0, 255), -1)
                    # cv2.circle(color_image_copy, (int(pixelSkeleton.midHip.x), int(pixelSkeleton.midHip.y)), 4,
                    #           (0, 0, 255), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.lShoulder.x), int(pixelSkeleton.lShoulder.y)), 4,
                            (0, 255, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.rWrist.x), int(pixelSkeleton.rWrist.y)), 4,
                            (255, 0, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.lWrist.x), int(pixelSkeleton.lWrist.y)), 4,
                            (0, 255, 0), -1)
                    cv2.circle(color_image_copy, (int(pixelSkeleton.neck.x),int(pixelSkeleton.neck.y)),4,(0,0,255),-1)
                    cv2.putText(color_image_copy, "frame no. "+str(numOfFrames), (int(h/2), 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.imshow(orientation + " bag file" ,color_image_copy)
                    cv2.waitKey(1)


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
                            if Algebra.outOfBoundries(pixel_x, pixel_y, isRotated):
                                skeletonObject.keypoints[i] = 0
                                skeletonObject.keypoints[i + 1] = 0
                                skeletonObject.keypoints[i + 2] = 0
                            else:
                                erodedSet = Algebra.eroding(pixel_y, pixel_x, aligned_depth_frame.height,
                                                       aligned_depth_frame.width, isRotated)
                                collect = []
                                for item in erodedSet:
                                    xyz = verts[item.x, item.y]
                                    collect.append(Point3D(xyz[0], xyz[1], xyz[2]))
                                xyz = verts[pixel_y, pixel_x]
                                point = Algebra.rounded(xyz[2], collect)
                                # xyz = verts[pixel_y, pixel_x]
                                skeletonObject.keypoints[i] = point.x
                                skeletonObject.keypoints[i + 1] = point.y
                                skeletonObject.keypoints[i + 2] = point.z

                    # ALPHA POSE
                    # skeleton = AlphaPoseSkeleton(skeletonObject)
                    # OPEN POSE
                    skeleton = OpenPoseSkeleton(skeletonObject.keypoints)

                    # test, deletable
                    # if loop > 100:
                    #     if (isZero(skeleton.rElbow) == False and isZero(
                    #             skeleton.rKnee) == False):
                    #         dist = getDistance(skeleton.rElbow, skeleton.rKnee)
                    #         #print(dist)
                    numOfFrames += 1
                    lastTimeStamp = frameTimestamp

                    data_skeletons.append(skeleton)
                    data_timestamps.append((lastTimeStamp - startingTimeStamp) / 100)

                # Algebra.roundGraph(dataX,dataY,ax)

            #print(str(loop) + 'th frame')
            # resuming after heavy calculations.
            playback.resume()


    finally:
        pipeline.stop()
        # save last state of plot
        return data_skeletons, data_timestamps


def OpenLogFile(path, orientation):
    # open's the log as json array.
    log_file = open(path + orientation + "_log.json")
    log_array = json.load(log_file)
    log_list = []
    # generating the log_list
    for item in log_array:
        log_list.append(
            Timestamp(item['color_timestamp'],
                      item['depth_timestamp']))  # json_list contains all pairs of depth & color
    return log_list