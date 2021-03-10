import glob
import json

from Entities.OpenPose import OpenPoseObject


def OpenPoseReader(path, orientation):
    # final objects list
    openpose_list = []
    # original path to files
    # raw_filenames = glob.glob('*openpose/*.json')
    raw_filenames = glob.glob(path + orientation + '_openpose/*.json')
    # constructing filenames
    for item in raw_filenames:
        # open json file
        with open(item) as json_file:
            data = json.load(json_file)
        name_temp = item.replace(path + orientation + '_openpose\\', '')
        # get image id name
        image_id = name_temp.replace('_keypoints.json', '')
        # get keypoints
        if len(data['people']) > 0:
            keypoints = data['people'][0]['pose_keypoints_2d']
            openpose_list.append(OpenPoseObject(image_id, keypoints))
    return openpose_list