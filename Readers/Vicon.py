import csv

from Entities.Vicon import ViconObject


def ViconReader(filename):
    vicon_list = []
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 1
        flag = False
        for row in csv_reader:
            if flag is True:
                points = []
                keypoints = []
                points.append(row)
                for sublist in points:
                    for item in sublist:
                        if item != '':
                            keypoints.append(float(item))
                        else:
                            keypoints.append(0)
                if len(keypoints) != 0:
                    vicon_list.append(ViconObject(keypoints))
            if 'Trajectories' in row:
                next(csv_reader)
                next(csv_reader)
                next(csv_reader)
                next(csv_reader)
                flag = True
            line_count += 1
        return vicon_list