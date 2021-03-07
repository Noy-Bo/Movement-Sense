from Algebra import Point3D
import csv


class ViconObject(object):
    def __init__(self, keypoints):
        self.frame = keypoints[0]
        i = 2
        self.LFHD = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RFHD = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LBHD = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RBHD = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.C7 = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.T10 = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.CLAV = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.STRN = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RBAK = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LSHO = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LUPA = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LELB = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LFRM = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LWRA = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LWRB = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LFIN = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RSHO = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RUPA = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RELB = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RFRM = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RWRA = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RWRB = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RFIN = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LASI = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RASI = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LPSI = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RPSI = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LTHI = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LKNE = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LTIB = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LANK = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LHEE = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.LTOE = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RTHI = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RKNE = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RTIB = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RANK = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RHEE = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.RTOE = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.frame == other.frame

    def __hash__(self):
        return self.frame.__hash__()

    # object to string
    def __repr__(self):
        return "frame number: " + str(self.frame)

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return (self.x, self.y).__hash__()
        # return hash(self.x,self.y)


def ViconReader(filename):
    points = []
    keypoints = []
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


# test
# viconSkeletons = ViconReader(r'C:\Users\\1\PycharmProjects\pythonProject3\Sub002_Squat.csv')
# print("test")


