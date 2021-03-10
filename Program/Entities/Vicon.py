from Entities.Point3D import Point3D


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