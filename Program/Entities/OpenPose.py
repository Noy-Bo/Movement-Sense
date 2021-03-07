from Entities.Point3D import Point3D


class OpenPoseSkeleton(object):
    def __init__(self, keypoints):
        i = 0
        self.nose = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.neck = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rShoulder = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rElbow = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rWrist = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lShoulder = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lElbow = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lWrist = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.midHip = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rHip = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rKnee = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rAnkle = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lHip = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lKnee = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lAnkle = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rEye = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lEye = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rEar = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lEar = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lBigToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lSmallToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.lHeel = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rBigToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rSmallToe = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])
        i += 3
        self.rHeel = Point3D(keypoints[i], keypoints[i + 1], keypoints[i + 2])

class OpenPoseObject(object):
    def __init__(self, image_id, keypoints):
        self.image_id = image_id + ".png"
        self.keypoints = keypoints

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.image_id == other.image_id

    def __hash__(self):
        return self.image_id.__hash__()

    # object to string
    def __repr__(self):
        return "image id: " + str(self.image_id)

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.image_id == other.image_id

    def __hash__(self):
        return (self.image_id).__hash__()