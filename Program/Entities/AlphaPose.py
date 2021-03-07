from Entities.Point3D import Point3D


class AlphaPoseSkeleton(object):
    def __init__(self, alpha):
        i = 0;
        self.nose = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lEye = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rEye = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lEar = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rEar = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lShoulder = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rShoulder = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lElbow = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rElbow = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lWrist = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rWrist = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lHip = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rHip = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lKnee = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rKnee = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lAnkle = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rAnkle = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.head = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.neck = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.hip = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lBigToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rBigToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lSmallToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rSmallToe = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.lHeel = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3
        self.rHeel = Point3D(alpha.keypoints[i], alpha.keypoints[i + 1], alpha.keypoints[i + 2])
        i += 3


# class for alphapose cloud
class AlphaPoseObject(object):
    def __init__(self, image_id, score, box, keypoints):
        self.image_id = image_id
        self.score = score
        self.box = box
        self.keypoints = keypoints

    def __hash__(self):
        return hash(self.image_id)

    def __eq__(self, other):
        if not isinstance(other, type(self)): return NotImplemented
        return self.image_id == other.image_id

    # object to string
    def __repr__(self):
        return self.image_id + " " + str(self.score)

    # function to add to global table
    def add(self):
        global skeletonsTable  # uses the global table declared in main, no idea why we need this
        if self in skeletonsTable:
            index = skeletonsTable.index(self)
            if self.score > skeletonsTable[index].score:
                skeletonsTable.pop(index)
                skeletonsTable.append(self)
        else:
            skeletonsTable.append(self)
