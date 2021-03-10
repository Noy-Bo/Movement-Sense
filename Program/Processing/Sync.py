import Calculations.Algebra

from Calculations.Algebra import isZero, getDistance, takeClosest

factor = 0.7


def SyncByMovementOpenpose(openPoseSkeletons, timestamps): #TODO return stable frames after the movement.
    # openpose
    first = None
    idx = None
    for i in range(0, len(openPoseSkeletons)):

        dist = CalcDistance(openPoseSkeletons[i], "openpose")

        if dist is not None:

            if first is None:
                first = dist

            if dist < (first * factor):
                idx = i
                break

    openPoseSkeletons = openPoseSkeletons[idx:len(openPoseSkeletons)]    # TODO: EXCEPTION
    timestamps = timestamps[i:len(timestamps)]
    # timestamps refactoring
    first = timestamps[0]
    for i in range(0, len(timestamps)):
        timestamps[i] -= first
    return openPoseSkeletons, timestamps


def SyncByMovementVicon(viconSkeletons):
    # vicon
    first = None
    idx = None
    for i in range(0, len(viconSkeletons)):

        dist = CalcDistance(viconSkeletons[i], "vicon")

        if dist is not None:

            if first is None:
                first = dist

            if dist < (first * factor):
                idx = i
                break

    viconSkeletons = viconSkeletons[idx:len(viconSkeletons)]    # TODO: EXCEPTION
    return viconSkeletons


def CalcDistance(skeleton, type):
    dist = None

    if "openpose" in type:
        if (isZero(skeleton.neck) == False and isZero(
                skeleton.rKnee) == False):
            dist = getDistance(skeleton.neck, skeleton.rKnee)

    elif "vicon" in type:
        if ((isZero(skeleton.RKNE) is False) and (isZero(skeleton.CLAV) is False)):
            dist = getDistance(skeleton.RKNE, skeleton.CLAV)

    return dist


def matchTimestamps(openposeTimestamps, viconMeasurements):

    viconTimestamps = []
    cutViconMeasurements = []
    cutViconTimestamps = []

    # generate vicon timestamps
    for i in range(0, len(viconMeasurements)):
        viconTimestamps.append(float(float(i)/120))

    # for each op-timestamp we take closest timestamp from vicon-timestamps
    for idx in range(0, len(openposeTimestamps)):
        vicon_timestamp = takeClosest(openposeTimestamps[idx], viconTimestamps)
        for vIdx in range(0, len(viconTimestamps)):
            if vicon_timestamp == viconTimestamps[vIdx]:
                cutViconTimestamps.append(viconTimestamps[vIdx])
                cutViconMeasurements.append(viconMeasurements[vIdx])

    return cutViconMeasurements