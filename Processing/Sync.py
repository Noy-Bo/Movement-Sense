import Calculations.Algebra

from Calculations.Algebra import isZero, getDistance, takeClosest

factor_forward = 1.4
factor_backward = 1.3


def SyncByMovementOpenpose(openPoseSkeletons, timestamps): #TODO return stable frames after the movement.
    # openpose
    a = []
    firstDistance = None
    idx = None
    passedForwardCheck = False
    for i in range(0, len(openPoseSkeletons)):

        dist = CalcDistance(openPoseSkeletons[i], "openpose")

        if dist is not None:
            a.append(dist)
            if firstDistance is None:
                firstDistance = dist

            if dist > (firstDistance * factor_forward) and dist < (firstDistance *3):
                passedForwardCheck = True

            if passedForwardCheck is True and dist < (firstDistance * factor_backward):
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
    firstDistance = None
    idx = None
    passedForwardCheck = False
    for i in range(0, len(viconSkeletons)):

        dist = CalcDistance(viconSkeletons[i], "vicon")

        if dist is not None:

            if firstDistance is None:
                firstDistance = dist

            if dist > (firstDistance * factor_forward):
                passedForwardCheck = True

            if passedForwardCheck is True and dist < (firstDistance * factor_backward):
                idx = i
                break

    viconSkeletons = viconSkeletons[idx:len(viconSkeletons)]    # TODO: EXCEPTION
    return viconSkeletons


def CalcDistance(skeleton, type):
    dist = None

    if "openpose" in type:
        if (isZero(skeleton.rElbow) == False and isZero(
                skeleton.rKnee) == False):
            dist = getDistance(skeleton.rElbow, skeleton.rKnee)

    elif "vicon" in type:
        if ((isZero(skeleton.RELB) is False) and (isZero(skeleton.RKNE) is False)):
            dist = getDistance(skeleton.RELB, skeleton.RKNE)

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

def removeOutliers(cutViconData,openposeData,correspondingTimestamps):

    outliersIndexes = []
    for i in range(len(openposeData)):
        if openposeData[i] == 0 or  openposeData[i] == -1 or openposeData[i] == -2 or cutViconData[i] == 0 or cutViconData[i] == -1 or cutViconData[i] == -2:
            outliersIndexes.append(i)

    for i in range(len(outliersIndexes)):
        del cutViconData[outliersIndexes[i]-i]
        del openposeData[outliersIndexes[i]-i]
        del correspondingTimestamps[outliersIndexes[i]-i]

    return cutViconData,openposeData,correspondingTimestamps