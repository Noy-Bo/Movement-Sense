import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np

def GetMeasurementType(calculation):
    ylabel = None
    if calculation is 'Knees':
        ylabel = 'Angle (degrees)'
    elif calculation is 'Kyphosis':
        ylabel = 'Angle (degrees)'
    elif calculation is 'Mass':
        ylabel = 'Variance'
    return ylabel


def GenerateGraphOfCorrelation():


def GenerateGraph(timestamps, openposeMeasurements, viconMeasurements, calculation, orientation):
    colors = cm.rainbow(np.linspace(0, 1, len(ys)))
    fig = plt.figure(num=None, figsize=(18, 10))
    # ax = fig.add_subplot(111)
    plt.title(calculation + GetMeasurementType(calculation) + ' from ' + orientation + ' camera')
    plt.set_ylabel(GetMeasurementType(calculation))
    plt.set_xlabel('time (sec)')
    for i in range(len(timestamps)):
        plt.scatter(timestamps[i], measurements[i], color=rnd)
    plt.savefig(orientation + '_' + calculation + '.pdf')
