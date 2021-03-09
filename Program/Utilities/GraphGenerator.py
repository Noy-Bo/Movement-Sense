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





def GenerateGraph(openposeMeasurements,timestamps, viconMeasurements, calculation, orientation, path):
    # colors = cm.rainbow(np.linspace(0, 1, len(ys)))
    title = calculation + GetMeasurementType(calculation) + ' from ' + orientation + ' camera'
    fileName = orientation + '_' + calculation
    plt.title(title)
    plt.ylabel(GetMeasurementType(calculation))
    plt.xlabel('time (sec)')
    if openposeMeasurements is not None:
        plt.scatter(timestamps, openposeMeasurements, color='RED')
        fileName += '_Openpose'
    if viconMeasurements is not None:
        plt.scatter(timestamps, viconMeasurements, color='BLUE')
        fileName += '_Vicon'
    try:
        plt.savefig(path + fileName + '.pdf')
    except:
        print("Can't overwrite open file")
    plt.clf()
