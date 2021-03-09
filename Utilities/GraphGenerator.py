import matplotlib.pyplot as plt
import scipy
import scipy.stats
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


def GenerateGraphOfCorrelation(openposeMeasurements, viconMeasurements, name, path):
    # correlation parameters
    peterson = scipy.stats.pearsonr(openposeMeasurements, viconMeasurements)  # Pearson's r
    spearman = scipy.stats.spearmanr(openposeMeasurements, viconMeasurements)  # Spearman's rho
    kendall = scipy.stats.kendalltau(openposeMeasurements, viconMeasurements)  # Kendall's tau

    minVal = min(min(openposeMeasurements), min(viconMeasurements)) * 0.85
    maxVal = max(max(openposeMeasurements), max(viconMeasurements)) * 1.15

    # graph
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_ylabel("Vicon measurements")
    ax.set_xlabel("Openpose measurements")
    ax.scatter(openposeMeasurements, viconMeasurements, color='RED', s=3)
    ax.set_ylim([minVal, maxVal])
    ax.set_xlim([minVal, maxVal])
    ax.text(1, 1, ("Pearson's r: {} \n Spearman's rho: {} \n Kendall's tau: {} \n".format(str(peterson[0])[:-12],
                                                                                          str(spearman[0])[:-12],
                                                                                          str(kendall[0])[:-12])),
            horizontalalignment='right',
            verticalalignment='top',
            transform=ax.transAxes)
    # ax.plot([minVal, minVal], [maxVal, maxVal], transform=ax.transAxes)
    ax.plot(ax.get_xlim(), ax.get_ylim(), ls="--", c=".2")
    plt.savefig(path + "correlation_" + name + ".pdf")
    ax.clear()
    plt.clf()


def GenerateGraph(timestamps, openposeMeasurements, viconMeasurements, calculation, orientation, path):
    # colors = cm.rainbow(np.linspace(0, 1, len(ys)))
    title = calculation + GetMeasurementType(calculation) + ' from ' + orientation + ' camera'
    fileName = orientation + '_' + calculation
    plt.title(title)
    plt.ylabel(GetMeasurementType(calculation))
    plt.xlabel('time (sec)')
    if openposeMeasurements is not None:
        plt.scatter(timestamps, openposeMeasurements, color='RED', s=3)
        fileName += '_Openpose'
    if viconMeasurements is not None:
        plt.scatter(timestamps, viconMeasurements, color='BLUE', s=3)
        fileName += '_Vicon'
    try:
        plt.savefig(path + fileName + '.pdf')
    except:
        print("Can't overwrite open file")
    plt.clf()
