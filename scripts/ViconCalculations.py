from time import sleep

from pandas import np

import Algebra
import Vicon
import matplotlib.pyplot as plt

# WRB מפרק כף היד, הצד שבצד של הזרת
# WRA מפרק כף היד, צד של אגודל
# FIN הKNUCKLE של האצבע המורה
# ASI הנקודה הבולטת באגן מקדימה
# PSI השקע באגן מאחורה
# THI ירך
# KNE ברך
# TIB נקודה בין הברך לקרסול, על עצם הטיביה
# ANK קרסול (חיצוני)
# HEE עקב
# TOE מסביר את עצמו, לא? על הKNUCKLE מעל הבוה
# UPA זה בין הכתף למרפק
# בראש:
# FHD מצח (רקות)
# BHD נקודות באחורה של הראש
# C7 נקודה בולטת בצוואר
# T10 חולית T10, בערך באמצע הגב
# RBAK נקודה על השכמה הימנית, לא קבועה
# SHO כתף
# FRM זרוע
# ELB מרפק

# plot stuff
dataX = []
#dataX2 = []
dataY = []
#dataY2 = []
fig = plt.figure(num=None, figsize=(36, 10), dpi=80)
ax = fig.add_subplot(221)
#ax.set_ylabel("Angle (degrees)")
#ax.set_xlabel("Frame number")
#bx = fig.add_subplot(223)
# bx.set_ylabel("Butt to floor distance (Meter)")
# bx.set_xlabel("Time (Sec)")

# read vicon csv
viconSkeletons = Vicon.ViconReader(r'C:\Users\\1\PycharmProjects\pythonProject3\Sub002_Stand.csv')

for i in range(0,len(viconSkeletons)):

    # # knee angle
    # if (Algebra.isZero(viconSkeletons[i].RPSI) == False and Algebra.isZero(
    #         viconSkeletons[i].RKNE) == False and Algebra.isZero(viconSkeletons[i].RANK) == False):
    #
    #     hip = np.array([viconSkeletons[i].RPSI.x, viconSkeletons[i].RPSI.y, viconSkeletons[i].RPSI.z])
    #     knee = np.array([viconSkeletons[i].RKNE.x, viconSkeletons[i].RKNE.y, viconSkeletons[i].RKNE.z])
    #     ankle = np.array([viconSkeletons[i].RANK.x, viconSkeletons[i].RANK.y, viconSkeletons[i].RANK.z])
    #
    #     hipToKnee = Algebra.getVectorFrom2Points(hip, knee)
    #     KneeToAnkle = Algebra.getVectorFrom2Points(ankle, knee)
    #
    #     angle = Algebra.getAngle(hipToKnee, KneeToAnkle)
    #     # angle = 180 - angle
    #
    #     dataX.append(i)
    #     dataY.append(angle)
    #     print(angle)



    # kyphosis
    if (Algebra.isZero(viconSkeletons[i].RSHO) == False and Algebra.isZero(
            viconSkeletons[i].LSHO) == False and Algebra.isZero(viconSkeletons[i].CLAV) == False):

        viconSkeletons[i].LSHO.y = viconSkeletons[i].RSHO.y = viconSkeletons[i].CLAV.y

        lshoulder = np.array([viconSkeletons[i].LSHO.x, viconSkeletons[i].LSHO.y, viconSkeletons[i].LSHO.z])
        rshoulder = np.array([viconSkeletons[i].RSHO.x, viconSkeletons[i].RSHO.y, viconSkeletons[i].RSHO.z])
        clav = np.array([viconSkeletons[i].CLAV.x, viconSkeletons[i].CLAV.y, viconSkeletons[i].CLAV.z])

        rightShoulderToNeck = Algebra.getVectorFrom2Points(rshoulder, clav)
        leftShoulderToNeck = Algebra.getVectorFrom2Points(lshoulder, clav)

        viconSkeletons[i].LSHO.y = viconSkeletons[i].RSHO.y
        viconSkeletons[i].CLAV.y = viconSkeletons[i].RSHO.y

        angle = Algebra.getAngle(rightShoulderToNeck, leftShoulderToNeck)
        # angle = 180 - angle

        dataX.append(i)
        dataY.append(angle)
        print(angle)





Algebra.roundGraph(dataX, dataY, ax, "Frame number", "Angle (degrees)", 200)
#Algebra.roundGraph(dataX2, dataY2, bx, "Angle (degrees)", "Knee angle (degrees)", 200)
plt.savefig("shoulders kyphosis (VICON).pdf")

sleep(1)
plt.close(fig)