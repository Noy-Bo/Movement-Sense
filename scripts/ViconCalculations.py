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
fig = plt.figure(num=None, figsize=(60, 10), dpi=80)
ax = fig.add_subplot(221)
#ax.set_ylabel("Angle (degrees)")
#ax.set_xlabel("Frame number")
#bx = fig.add_subplot(223)
# bx.set_ylabel("Butt to floor distance (Meter)")
# bx.set_xlabel("Time (Sec)")

# read vicon csv
viconSkeletons = Vicon.ViconReader(r'C:\Users\\1\PycharmProjects\pythonProject3\Sub002_Squat.csv')

for i in range(0,len(viconSkeletons)):

    if (Algebra.isZero(viconSkeletons[i].RPSI) == False and Algebra.isZero(
            viconSkeletons[i].RKNE) == False and Algebra.isZero(viconSkeletons[i].RANK) == False):
        normalizeHipToKnee = Algebra.getNormalizeVector(viconSkeletons[i].RPSI, viconSkeletons[i].RKNE)
        normalizeKneeToAnkle = Algebra.getNormalizeVector(viconSkeletons[i].RANK, viconSkeletons[i].RKNE)
        if (Algebra.isZero(normalizeHipToKnee) == False and Algebra.isZero(normalizeKneeToAnkle) == False):
            angle = Algebra.getAngle(normalizeHipToKnee, normalizeKneeToAnkle)
        else:
            angle = -1

        print(angle)

        dataX.append(i)
        dataY.append(angle)

Algebra.roundGraph(dataX, dataY, ax, "Frame number", "Angle (degrees)", 200)
#Algebra.roundGraph(dataX2, dataY2, bx, "Angle (degrees)", "Knee angle (degrees)", 200)
plt.savefig("plotNormal.pdf")
input()