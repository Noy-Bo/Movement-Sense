import numpy as np
import glob
import cv2

img_array = []
counter = 1
for filename in glob.glob('C:\Users\\1\PycharmProjects\pythonProject3\*.png'):
    img = cv2.imread('C:\Users\\1\PycharmProjects\pythonProject3\plot{}.png'.format(counter))
    height, width, layers = img.shape
    size = (width, height)
    img_array.append(img)
    counter = counter+1

out = cv2.VideoWriter('project.avi', cv2.VideoWriter_fourcc(*'DIVX'), 17, size)

for i in range(len(img_array)):
    out.write(img_array[i])
out.release()