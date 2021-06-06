# Movement Sense

Image Vision Project at Geometric Image Processing Lab (GIP) – Technion.
###
Body posture is one of the fundamental indicators for evaluating health and quality of life.
As a part of a research that aims to detect and analyze human posture, our goal is to be able to
extract accurate data using photogrammetry tools. The data, collected via Intel depth cameras,
consists of subjects standing in various poses and performing sport exercises, analyzed using pose
estimation algorithms with heuristics we designed.
In order to validate our results, data were collected from the Vicon system which is regarded
as the “gold standard” in assessing human movement, we referred to it as ground truth. The results
calculated by the Realsense were compared with those obtained using the Vicon system. Correlation
measurements show that there is a strong correlation between our results and the ground truth.
Our work aims to provide precise data of human posture. In addition, we provide an
accessible-to-all tool to automate the analyzation and validation process of the data collected by
Realsense and Vicon systems.

## 
* Compared & tested AI pose estimation algorithms.
* Built a working pipeline, including: data collection, skeleton extraction, data calculation and results display.
* Comparing extracted data with Vicon(current kinematics Gold-Standart) data.
* Accessable-to-all GUI tool to automate the extraction, analyzation and validation process.
##
### Technologies:
OpenCV, Tkinter, Cubemos – skeleton tracking SDK, OpenPose - pose estimation, AlphaPose - pose estimation, Intel Realsense Depth Camera D415 / D435, Intel Realsense SDK.
##
### Visualization

<p float="center">
  <img src="https://github.com/Noy-Bo/Image-Vision-Project/blob/main/readme/project_30.gif" alt="alt text">
  
</p>
