# Simultaneous Localization and Mapping (SLAM)

SLAM comprises the simultaneous estimation of the state
of a robot equipped with on-board sensors, and the construction of a model (the map) of the environment that the sensors are perceiving[1]. In this project the location of a quadcopter and AprilTags are found by solving the factor graph[2] formalization of SLAM[3].


## About Code

A set of AprilTags of known dimension are placed on the ground. The location of the tags and their ID is unknown except for Tag 10, whose bottom left corner is chosen as the origin. The location of all the tags and the motion of quadcopter are to be found based on this origin. A quadcopter is being flown in a pattern so that it can see a subset of the tags at any given point in time. Using the images captured from a calibrated camera mounted on the quadcopter, the image coordinates of the subset of tags seen are found and their IDs are identified. With the image coordinates of the tags and the calibration parameters (arranged in a matrix) of the camera, the pose (location w.r.t the origin) is calculated using RPnP algorithm. With the pose of the quadcopter and the calibration matrix, the coordinates (w.r.t origin) of the remaining (previously unidentified) tags detected by the camera are calculated. This procedure is repeated for each image taken by the camera. A factor graph is then constructed with the set of poses of the quadcopter and the set of locations of the tags that is detected in each image. The graph is solved using a trust-region optimizer: Dogleg.

**_See ProjectOutline.pdf in "/docs" folder for a detailed description of the data and the project._**


## How to Run?

Add to MATLAB path the "src", "libraries" and "data" folders. Make sure to download and install GTSAM package from https://borg.cc.gatech.edu/download.html. 

**1.0**

Goto /src and run "Wrapper.m". If you want to view the results for the different flight patterns of the quadcopter, change the "DataSquare.mat" in the Wrapper.m to any of the files in "/Data" folder.

You may have to change the noise sigmas in lines 68 to 70 in SLAMunsingGTSAM.m to get reasonable results. 

Note: You might get an error for different flight patterns, if you run the same code. It's because the graph couldn't be solved with the same noise sigmas for all the scenarios. This shouldn't be the case as the noise is typically dependent on the sensor and it's accompanying electronics. However, there was no information given as to how the data was collected, and there was no information given about the accuracy of the detection of tags. I chose the noise sigmas based on trial and error for a particular flight pattern and found that it didn't wok for all the flight patterns. However when I changed the noise sigmas, I got results for some flight patterns that previously gave an error.


## References

[1] C. Cadena and L. Carlone and H. Carrillo and Y. Latif and D. Scaramuzza and J. Neira and I. Reid and J.J. Leonard, “Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age”, in IEEE Transactions on Robotics 32 (6) pp 1309-1332, 2016.

[2] Factor Graph Tutorial, HU, Pili, Feb 29, 2012.

[3] Factor Graphs for Robot Perception, Foundations and Trends in Robotics, Vol. 6, No. 1-2 (2017) 1–139, 2017 F. Dellaert and M. Kaess, DOI: 10.1561/2300000043
