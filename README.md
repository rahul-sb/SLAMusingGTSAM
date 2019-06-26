# Simultaneous Localization and Mapping (SLAM)

SLAM comprises the simultaneous estimation of the state
of a robot equipped with onboard sensors, and the construction of a model (the map) of the environment that the sensors are perceiving[1]. In this project, the location of a quadcopter and AprilTags are found by solving the factor graph[2] formalization of SLAM[3].


## About Code

A set of AprilTags of known dimension is placed on the ground. The location of the tags and their ID is unknown except for Tag 10, whose bottom left corner is chosen as the origin. The location of all the tags and the motion of quadcopter are to be found based on this origin. A quadcopter is being flown in a pattern so that it can see a subset of the tags at any given point in time. Using the images captured from a calibrated camera mounted on the quadcopter, the image coordinates of the subset of tags seen are found, and their IDs are identified. With the image coordinates of the tags and the calibration parameters (arranged in a matrix) of the camera, the pose (orientation and location w.r.t the origin) is calculated using RPnP algorithm. With the pose of the quadcopter and the calibration matrix, the coordinates (w.r.t origin) of the remaining (previously unidentified) tags detected by the camera are calculated. This procedure is repeated for each image taken by the camera. A factor graph is then constructed with the set of poses of the quadcopter and the set of locations of the tags that are detected in each image. The graph is solved using a trust-region optimizer: Dogleg.

**_See ProjectOutline.pdf in "/docs" folder for a detailed description of the data and the project._**


## How to Run?

Add to MATLAB path the "src", "libraries" and "data" folders. Make sure to download and install GTSAM package from https://borg.cc.gatech.edu/download.html. 

**Release 1.0**

Goto /src and run "Wrapper.m". If you want to view the results for the different flight patterns of the quadcopter, change the "DataSquare.mat" in the Wrapper.m to any of the files in "/Data" folder.

You may have to change the noise sigmas in lines 68 to 70 in SLAMunsingGTSAM.m to get consistent results. 

**Release 2.0**

Goto /src and run "visualSLAM_main.m". If you want to check the results for different paths that the quadcopter took, change the "DataSquare.mat" in the "visualSLAM_main.m" to any of the files in /Data. 

In **2.0** I made an abstraction over the GTSAM toolbox and arranged the relevant parts of the code in v1 into different functions so that it is a little easier to read and get a better idea of the overall structure of the program. 

You may have to change the noise sigmas in lines 6 to 8 in "visualSLAM_main.m" to get consistent results. 

Note: You might get an error for different flight patterns if you run the same code. It's because the graph couldn't be solved with the same noise sigmas for all the scenarios. This shouldn't be the case as the noise is typically dependent on the sensor and it's accompanying electronics. However, there was no information given as to how the data was collected, and there was no information provided about the accuracy of the detection of tags. I chose the noise sigmas based on trial and error for a particular flight pattern and found that it didn't work for all the flight patterns. However, when I changed the noise sigmas, I got results for some flight patterns that previously gave an error.


## Results

Side View from DataMapping dataset
![Side view of Result from DataMapping](https://drive.google.com/uc?export=view&id=1FUsxQDNVCCNPayYACJnE6xbJZk5DQaeZ)

Top View from DataMapping dataset
![Side view of Result from DataMapping](https://drive.google.com/uc?export=view&id=1VRtdg9xIld4bH2ADVM27WGWV6DxUIHf7)

The green \* represents the corners of the tags. The blue \* represents the location of the quadcopter. The blue line that connects two blue \* represents the translation of quadcopter from one state to another. The three perpendicular lines on the blue \* represent the orientation of the quadcopter.

*To have a look at results from other flight patterns have a look at the ProjectReport.pdf in the "/Results" folder.*


## How does SLAM work?


TL;DR: Wait for a blog post at the end of August. Until then, see references.

It can be challenging to understand SLAM, especially if you are learning it for the first time. I plan to write a detailed blog post on the history of SLAM, an explanation for the math that goes behind it, the reasoning behind the formulation of SLAM based on the math that might have gone while these algorithms were being developed. 

I also plan on to write a series of posts introducing readers to the field of robotics. When I was introduced into the field of robotics, there were many equations describing kinematics and dynamics, and the teachers gave examples of the Industrial manipulators, and we (students) derive equations for them and formulate control logic to control them. I want to introduce readers into robotics from a different viewpoint, from the electronics side, which I find very interesting. I'll describe the limitations/problems of the devices (sensors) that are typically used in robotics, how it was/is currently being solved, the resulting repercussions and then eventually building the explanation towards today's state-of-the-art algorithms in Machine Learning and Artificial Intelligence. These blog posts will be designed for a high-school graduate. I'll do my best to explain these concepts and the accompanying math in an easy to digest manner and also give pointers to resources where the reader can dig into the topics much more in-depth. I'm planning to finish a few posts by the end of August 2019 (might change depending on current job search), and from then onwards I plan to post a blog every week. I'll make sure to update the timeline and schedule here if there are any changes.  Until then, if you want to learn more about SLAM, have a look at the list of references.



## References

[1] C. Cadena and L. Carlone and H. Carrillo and Y. Latif and D. Scaramuzza and J. Neira and I. Reid and J.J. Leonard, "Past, Present, and Future of Simultaneous Localization And Mapping: Towards the Robust-Perception Age", in IEEE Transactions on Robotics 32 (6) pp 1309-1332, 2016.

[2] Factor Graph Tutorial, HU, Pili, Feb 29, 2012.

[3] Factor Graphs for Robot Perception, Foundations, and Trends in Robotics, Vol. 6, No. 1-2 (2017) 1–139, 2017 F. Dellaert and M. Kaess, DOI: 10.1561/2300000043
