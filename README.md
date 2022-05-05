# Group 8 Final Project -  VI-SLAM on NUance Car Dataset
## Curtis Manore, Matthew Euliano, and Andrew Goering

This project uses ORB-SLAM3 to conduct Visual Inertial SLAM using the NUance car.

### Repo Directory:

The final project proposal and final project presentation are located in the top-level directory. 

### Videos
Two shortened videos from the presentation are located in the videos directory since the videos cannot be displayed properly in a PDF version of the Final Presentation (videos were replaced with hyperlinks instead). The YouTube video links are also listed below:

- [ORB-SLAM3 on TUM-VI dataset](https://www.youtube.com/watch?v=qgXXpfL5U2A)
- [ORB-SLAM3 on NUance Car](https://www.youtube.com/watch?v=zdDk7nShIZg)

### Source Code
Source code is located in the src directory. There are three subdirectories in the src folder:
- **ORB-SLAM3**: this git submodule links to our GitHub fork of ORB-SLAM3, which works with Ubuntu 20.04 and OpenCV 4.2. Can also be found [here](https://github.com/cmanore25/ORB_SLAM3).
- **orb_slam3_ros_wrapper**: this git submodule links to our GitHub fork of an open source ROS wrapper for ORB-SLAM3 that we modified to work with V1.0 of ORB-SLAM3. Can also be found [here](https://github.com/cmanore25/orb_slam3_ros_wrapper).
- **transforms**: this folder is our source code for finding the extrinsic imu-left camera transform using the ROS tf tree.

### Configuration
The configuration files for running on the TUM-VI and NUance car can be found in the config folder. These consists of the respective yaml and launch files for stereo and stereo-inertial SLAM.

### Analysis
The analysis folder consists of MATLAB scripts, plots, and odometry data from our collected results. A summary of the MATLAB scripts is provided below:
- **gt_sensor_fusion.m** creates a 16 state, nonholonomic Extended Kalman Filter using GPS and IMU data and then exports the resulting trajectory to a text file for plotting.
- **convert_odom.m** converts the ROS odometry message that was logged to a text file to a TUM-format odometry text file. We do this conversion to compare Absolute Pose Error using the Evo plotting tool. [^1]
- **plot_computation.m** plots the CPU Load and Memory Utilization from a *glances* csv file. We collected this computational data while running ORB-SLAM3 with the NUance dataset.


[^1]: [Michael Grupp's Evo plotting tool](https://github.com/MichaelGrupp/evo) was used to compare the reference EKF trajectory to the ORB-SLAM trajectory from the NUance car. It uses Umeyama alignment to help us better compare RMSE error quantitatively.

