# ROS wrapper for H-SLAM (Hybrid Visual Simultaneous Localization and Mapping).

Easy-to-deploy ROS implementation, of a Visual SLAM application that leverages both direct and indirect methods.

For an easier "out-of-the-box" Visual SLAM system leveraging docker refer to this [HSLAM Docker Repo](https://github.com/8bit-nyk/hslam_ros_docker) .

For the base implementation that runs on datasets refer to this [HSLAM Repo](https://github.com/8bit-nyk/HSLAM)



### Related Publications:
[A Unified Hybrid Formulation for Visual SLAM](https://scholarworks.aub.edu.lb/bitstream/handle/10938/22253/YounesGeorges_2021.pdf?sequence=5) (Doctoral dissertation), Younes, G. (2021).

[H-SLAM: Hybrid Direct-Indirect Visual SLAM](https://doi.org/10.1016/j.robot.2024.104729)  Younes, G. et al (2024).

Please cite the paper if used in an academic context.
```
@article{younes2024h,
  title={H-SLAM: Hybrid direct-indirect visual SLAM},
  author={Younes, Georges and Khalil, Douaa and Zelek, John and Asmar, Daniel},
  journal={Robotics and Autonomous Systems},
  pages={104729},
  year={2024},
  publisher={Elsevier}
}

```


# Prerequisites
We have tested the library in **Ubuntu 18.04** and **20.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.
### Dependencies:
**[NOTE]: The correct versions will be installed when building project with the provided `build.sh` script. (See Section 3 for more details)**

The project is dependent on specific versions of the following libraries:
- C++11 or C++0x Compiler: We use the new thread and chrono functionalities of C++11.
- [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. (Included in Thirdparty folder)
- OpenCV 3.4.6 (Included in Thirdparty folder)
- DBoW2 and g2o (Included in Thirdparty folder)

## Building the project

0. Create a catkin workspace
```
mkdir -p catkin_ws/
```
1. Clone the repository:
```
git clone https://github.com/8bit-nyk/hslam_ros.git
```

2. **Building the main project** .Navigate to the project directory:
```
cd <your_working_directory>/catkin_ws/src/hslam_ros/HSLAM
```

Before building the main project we need to build the thirdparty dependancies


3. Navigate to the Thirdparty directory:
```
cd Thirdparty
```

4. We provide a script `build.sh` to download and install the  specific versions of the dependency libraries needed. Execute:

  ```
  chmod +x build.sh && ./build.sh
  ```

5. Configuring and building main project:

    In terminal navigate back to HSLAM main project directory
    ```
    cd ..
    
    ```
    
    Run the following command to create the build directory
    ```
    mkdir -p build 
    ```
    Navigate into the create build directory:
    ```
    cd build 
    ```
    Configure cmake:
    ```
    cmake .. -DCMAKE_BUILD_TYPE=RelwithDebInfo 
    ``` 
    Build the Project:
    ```
    make -j $(nproc)

6.  Building the ROS wrapper:
    
    Navigate back to catkin workspace:
    ```
    cd ..
    ```
    or 
    ```
    cd <your_working_directory>/catkin_ws
    ```

    Initialize and configure catkin:
    ```
    catkin init 
    ```
    ```
    catkin config -DCMAKE_BUILD_TYPE=Release --extend /opt/ros/$ROS_DISTRO 
    ```

    Build catkin workspace:
    ```
    catkin build hslam_ros
    ```
    
## Usage

To run the H-SLAM project you will need to run two containers of the same image.
One to to publish images from the camera and the other to run the H-SLAM main application.

0. Allow access to containers:
``` bash
xhost +
```

1. Open two terminals and execute the following command in each:
``` bash
docker run -it --net=host --privileged -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/video0:/dev/video0  hslam /bin/bash
```
This command starts the container and provides an interactive terminal within it.

2. In the first terminal run the following command to open a camera stream through ROS and publish the camera's images unto a ROS topic:
``` bash
roslaunch usb_cam usb_cam-test.launch
```
A display window will pop with the camera's stream.

3. In the second terminal, execute this command to run the H-SLAM algorithm on the image stream.
``` bash
rosrun hslam_ros hslam_live image:=/usb_cam/image_raw calib=/catkin_ws/src/res/camera.txt gamma=/catkin_ws/src/res/pcalib.txt vignette=/catkin_ws/src/res/vignette.png
```

Start moving the camera around and perform realtime Visual SLAM!

### Results:

The HSLAM system output two files when it exists:
1. **result.txt** file that contains corrected trajectory.
2. **map.pcd** file containing point cloud of the contrusted map. 

Additionally, the following data is published over the ROS network:
1. **/hslam_path**: publishes the current camera pose.
2. **/hslam_pose**: published the path tracked so far.
3. **/hslam_map**: publishes the map redered so far.


## Features

- Utilizes the H-SLAM algorithm for simultaneous localization and mapping.
- Integrates with ROS Noetic and utilizes various ROS functionalities.
- Supports camera integration, including Intel Realsense cameras.
- Provides a wrapper for ROS integration and additional functionality.

## Contributing

Contributions to the H-SLAM project are welcome. If you would like to contribute, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Make the necessary changes and commit them.
4. Push your changes to your forked repository.
5. Submit a pull request detailing the changes you have made.

## License
This project is licensed under the [MIT License](LICENSE).
