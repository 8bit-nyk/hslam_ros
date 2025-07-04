# ROS wrapper for H-SLAM (Hybrid Visual Simultaneous Localization and Mapping).
### This is an outdated version of the HSLAM project, for the most recent development repository please refer to [HSLAM Dev](https://github.com/8bit-nyk/hslam_ros2)
Easy-to-deploy ROS implementation, of a Visual SLAM application that leverages both direct and indirect visual feature tracking methods.

For an easier "out-of-the-box" Visual SLAM system leveraging docker refer to this [HSLAM Docker](https://github.com/8bit-nyk/hslam_ros_docker) .

For the base implementation that runs on datasets refer to this [HSLAM OG](https://github.com/8bit-nyk/HSLAM)


### Related Publications:

Please cite the paper if used in an academic context.

[H-SLAM: Hybrid direct--indirect visual SLAM](https://doi.org/10.1016/j.robot.2024.104729)
 ```bibtex
    @article{younes2024h,
    title={H-SLAM: Hybrid direct--indirect visual SLAM},
    author={Younes, Georges and Khalil, Douaa and Zelek, John and Asmar, Daniel},
    journal={Robotics and Autonomous Systems},
    volume={179},
    pages={104729},
    year={2024},
    publisher={Elsevier}
    }
 ```
#### Other Related Works

[Inline Photometrically Calibrated Hybrid Visual SLAM](https://doi.org/10.1109/IROS58592.2024.10802153)


```bibtex
@inproceedings{abboud2024inline,
  title={Inline Photometrically Calibrated Hybrid Visual SLAM},
  author={Abboud, Nicolas and Sayour, Malak and Elhajj, Imad H and Zelek, John and Asmar, Daniel},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={10089--10096},
  year={2024},
  organization={IEEE}
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

To build the project with ease follow the directory structure as outlined below.

Otherwise, you might face errors relating to some dependencies not being found 

1. Clone the repository:
```
git clone https://github.com/8bit-nyk/hslam_ros.git
```
Change its name to **hslam_ros_ws**

2. **Building the main project** .Navigate to the project directory:
```
cd <your_working_directory>/hslam_ros_ws/src/HSLAM
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
    cd ../../..
    ```
    or 
    ```
    cd <your_working_directory>/hslam_ros_ws
    ```

    Initialize and configure catkin:
    ```
    catkin init 
    ```

    ```
    catkin config -DCMAKE_BUILD_TYPE=RelwithDebInfo --extend /opt/ros/$ROS_DISTRO 
    ```

    Build catkin workspace:
    ```
    catkin build hslam_ros
    ```

## Usage
### Sourcing ROS and catkin
1. In ALL terminal sessions source ROS using the below command make sure to change <ros_distro> below to your ros distribution :

```
source /opt/ros/<ros_distro>/setup.bash
```

2. Source the catkin workspace using the below:
```
source ~/hslam_ros_ws/devel/setup.bash
```
**Pro Tip**

 Automate the Sourcing:

You can add these source commands to your shell startup script so that they run automatically when you open a new terminal.
1. Edit your shell startup script:
```
nano ~/.bashrc
```

2. Add the source commands:
```
source /opt/ros/<ros_distro>/setup.bash
source ~/hslam_ros_ws/devel/setup.bash

```

3. Save and exit the editor (in nano, press CTRL + X, then Y, then Enter).

4. Reload the startup script to apply changes immediately:
```
source ~/.bashrc
```
### Running the System
To run the Hybrid Visual SLAM we need a camera feed over the ROS network.

This can be provided either through a rosbag or through a live camera feed.

To publish the webcam feed (or any standard usb camera) over a ROS topic run the below command:


```
roslaunch usb_cam usb_cam-test.launch
```
A display window will pop with the camera's stream.

3. In another terminal, execute this command to run the H-SLAM algorithm on the image stream.
``` bash
rosrun hslam_ros hslam_live image:=/usb_cam/image_raw calib=~/hslam_ros_ws/src/res/camera.txt 
```
Using ROS launch:
```
roslaunch hslam_ros hslam_live.launch

```
Start moving the camera around and perform realtime Visual SLAM!

**NB.** to get better results to provide the proper camera matrix instead of the stock camera.txt file provided.

### Results:

The HSLAM system output two files when it exists:
1. **result.txt** file that contains corrected trajectory.
2. **map.pcd** file containing point cloud of the contrusted map. 

Additionally, the following data is published over the ROS network:
1. **/hslam_path**: publishes the current camera pose.
2. **/hslam_pose**: published the path tracked so far.
3. **/hslam_map**: publishes the map redered so far.

You should get an output similair to screenshot below showing the SLAM process:
![hslam-output-screenshot](https://github.com/8bit-nyk/HSLAM/assets/49674476/ab3a1c94-8f38-41da-855d-c25566f720c4)




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
This repository is licensed under the GNU General Public License version 3 [GPLv3](LICENSE).

This work is a joint collaborative effort between the:

- Vision and Robotics Lab at the American University of Beirut (AUB)
- Vision and Image Processing Group at the University of Waterloo (UW)
