# Creating a Privacy-aware Environment Map

## Overview

**Privacy-aware Environment Map** is a ROS-based simulation project for creating a 2D environment map and
marking privacy-sensitive objects detected by the robot's cameras.

The project is developed around the **Care-O-Bot 4**
simulation in **Gazebo**, using **ROS Noetic**. The system uses:

1.  LiDAR data and **GMapping** to construct a 2D occupancy map.
2.  RGB camera images and a trained **YOLO** model to detect
    privacy-sensitive objects.
3.  Registered RGB-D depth data to estimate the 3D position of detected
    objects.
4.  TF transformations to convert camera-relative positions into the
    global map frame.
5.  Object tracking and RViz markers to maintain and visualize detected
    privacy objects.

The final result is an occupancy grid with persistent privacy-object
markers that can be used as a basis for privacy-aware robot behavior.

## Running the System

Run the startup script:

```bash
./start_privacy_simulator.sh
```

The provided startup pipeline performs the following:

1.  Sources ROS and the catkin workspace.
2.  Starts the Care-O-Bot simulation.
3.  Waits for the simulator to initialize.
4.  Starts GMapping.
5.  Starts left and right privacy detectors.
6.  Starts left and right object-position nodes.
7.  Starts left and right privacy-marker nodes.

After startup, use RViz to visualize the `/map` topic and privacy markers.

## Project Goals

The project follows four main steps:

1.  Familiarize with the Care-O-Bot system and its simulation
    environment.
2.  Generate an environment map using the robot's LiDAR sensors.
3.  Identify privacy-sensitive objects using camera-based object
    detection.
4.  Mark the detected objects in the generated map.

## Key Features

-   2D environment mapping using GMapping
-   YOLO-based privacy-sensitive object detection
-   Left and right RGB-D camera pipelines
-   Registered-depth-based object localization
-   TF-based transformation into the map frame
-   Object tracking and smoothing
-   Persistent privacy markers in RViz
-   Simulation logging and configurable detection parameters

## System Architecture

``` text
                    Care-O-Bot 4 Simulation
                           (Gazebo)
                                |
              +-----------------+-----------------------------+
              |                                               |
           LiDAR                                           Cameras
              |                                               |
      /scan_unified                     +---------------------+---------------------+
              |                         |                                           |
         GMapping                 Left RGB-D Camera                          Right RGB-D Camera
              |                         |                                           |
           /map                    YOLO Detection                              YOLO Detection
                                        |                                           |
                                  Depth + CameraInfo + TF                     Depth + CameraInfo + TF
                                        |                                           |
                                  Object Localization                         Object Localization
                                        |                                           |
                                  Object Tracking                             Object Tracking
                                        |                                           |
                                  Privacy Markers                             Privacy Markers
                                        |                                           |
                                        +---------------------+---------------------+
                                                              |
                                                              v
                                                            RViz
                                                (Map + Privacy Object Locations)
```

## Repository Structure

```text
privacy-aware-environment-map/
├── Analysis/
│   └── ...
├── Report/
│   └── Privacy_Aware_Environment_Map.pdf
├── Simulated World/
│   ├── envs/
│   ├── launch/
│   └── urdf/
├── README.md
├── object_position.py
├── privacy_detector.py
├── privacy_marker.py
├── start_privacy_simulator.sh
├── stop_privacy_simulator.sh
└── ...
```

## Software Stack

-   ROS Noetic
-   Gazebo
-   Care-O-Bot 4
-   GMapping
-   Python / rospy
-   OpenCV
-   cv_bridge
-   Ultralytics YOLO
-   RViz

## Prerequisites

Install/configure:

-   Ubuntu environment compatible with ROS Noetic
-   ROS Noetic
-   Gazebo
-   Catkin workspace
-   Care-O-Bot simulation packages
-   Python dependencies used by the detector
-   A trained YOLO model

Source ROS and the project workspace:

``` bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## Hardware and Simulation

The project uses the simulated **Care-O-Bot 4**.

The startup configuration uses:

``` text
ROBOT=cob4-5
ROBOT_ENV=ipa-Nirish
```

Logs are stored under:

``` text
$HOME/privacy_sim_logs
```

The project documentation also references the ipa-apartment
environment.

## Model Setup

The detector uses a trained YOLO model. The configured/default project
path is:

``` text
/home/labuser/catkin_ws/src/privacy_environment_map/models/Model.pt
```

## Manual Pipeline Execution (Starting Each Component Indivisually)

The main components can be started separately when debugging:

``` bash
# ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Robot configuration
export ROBOT=cob4-5
export ROBOT_ENV=ipa-Nirish

# Start simulation
roslaunch cob_bringup_sim robot.launch
```

Then start mapping:

``` bash
# ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# start SLAM mapping
rosrun gmapping slam_gmapping \
  scan:=/scan_unified \
  _base_frame:=base_link \
  _odom_frame:=odom_combined
```

Start the privacy detector script.

``` bash
# ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Start the privacy detector script
rosrun privacy_environment_map privacy_detector.py _camera_id:=left _node_name:=privacy_detector_left
rosrun privacy_environment_map privacy_detector.py _camera_id:=right _node_name:=privacy_detector_right
```

Start the object localization script.

``` bash
# ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Start the object localization script
rosrun privacy_environment_map object_position.py _camera_id:=left __name:=object_position_left
rosrun privacy_environment_map object_position.py _camera_id:=right __name:=object_position_right
```

Start the marker nodes script.

``` bash
# ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Start the marker nodes script
rosrun privacy_environment_map privacy_marker.py _camera_id:=left __name:=privacy_marker_left
rosrun privacy_environment_map privacy_marker.py _camera_id:=right __name:=privacy_marker_right
```

## ROS Nodes

| Node / Script | Purpose |
|---|---|
| `privacy_detector.py` | Detects privacy-sensitive objects using YOLO |
| `object_position.py` | Estimates object positions from RGB-D data |
| `privacy_marker.py` | Publishes privacy objects as RViz markers |

## ROS Topics

### Mapping

``` text
/scan_unified
/map
```

Source LiDAR streams include:

``` text
/base_laser_front/scan
/base_laser_left/scan
/base_laser_right/scan
```

### Left Camera

``` text
/torso_cam3d_left/rgb/image_raw
/torso_cam3d_left/depth_registered/image_raw
/torso_cam3d_left/rgb/camera_info
```

### Right Camera

``` text
/torso_cam3d_right/rgb/image_raw
/torso_cam3d_right/depth_registered/image_raw
/torso_cam3d_right/rgb/camera_info
```

Detection topics are camera-specific:

``` text
/privacy_detections_<camera_id>
```

## Object Localization

``` text
YOLO Bounding Box
       |
       v
Bounding-box Center
       |
       v
Registered Depth Image
       |
       v
Depth Sampling
       |
       v
Camera Pinhole Back-projection
       |
       v
3D Camera Position
       |
       v
TF Transformation
       |
       v
3D Position in map
```

This is particularly useful for free-standing objects and objects
located on tables or shelves.

## Object Tracking

The tracker maintains:

-   Object class
-   Camera ID
-   Image-space center
-   Map position
-   Depth/range
-   Last-seen timestamp

## Logs and Debugging

Simulation logs are stored in:

``` text
$HOME/privacy_sim_logs
```

Useful ROS commands:

``` bash
rostopic list
rostopic echo /map
rostopic echo /scan_unified
rostopic hz /scan_unified
rosrun tf view_frames
```

Check camera topics with:

``` bash
rostopic list | grep torso_cam3d
```

Check privacy detections with:

``` bash
rostopic list | grep privacy_detections
```

## Expected Output

A successful run should produce:

1.  A Gazebo Care-O-Bot simulation.
2.  A 2D occupancy map on `/map`.
3.  YOLO detections from the configured cameras.
4.  Estimated positions for detected privacy-sensitive objects.
5.  Tracked objects in the global `map` frame.
6.  Privacy markers visible in RViz.

## Known Limitations

The project documentation identifies several limitations:

-   2D map representation
-   GMapping is used for mapping
-   Independent camera tracks can result in duplicate detections
-   Fixed privacy-detection classes
-   Possible false positives
-   Depth noise
-   Simulation-focused evaluation
-   Limited quantitative evaluation

## Future Work

Potential future improvements include:

-   3D mapping using OctoMap
-   Alternative mapping approaches such as Cartographer
-   Autonomous exploration
-   Larger and more diverse training datasets
-   Multi-view camera fusion
-   Privacy severity levels
-   Privacy-aware robot behavior
-   Real-hardware validation

## Privacy Considerations

The purpose of the project is to explicitly represent privacy-sensitive
objects in an environment map so that future robot behavior can take
privacy into account.

The current implementation focuses on **detection, localization,
tracking, and visualization**. Automated privacy-aware robot actions are
outside the demonstrated core pipeline.