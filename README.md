# WallStalker-Lidar
This project provides wall localization functionality based on laser scanners, suitable for scenarios where a cleaning robot performs wall-following cleaning tasks.


![overviw](/assets/thumnail.png)

## Usage
### Compile
```
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/SheldonFung98/WallStalker-Lidar.git
cd ..
catkin_make
source devel/setup.sh
```

### Run
```
roslaunch laser_stickwall detect_wall.launch
```

## Code Explain

### Wall Following Detection Functionality

1. Region of Interest (ROI) Extraction: The laser scanner's ROI is defined as 2 meters in front, 0.5 meters behind, and 1 meter on each side of the robot's laser origin. This can be configured in `laser_stickwall/launch/detect_wall.launch`.
2. Rough Wall Matching: Linear regression is performed on all points within the ROI to approximate the wall. The quality of the fit is determined based on the point-to-line distance error.
3. Obstacle Removal: Starting from the foremost point among the roughly matched wall points, obstacles (points with a distance too far from the fitted line) are removed to obtain valid wall points.
4. Fine Wall Matching: Another linear regression is applied to the valid wall points to obtain a more accurate representation of the wall.
5. Navigation Point Generation: Uniformly spaced navigation points are generated along the wall line.

### Debug Visualization Functionality
1. Visualization information can be obtained during debugging by subscribing to the topic `/elevator/markerArray`.
2. Visualized images can be obtained during debugging by subscribing to the topic `/elevator/image`.

### Constraints and Conditions
1. At the starting point of the wall-following task, the robot should be close to the wall (within 1 meter).
2. The wall from the starting point to the endpoint should be at least 2 meters long.

## Communication Protocol
### Input
| Topic Name                        | Type                        | Description               | 
| -----------                      | -----------                 | -----------               |
| /scan                            | sensor_msgs/LaserScan       | Laser scanner data        |

### Output
| Topic Name                       | Type                         | Description               | 
| -----------                     | -----------                  | -----------               |
| /wall_nav/wall_path             | nav_msgs::Path               | Wall-following navigation information |
| /wall_nav/image                 | sensor_msgs::Image           | Image for debugging       |