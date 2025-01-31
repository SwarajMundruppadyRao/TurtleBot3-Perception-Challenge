# TurtleBot3-Perception-Challenge
This repository contains the code for ENPM673 Final Project. Implementation of a paper trail following, Stop Sign Detection, Dynamic Obstacle Detection and avoidance were the goals of this project. 

## Team Members : 

- Anbarasan Kandasamy (120270697) 
- Hariharasudan Muralidaran (120172656)
- Manoj Kumar Selvaraj (120511257)
- Swaraj Mundruppady Rao (120127007)

## Requirements 
-  TurtleBot3 Waffle
-  ROS2 Humble
-  OpenCV library

## Source Files

The `src` directory contains the following key Python scripts:

- `stop_sign_detection.py`: This script is responsible for detecting stop signs. When a stop sign is detected, it publishes a boolean value (true/false) to the `stop_sign_detected` topic.

- `optical_flow.py`: This script is used to detect and stop the TurtleBot when a moving obstacle is encountered. Also the movement detected true / false information is published to `optical_flow` topic. While displaying the output of motion detection the horizon line is also shown as per the information it received from the `horizon_line` topic.

- `horizon.py` : This script contains the code to find the Horizon line of the image, the code will publish the horizon line information to this topic : `horizon_line`

- `enpm673_final_proj_main` : This script contains the code to run the turtlebot3 in the environment by following the paper trail. It also subscribes to the `stop_sign_detected` and `optical_flow` topic and takes appropriate actions.


## Running the Simulation

Unzip the src.zip file in your workspace

To run the simulation, please follow the following steps:

- Launch the turtlebot 3 in the environment ( Follow the instructions given in this repository https://github.com/TommyChangUMD/ENPM673_turtlebot_perception_challenge )

- Start the Horizon Line code using the following command to detect the horizon line

```bash
    ros2 run finalrun horizon
```
- To start the motion detection using Optical flow, use the following command. The threshold was set according to the observations on out computer, kindly change the motion and angle threshold if required for efficient run in the environment

```bash 
    ros2 run finalrun optical_flow 
```

- To start the stop sign detection use the following command. This will open a window in which a bounding box is drawn over the stop sign when detected.

```bash
    ros2 run finalrun stop_sign_detection
```

- To start the paper trail following sequence use the following command and view the trail following on the pop up window

```bash
    ros2 run finalrun enpm673_final_proj_main
```

## To run on a real TurtleBot3-waffle

1. SSH into TurtleBot and run the following command:

  ```bash
    ros2 launch turtlebot3_bringup robot.launch.py
  ```
2. Set the ROS Domain ID to ensure that the system and TurtleBot is on the same network and also have the same ROS_DOMAIN_ID. The ROS_DOMAIN_ID can be set using the following command:
   ```bash
    export ROS_DOMAIN_ID=$domain_id$
   ```

3. Start the Horizon Line code using the following command to detect the horizon line

```bash
    ros2 run finalrun horizon
```
4. To start the motion detection using Optical flow, use the following command. The threshold was set according to the observations on out computer, kindly change the motion and angle threshold if required for efficient run in the environment

```bash 
    ros2 run finalrun optical_flow 
```

5. To start the stop sign detection use the following command. This will open a window in which a bounding box is drawn over the stop sign when detected.

```bash
    ros2 run finalrun stop_sign_detection
```

6. To start the paper trail following sequence use the following command and view the trail following on the pop up window

```bash
    ros2 run finalrun enpm673_final_proj_main
```


## Videos

### TurtleBot3 WafflePi Run 

https://github.com/user-attachments/assets/fc94bf12-7f4a-40cc-95b9-e5eefa305822

### Simulation Runs 

#### Stop Sign Detection 



https://github.com/user-attachments/assets/ece27263-208a-4996-8b2b-f839e4abcd24


#### Optical Flow 


https://github.com/user-attachments/assets/a186969d-e9cd-4d6d-881c-6f99ceb2e7ea







 




