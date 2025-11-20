# ROS2-Dice-Roller

Presentation link:  
https://docs.google.com/presentation/d/17dWJg9Xzjme5_viVJAJSdn36DbBFt2w16UpQ6tVTV7M/edit?usp=sharing

Download this package:  
git clone https://github.com/mac785/ROS2-Dice-Roller

## Preparing the Docker Container

Perform all following commands from inside the cloned repository

Step 1: Build the image:  
`docker build -t yolo_ros .`

Once the image is build, launch the container:  
`docker compose up -d`  
(Note: you may need to adjust the contents of docker-compose.yml to accurately map camera and controller functionality)

Once the container is launched, open a terminal with:  
`docker compose exec ros bash`

## Running the Program

Once inside the container, you can run the full project with:  
`ros2 launch bringup full_system.launch.py`

Or, if you'd like to run each node individually:  

`ros2 run webcam_publisher webcam_pub`  
`ros2 run dice_detector dice_node`  
`ros2 run joy joy_node`  
`ros2 run dualsense_node dualsense_node`  
`ros2 run trigger_node trigger_node`  
`ros2 launch my_robot_bringup my_robot_gazebo.launch.xml`


Once all nodes are online, roll some dice in front of the camera. Then, press directional buttons on the controller (D-Pad). This will move the robot.  
Up: forwards  
Down: backwards  
Left: turn left  
Right: turn right  

Dockerfile based on and requirements.txt sourced from [yolo_ros by mgonzs13](https://github.com/mgonzs13/yolo_ros)

Image dataset [Dice by Roboflow user Workspace (workspace-spezm)](https://universe.roboflow.com/workspace-spezm/dice-0sexk)

v4l-utils to find your camera info
`v4l2-ctl --list-devices`
