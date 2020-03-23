# Training YOLO for your Own Custom Object 

## Generate a Dataset 

In this tutorial I will be using the gazebo sim we have used in the previous tutorials and saving the footage from the drone's camera

launch sim
```
roslaunch iq_sim runway.launch 
```

```
./startsitl.sh
```

fly the drone around the sim

and run image saver to generate your dataset
```
rosrun image_view image_saver image:/webcam/image_raw
```
## Annotate dataset

use labelImg https://github.com/tzutalin/labelImg

- follow instructions on readme

## Train YOLO

use AlexeyAB's darknet repo https://github.com/AlexeyAB/darknet

- follow instructions on readme

## Setup darknet ROS 

follow my tutorial 

https://github.com/Intelligent-Quads/iq_tutorials/blob/master/intro_to_yolo.md


