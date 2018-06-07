# Visual Servoing and Hand Eye Calibration with Dobot
## Hand-eye calibration
The common method of hand eye calibration outlined in (https://ieeexplore.ieee.org/document/34770/)
which requires solving AX = XB where X is the pose of the camera with respect to the end
effector, A is a series of relative poses taken between pairs of end effector poses
and B is a series of relative poses taken between pairs of camera poses. However, 
since the dobot end effector, which the camera is mounted to, has only a single axis of rotation,
AX = XB cannot be solved. One option would be rotate the camera with respect to 
the end effector which would produce an extra axis of rotation. For the sake of time,
we simply measured the pose of the checkerboard with respect to the base of the
robot.

### Eye-on-hand calibration - sensors_main_eye_on_hand.m
This setup has a camera mounted rigidly to the robotic arm and the checkerboard is placed in the environment.
This requires the pose of the checkerboard with respect to the base of the robot to be
input by the user. We use 2 extra coordinate 
frames, B' and CB' (base' and checkerboard') which are the bottom right corners of the base 
and checkerboard respectively whose coordinate frames, B and CB, are in the centre of the 
area which makes it difficult to accurately measure to. 

The output of this script will be the pose of the camera with respect to the end effector.

### Eye-to-hand calibration
This setup has a camera fixed in the world and the checkerboard in this case is mounted to the end effector of the robot.
This requires the pose of the checkerboard with respect to the end effector of the robot.


The output of this script will be the pose of the fixed camera with respect to base of the robot.

## Visual Servoing
In order to run the visual servoing, the pose of the camera with respect to the end effector
of the robot needs to be known. This can be found using the eye-on-hand calibration script.

The visual servoing in its current state tracks 4 circular objects which are arranged in a 
square pattern.

# Authors
- Brendan Emery
- Ahmad Kamal