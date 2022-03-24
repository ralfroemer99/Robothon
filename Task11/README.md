# Task 1.1

This package solves the task 1.1. It provides a mean to check a set of joint poses
for a provided robot description and see whether they lead to self-collisions.
The result is outputted inside a json file.

![GIF Showing the Execution of Task 1.1](example_gif.gif)

## Installation
The usage of the task11 package necessitates dependencies. Therefore, when cloning
the entire repository, make sure to use the ```--recurse-submodules -j8``` option
when cloning the repository.
```
git clone --recurse-submodules -j8 <url>  
```
When this step is done, make sure to compile the package by going to your
catkin workspace and running
```
catkin_make
```
In addition, it is necessary to source the environment by adding the following 
command to one's .bashrc script:
```
echo "source <your_catkin_workspace_path>/devel/setup.bash" >> ~/.bashrc
```
## Usage
In order to start the program, perform the following command:
```
roslaunch t11 task11.launch
```
This launch file performs the following actions:
1. It loads the robot description file (.urdf)
2. It loads the semantic robot description file (.srdf)
3. It starts the task_11 node, which calculates the self-collisions
4. It starts Rviz after the user has provided an input to the common interface.
   Rviz is used to visualize the joint states, as well as 
   collision markers if applicable. The common interface outputs both these
   informations as well as the collision groups.

## Additional Information
Currently, as the collision geometry of the panda arm is not very precise, collisions are detected even when in reality the real robot is not self-colliding.