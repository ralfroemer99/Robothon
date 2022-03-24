# Task 2.1
Task 2.1 is solved by using CoppeliaSim with the [IK Plugin](https://www.coppeliarobotics.com/helpFiles/en/kinematicsPlugin.htm) and the [OMPL Library](https://ompl.kavrakilab.org).

## Scene 1
The grasping motion is achieved by interpolation between the starting and goal configuration (using one intermediate point) in Cartesian space. To this end, the inverse kinematics solution is calculated. 
To view the result, simply run the simulation in [scene_task21](https://gitlab.lrz.de/arcl21/team_a/-/blob/main/Task21/scene_task21.ttt).

## Scene 2
Due to the obstacle avoidance being much more difficult than for scene 1, we used the [PRMstar path planning algorithm](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1PRMstar.html) here to obtain a collision-free path between the start configuration and the desired end-effector pose.
To view the result, simply run the simulation in [scene_task21_cluttered](https://gitlab.lrz.de/arcl21/team_a/-/blob/main/Task21/scene_task21_cluttered.ttt).

### Videos
Videos showing the grasping motion for both scenes can be found in the [presentation](https://gitlab.lrz.de/arcl21/team_a/-/blob/main/Presentation/Presentation.pptx).
