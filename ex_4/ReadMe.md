This package was realized from these steps:
- In Matlab, the file circular_workspace.traj was created from the script matlab in order to generate a circular workspace path in front of the robot lying on the Y-Z plane with fixed orientation such that the flange lies on the same plane. 
- After the creation of the file .traj, the file named circular_path.cpp was realized in order to extract this trajectory from the workspace and obtain the one in joint space (thanks to the IK).Then, it was guaranteed the continuity and the respect for velocity and acceleration.
- There are two publishers: one created for visualize the kinematic solution on RViz (joint space trajectory) and another one to verify that joint limits are respected by publishing trajectories to rqt_multiplot.


Trajectory files are loaded by using the WorkspaceTrajectory class of the moveit_dp_redundancy_resolution package, which is, therefore, a dependency. Also, this package takes a dependency on smartsix_moveit_config, defining the planning information for the Comau SmartSix robot, used as an example.

## How it works:
Run the launch file :

```bash
roslaunch ex_4 planner.launch
```
In this way, it will open the rqt_multiplot (YOU HAVE TO RUN ALL PLOTS) and then RViz. On the terminal you will see that you can press Next and the robot will do the circular trajectory and and at the same time it will publish trajectory to rqt_multiplot.