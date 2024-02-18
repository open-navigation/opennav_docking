## Nova Carter Docking

This package contains a dock plugin, utility nodes, params, and demo launch files for working with the nova carter robot for auto-docking. 

The dock plugin is used by the framework to transact with the dock to get its pose, charging state information, and contact status. 
The `dock_pose_publisher` takes in the output of Isaac ROS' GPU optimized apriltags detector and republishes out a `PoseStamped` of the particular docking tag pose of interest.
The launch files provided launch the robot's basic hardware and sensors, the `isaac_ros_apriltags` server, the `dock_pose_publisher` node, and the pre-configued Docking Server.

This setup can use afixed apriltags (demo shown here) or other dock detection methods (i.e. FoundationPose, 2D lidar dock wedge detection, or 3D lidar detection) by removing the `dock_pose_publisher` and publishing the detected dock pose using your method of choice on the `detected_dock_pose` topic with type `PoseStamped`.
You can find the apriltags used in these demos in the ``media`` folder. Note: you must set up your printer to print to the exact scale - measure to make sure they are 6 inches wide.

---

### Brief Developer Instructions

Add `opennav_docking` and `navigation2` to your `isaac_ros-dev` workspace generated using the [Nova Carter instructions](https://nvidia-isaac-ros.github.io/robots/nova_carter.html#development-environment-setup) and Dockerfile. Use `humble` branch for both & build the entire workspace in Docker before continuing with `--symlink-install`.

```
# Add robot's IP to your /etc/hosts to ssh by name, add key to prevent password prompts (ssh-copy-id)
ssh nvidia@nova

# Must ensure robot is not connected to charging cable (dock OK) and bluetooth controller already connected if you want to use
cd $ISAAC_ROS_WS/src/isaac_ros_common &&   ./scripts/run_dev.sh $ISAAC_ROS_WS

# Launches the robot base, all the cameras, apriltag detector, dock pose publisher, and docking server
source install/setup.bash
ros2 launch nova_carter_dock demo_setup.launch.py

# Run test script that you wish
# ... python3 demo.py
```
