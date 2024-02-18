## Nova Carter Docking

This package contains a dock plugin and demo files for working with the nova carter dock and afixed apriltags or other dock detection methods using FoundationPose, 2D lidar dock wedge, or 3D lidar detection.

The dock plugin is used by the framework to transact with the dock to get its pose, charging information, and contact status. The launch files launch the `isaac_ros_apriltags` server and a simple node in this package `dock_pose_publisher` to convert the tag into a ``PoseStamped`` message that the dock plugin is expecting to receive.

You can find the apriltags used in these launch files in the ``media`` folder. Note that you need to set up your printer to print to the exact scale - measure to make sure they are 6 inches wide.

---

```
# Launches the robot base, all the camears, apriltag detector, and dock pose publisher
ros2 launch nova_carter_dock nova_carter_robot_and_detector.launch.py

# Get the docking server running
ros2 launch opennav_docking docking_server

# Run test script that you wish
# ...
```