# Open Navigation's Nav2 Docking Framework

This package contains an automatic robot docking framework & auxiliary tools. It uses plugin `dock` implementations for a particular platform to enable the framework to generalize to robots of many different kinematic models, charging methods, sensor modalities, and so on. It can also handle a database of many different docking locations and dock models to handle a heterogeneous environment. This task server should be called by an application BT or autonomy application to dock once completed with tasks or battery is low -- _not_ within the navigate-to-pose action itself.

This work is sponsored by [NVIDIA](https://www.nvidia.com/en-us/) and created by [Open Navigation LLC](https://opennav.org).

This is split into 4 packages

- `opennav_docking`: Contains the main docking framework
- `opennav_docking_msgs`: Contains the action interfaces for docking and undocking
- `opennav_docking_core`: Contains the dock plugin header file to be implemented for each dock type
- `opennav_docking_bt`: Contains behavior tree nodes and example XML files using the docking task server


TODO video in action / nv & on graphic

## Architecture

TODO + diagram + statem achine of sub-actions

## Dock Specification

There are two unique elements to consider in specifying docks: dock _instances_ and dock _plugins_. Dock instances are instances of a particular dock in the map, as the database may contain many known docks (and you can specify which by name you'd like to dock at). Dock plugins are the model of dock that each is an instance of. The plugins contain the capabilities to generically detect and connect to a particular dock model. This separation allows us to efficiently enable many individual docking locations of potentially several different revisions with different attributes.

The **dock plugins** are specified in the parameter file as shown below. If you're familiar with plugins in other Nav2 servers, this should look like a familiar design pattern. Note that there is no specific information about the dock's pose or instances. These are generic attributes about the dock revision (such as staging pose, enable charging command, detection method, etc). You can add additional parameters in the dock's namespace as you choose (for example `timeout`).

```
dock_plugins: ["dockv1", "dockv3"]
dockv1:
  plugin: "my_custom_dock_ns::Dockv1"
dockv3:
  plugin: "my_custom_dock_ns::Dockv3"
  timeout: 10.0
```

There are two ways to populate the database of **dock instances** in your environment: through the parameter file or an external file. If you'd like to embed your dock information in your Docking Server config file (if you only have a couple of docks), you may use a similar method as defining the dock plugins, specifying the docks in the ``docks`` parameter. Note that we specify the plugin type and the dock's location `[x, y, theta]` in a particular frame.

```
docks: ['dock1', 'dock2']
dock1:
  type: "dockv3"
  frame: map
  pose: [0.3, 0.3, 0.0]
dock2:
  type: "dockv1"
  frame: map
  pose: [0.0, 0.0, 0.4]
```

If you'd prefer to specify the docks using an external file, you may use the `dock_database` parameter to specify the filepath to the yaml file. The file should be laid out like:

```
docks:
  dock1:
    type: "dockv3"
    frame: map
    pose: [0.3, 0.3, 0.0]
  dock2:
    type: "dockv1"
    frame: map
    pose: [0.0, 0.0, 0.4]
```

Note that you may leave the `type` to an empty string **if** there is only one type of dock being used. The `frame` will also default to `map` if not otherwise specified. The `type` and `pose` fields are required.

## Interfaces 

TODO - docking and undocking spec.

## Dock Plugin

The dock plugin has several key functions to implement. First, there are two
functions related to poses:

 * `getStagingPose`: This function should transform the dock pose into a staging
   pose. Nav2 will be used to move the robot to the staging pose.
 * `getRefinedPose`: This function can be used refine the dock pose using sensors.
   Depending on how the robot can detect the dock, this might use laser scan data
   or camera data.

There are two functions used during dock approach:

 * `isDocked`: As the robot approaches the dock, this function should tell us when
   to stop driving forward. There are many ways this function could be implemented
   depending on the robot hardware:
   * If a charge dock communicates before charging starts, this should be done here.
     If the charge dock needs an "enable" message to be sent, this is a good place to send it.
   * If the charge dock doesn't use any sort of communications, you might still be
     able to detect contact with the dock by looking at the motor effort/current,
     or determining that velocity has dropped to zero.
   * In the absolute simplest case, `isDocked` might just call `isCharging` if there
     is no other feedback to know when we are docked.
 * `isCharging`: The approach stops when the robot reports `isDocked`, then we wait
   for charging to start by calling `isCharging`.

Similarly, there are two functions used during undocking:

 * `disableCharging`: This function is called before undocking commences to help
   prevent wear on the charge contacts. If the charge dock supports turning off
   the charge current, it should be done here.
 * `hasStoppedCharging`: This function is called while the controller is undocking.
   Undocking is successful when charging has stopped and the robot has returned to
   the staging pose.

Keep in mind that the docking and undocking functions should return quickly as they
will be called inside the control loop. Also make sure that `isDocked` should
return true if `isCharging` returns true.

### Simple Charging Dock Plugin

The `SimpleChargingDock` plugin is an example which may also be fully functional for
some robots. The section below details how the plugin works.

The `getStagingPose` function is quite simple. It applys two transformations to the
dock pose. The first is to back the pose up in the X axis based on the parameter
`staging_x_offset`. The pose can then be rotated by `staging_yaw_offset` parameter,
although this will usually be set to 0.0 unless the robot is backing onto the dock.

`getRefinedPose` can be used in two ways. The simplest is a blind approach where
the returned dock pose will simply be equal to whatever was passed in - this likely
won't work on a real robot but is useful for unit tests or simulation. The more
realistic use case is to use an AR marker. By setting the parameter
`use_external_detection_pose` to true, the plugin will subscribe to a
`geometry_msgs/PoseStamped` topic called `detected_dock_pose`. This can be used
with the `image_proc/TrackMarkerNode`. It is unlikely the AR marker pose is actually
the exact pose you want to dock with, so several parameters are supplied:

 * `external_detection_translation_x`
 * `external_detection_translation_y`
 * `external_detection_rotation_yaw`
 * `external_detection_rotation_pitch`
 * `external_detection_rotation_roll`

There is also a `external_detection_timeout` parameter. The detected pose can also
be filtered by setting the `filter_coef` parameter.

During the docking approach, there are two options for detecting `isDocked`:

 * TODO: document stall detection when merged
 * If stall detection is not used, the dock pose is compared with the robot
   pose and `isDocked` returns true when the distance drops below the
   specified `docking_threshold` parameter.

The `isCharging` and `hasStoppedCharging` functions rely on subscribing to a
`sensor_msgs/BatteryState` message on topic `battery_state`. The robot is
considered charging when the `current` field of the message exceeds the
`charging_threshold` parameter setting.

For debugging purposes, there are several publishers which can be used with RVIZ:

 * `dock_pose` (`geometry_msgs/PoseStamped`): Publishes the dock pose any time
   `getRefinedPose` is called.
 * `filtered_dock_pose` (`geometry_msgs/PoseStamped`): Publishes the filtered but
   untransformed detected dock pose. Only published if using external detection.
 * `staging_pose` (`geometry_msgs/PoseStamped`): Publishes the staging pose when
   `getStagingPose` is called.

## Configuration

TODO parameters

## Use Cases

While the most common use case is a charge dock with a known position in the `map` frame,
there are other ways to use this package, for instance:

 * Dock information could be provided in the message in `odom` or even `base_link` frame.
   For instance, the user might teleoperate the robot to a pose that is pointed at the dock
   and then have a button on the controller that calls the docking action with a pose
   directly in front of the robot.

## Etc

### On Staging Poses

Staging poses are where the robot should navigate to in order to start the docking procedure. This pose should be close enough to the dock to accurately detect the dock's presence, but far enough that if its moved slightly or the robot's localization isn't perfect it can still be detected. The robot's charging contacts or charging location should be pointed towards the dock in this staging pose. That way, a feasible global planner can be used to model your robot's real constraints while getting to the docking pose (non-circular, non-holonomic), rather than complicating the docking process itself.

