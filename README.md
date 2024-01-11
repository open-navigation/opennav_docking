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

TODO describe API and role.

## Configuration

TODO parameters

## Etc

TODO notes of interest
- staging pose and different robot kinematics (make contacts towards, so feasible planning to only require straight-ish)
