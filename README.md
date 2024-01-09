# Open Navigation's Nav2 Docking Framework

Docking framework for Nav2 (Sponsored by NVIDIA)


TODO readme docs - see other packages for reference

TODO - server into behavior plugin to share costmap subscribers and TF buffer?


```
dock_plugins: ["dockv1", "dockv3"]
dockv1:
  plugin: "my_custom_dock_ns::Dockv1"
dockv3:
  plugin: "my_custom_dock_ns::Dockv3"
```

either as param `docks` in server or as sep filepath. if only 1 type, can set type to either that type or empty string (and will implicitly use the only one, if one only)
```
# sep yaml
docks:
  dock1:
    type: "dockv3"
    pose: [0.3, 0.3, 0.0]
  dock2:
    type: "dockv1"
    pose: [0.0, 0.0, 0.4]

# param
docks: ['dock1', 'dock2']
dock1:
  type: "dockv3"
  pose: [0.3, 0.3, 0.0]
dock2:
  type: "dockv1"
  pose: [0.0, 0.0, 0.4]
```
