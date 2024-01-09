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
