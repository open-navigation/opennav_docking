# Following Demo

This package contains a launch file and a set of configuration files for the Following server. The purpose of this package is to provide a simple and easy-to-use example of how to set up and run the server.

After installing and sourcing...


Publish continuosly a ```geometry_msgs/PoseStamped``` message to the ```/detected_dynamic_pose``` topic to simulate the object to follow.

Start the following server with:
```bash
ros2 launch opennav_following_demo setup.launch.py
```

Execute the following command to start the following the demo, which will subscribe to the detected pose and will call the following server with the first detected pose:
```bash
cd opennav_following_demo
python3 demo.py
```

The following server will then follow the detected pose.