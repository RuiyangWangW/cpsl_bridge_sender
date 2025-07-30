# CPSL Bridge Sender

A ROS 2 package for sending a selected list of ROS2 topics messages over TCP/UDP between a center machine and multiple robots.

```text
cpsl_bridge_sender/
├── config/
│   ├── center_sender.yaml
│   ├── robot_sender.yaml
├── launch/
│   ├── center_sender.launch
│   ├── robot_sender.launch
├── src/
│   ├── map_sender.py
│   ├── tf_sender.py
│   ├── waypoint_sender.py
│   ├── exec_summary_sender.py
│   ├── scan_sender.py
├── package.xml
└── README.md
```

## Configurations
Please make sure the IP addresses, robot_ids and etc. are matched in the yaml files.

## Usage
Launch for the Center Computer
```bash
ros2 ros2_sender_pkg launch center_sender.launch
````
Make sure you have the receiver package running on every robot you want to connect before running the center sender.

Launch for the Individual Robot
```bash
ros2 ros2_sender_pkg launch robot_sender.launch
````
Make sure you have the receiver package running on the center computer you want to connect before running the robot sender.
