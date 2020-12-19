# omnivelma_navigation_2
ROS2 navigation stack for omnivelma


Navigation stack using ROS2, for master thesis.


Instructions to install:

Instructions to run:
1. Go to repository omnivelma_nav_setup
```
cd <path_to_ws>/src/omnivelma_nav_setup
```
2. 
```
source init.sh
```
3.Run simulation world - changing world instruction is in omnivelma repository
```
./scenarios/base.sh
```

4. In new terminal go to ROS2 workspace
```
cd <ros2_workspace>/src/omnivelma_navigation2
```
5. Run ros2 bridge to communicate with simulation
```
./scripts/ros_bridge.sh 
```
6. In each new console:
```
cd <ros2_workspace>
. install/setup.bash 
```
7. Launch base programs:
```
ros2 launch omnivelma_navigation_2 omnivelma_base.launch.py use_slam:=False
```
Launch localization:
```
ros2 launch omnivelma_navigation_2 omnivelma_localization.launch.py use_slam:=False
```

Launch navigation:
```
ros2 launch omnivelma_navigation_2 omnivelma_nav.launch.py
```
