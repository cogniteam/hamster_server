# hamster_server

# Prerequisites
map_server: 

```sudo apt-get install ros-indigo-map-server```


# Installation

```
cd ~/
mkdir hamster_ws/src -p
cd hamster_ws/src 
git clone https://github.com/cogniteam/hamster_server.git
cd ..
catkin_make
source devel/setup.bash
```

# Usage

---

After sourcing to the worskapce (source devel/setup.bash)
Run the following commands to launch the hamster server:

export ROS_MASTER_URI=http://10.0.2.152:11311
export ROS_IP=10.0.2.152
roslaunch hamster_server_launch server.launch

Once the server is launched, turn the hamster on and it should automatically connect to the server

*Make sure that the router is configured to hamster_net and that your IP is 10.0.2.152

---
