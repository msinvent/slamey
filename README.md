# ROS2 Motion Planning Implementation
## 0. System Requirements
* **Operating System** : Ubuntu 18.04
* **RAM** : 8 GB to build ros2
* **Processor** : Just have an average processor of better if you are serious about building ROS2

## 1. Install the ROS2 Source

Follow instruction on https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/
I have named my **~/ros2_dashing** folder as **~/ros2_ws/ros2_ws**. Again, final path of the setup is **~/ros2_ws/ros2_ws**

## 2. Build the ROS2 source

```bash
cd ~/ros2_ws/ros2_ws/
colcon build --symlink-install
```
Depending on your system this may take 10 to 30 minutes or more, so just sit back and hear the sound of your laptop fan maxing out.

**Recommendation : do not include ros or ros2 setup.bash in your .bashrc permanently**. Rather include it in every new terminal individually. The way I do it is by having a function defined in my **.bashrc**. That provide me a one command setup everytime without worrying about environment conflicts.

```bash
function setupROS2()
{
	ROS2PATH=${HOME}/ros2_ws/ros2_ws
	source ${ROS2PATH}/install/setup.bash
	export ROS2PATH
	export ROS_DOMAIN_ID=24
	export ROS_MASTER_URI=http://master:11311
}
```
All I have to do is call setupROS2 in any new terminal to have all the ROS2 paths available
**Recommendation : Use tmux for splitting your ROS2 environment and visualizing various terminals at the same time.**

## 3. Try Examples to confirm that the build is successful
Open two separate terminals, call "setupROS2" in each to setup ROS2 environment in both. Call the talker and listener demo nodes among the two terminals.

**Terminal 1**
```
setupROS2
ros2 run demo_nodes_cpp talker
```

**Terminal 2**
```
setupROS2
ros2 run demo_nodes_cpp listener
```

So far we have tried to follow the instructions provided at https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/

Now lets try to make our own publisher and subscribers 

## 3. Creating our first ROS2 package

## 4. Shortcuts to bookmark
1.	https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet
2.	https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Development-Setup/
3.	https://github.com/msinvent/slamey


