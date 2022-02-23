# purdue_lunabotics

This is the official git repo of the Purdue Lunabotics software team.

## Structure

- Make sure to check out the `readme.md` for each package for specific usage instructions.

- Each branch of format `year-(year+1)` refers to a stable version of the software system of that year.

- The `master` branch is the latest, stable code release.

TODO: Use releases feature to simplify versioning

## Quick Start

1. [Install ROS](https://wiki.purduearc.com/wiki/tutorials/setup-ros)

This tutorial assumes you have your `catkin_ws` initialized in your home directory: `~/catkin_ws`

2. Clone the repository to `src` folder of ROS workspace

```
cd ~/catkin_ws/src
git clone https://github.com/PurdueLunabotics/purdue_lunabotics.git
```
3. Install dependencies at the root of your catkin workspace
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```

4. Build + source (Do this every time you download new packages)

**For MacOS only**

The `rplidar_ros` package doesn't build as of now, so run this to ignore it during build: 
```
touch ~/catkin_ws/src/purdue_lunabotics/rplidar_ros/CATKIN_IGNORE
```

**All**

```
catkin build
source ~/catkin_ws/devel/setup.bash # or .zsh if you use a zsh terminal
```
> Note: Build + source every time you add new packages. Source every time you open a fresh terminal, or add the line to your ~/.bashrc (or .zshrc) so it sources automatically

5. Check out specific package readmes in:

- `lunabot_bringup`
- `lunabot_localization`
- `lunabot_nav`
