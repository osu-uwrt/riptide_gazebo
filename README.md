# Riptide Gazebo

The team's simulator runs on Ubuntu 16.04 with ROS Kinetic but newest updates can only run on 18.04 with ROS Melodic. For latest developments (most of this readme refers to), checkout branch "/tweaker". This package is still a work in progress.

## Setup

```text
cd ~/osu-uwrt/riptide_setup/scripts/setup_scripts
```
```text
./install_simulator.sh
```
```text
./update_system.sh
```

## Understanding the files within riptide_gazebo package
Note: Keep in mind there are other important files to the simulator that are not in riptide_gazebo but instead riptide_description (robot's xacro files and models) or riptide_bringup (simulation.launch).

### launch
world.launch - this just launches an empty transdec world. Import launch arg: world. For example "roslaunch riptide_gazebo world.launch world:=run9"
### models

### scripts

### worlds

## Testing Task Code with Tweaker

If you wish to generate random worlds with random environment variations, switch over to git branch "/tweaker". You must switch riptide_descriptions and riptide_bringup to this branch as well. The OS must be 18.04 with ROS Melodic. 

### Understanding the Environment Parameters
These descriptions help give users a background for how to specify parameters.

#### Ambience
Ambience refers to "color of ambient light" and can be thought of as the world's color tint. It is defined by 4 floats. For example, the default ambience is 0.4 0.4 0.4 1

#### Props
Implemented props might include the gate, Cutie, or the crucifix. Their sdf snippets are held within the props_template.xml file.

#### Fog

#### Lights (Sun)

## FAQ and Troubleshooting
