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
world.launch - this just launches an empty transdec world. Important launch arg: world. For example "roslaunch riptide_gazebo world.launch world:=run9"

### models
these are all the models used in world files. We use colladae files because texture info is also stored. In the models folder, every model has its own folder with the .dae file, a model.config file, and a model.sdf file. In doing this, we can reference models throughout world files.

### scripts
automation_handler.py - this is the launch-kill script that will run through every generated world. Right now it does only that but soon it will deem success/failure percentages depending on what task code is being tested.

### worlds
2019_finals.world - this is transdec with default environment parameters. It is what world.launch defaults if no world args are specified.

generateRuns.py - right now you must edit the parameters at the top of the file because the GUI does not yet exist. Parameters like prop location mean, prop location standard deviation, number of runs, etc.
 
generateWorlds.py - it reads the run_config xml file and generates the world sdf files accordingly

props_template.xml - all the sdf snippets of models used by generateWorlds.py 

run_config.xml - this is for manual specification of worlds. 

run_config2.xml - this is what generateRuns.py generates worlds onto

run_template.xml - sdf skeleton used by generateWorlds.py

## Testing Task Code with Tweaker

If you wish to generate random worlds with random environment variations, switch over to git branch "/tweaker". You must switch riptide_descriptions and riptide_bringup to this branch as well. The OS must be 18.04 with ROS Melodic. 

### Understanding the Environment Parameters
These descriptions help give users a background for how to specify parameters.

#### Ambience
Ambience refers to "color of ambient light" and can be thought of as the world's color tint. It is defined by 4 floats. For example, the default ambience is 0.4 0.4 0.4 1

#### Props
Implemented props might include the gate, Cutie, or the crucifix. Their sdf snippets are held within the props_template.xml file. They have a 6 float location and orientation to be specified. x, y, z, roll, pitch, and yaw.

#### Fog
Fog has a color and a "thickness" that works by defining the start and end of a gradient. 

#### Lights (Sun)
Direction, location, brightness

## FAQ and Troubleshooting
