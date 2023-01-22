#Overview to avoid Gazebo Garden Documentation headaches

To install:
`colcon build`
`cd build/CompObjectsPlugin`
`export GZ_GUI_PLUGIN_PATH="$(pwd)"`
In other words, the GZ_GUI_PLUGIN_PATH needs to point to the directory with the CompObjectsPlugin.qrc.depends file is in

The Competition Objects plugin will now appear whenever you open Gazebo Garden by clicking the dropdown on the top right of the gui.