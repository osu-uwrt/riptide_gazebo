#!/usr/bin/env python3
import generateRuns
import generateWorlds
import automation_handler

''' Edit the run and environment parameters then run this script for general usage '''

#Run Parameters
run_config = "run_config2.xml"
num_runs = 10
package = "riptide_bringup"
launch_file = "simulation.launch"

#Ambient Color parameters
amb_means = [0.01, 0.01, 0.01, 1]
amb_stds = [0.01, 0.01, 0.01, 0.01]

#Fog Parameters
fog_profile = 'linear'
fog_color_mean = [0.1, 0.2, 0.3, 1.0]
fog_color_std = [0.01, 0.02, 0.03, 0.001]
fog_start_mean = 2.5
fog_start_std = 1
fog_end_mean = 0
fog_end_std = 1

#Prop location parameters
# {'prop_preset_name':[list of locations]}
# len(prop_locations['gate']) = number of props of type
props_locations_means = {'gate': [[25, 20, -1.5, 0, 0, 1.57]], 'cutie': [[27, 20, 0, 0, 0, 1.57],[27, 20, 0, 0, 0, 1.57]]}
props_locations_stds = {'gate': [[2, 2, 2, .3, .3, .3]], 'cutie': [[2, 2, 1, .1, .1, .1],[2, 2, 1, .3, .3, .3]]}

#Light Parameters
light_pose_mean = [0, 0, 0, 0, 0, 0]
light_pose_std = [1, 1, 1, 1, 1, 1]
light_diffuse_mean = [1, 1, 1, 1]
light_diffuse_std = [.1, .1, .1, .1]
light_direction_mean = [0, 0, -1]
light_direction_std = [0.2, 0.2, 0.2]
light_specular_mean = [0.1, 0.1, 0.1, 1]
light_specular_std = [0.01, 0.01, 0.01, 0.01]

#package parameters into lists
amb_params = [amb_means, amb_stds]
fog_params = [fog_profile, fog_color_mean, fog_color_std, fog_start_mean, fog_start_std, fog_end_mean, fog_end_std]
props_params = [props_locations_means, props_locations_stds]
light_params = [light_pose_mean, light_pose_std, light_diffuse_mean, light_diffuse_std, light_direction_mean, light_direction_std, light_specular_mean, light_specular_std]

def main():
    generateRuns.write_run_config(run_config, num_runs, amb_params, fog_params, props_params, light_params)
    generateWorlds.read_run_config_and_build(run_config)
    automation_handler.launch_kill(num_runs, package, launch_file)

if __name__ == "__main__":
    main()