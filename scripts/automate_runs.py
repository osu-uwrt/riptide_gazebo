#!/usr/bin/env python3
import generateRuns
import generateWorlds
import automation_handler

# Load in the data file 
# Eventually, when GUI is done, we will just serialize the JSON and pass it in rather than loading from file
# .json file is still needed even in that case for GUI because it defines the format that stuff displays in
import json
with open("params.json", "r") as read_file: 
    data = json.load(read_file)

''' Edit the run and environment parameters then run this script for general usage '''

#Run Parameters
run_config = data["general_config"]["fields"]["run_config"]["default"]
num_runs = data["general_config"]["fields"]["num_runs"]["default"]
package = data["general_config"]["fields"]["package"]["default"]
launch_file = data["general_config"]["fields"]["launch_file"]["default"]

#Ambient Color Parameters
amb_means = data["ambient_params"]["fields"]["amb_means"]["default"]
amb_stds = data["ambient_params"]["fields"]["amb_stds"]["default"]

#Fog Parameters
fog_profile = data["fog_params"]["fields"]["fog_profile"]["default"]
fog_color_mean = data["fog_params"]["fields"]["fog_color_mean"]["default"]
fog_color_std = data["fog_params"]["fields"]["fog_color_std"]["default"]
fog_start_mean = data["fog_params"]["fields"]["fog_start_mean"]["default"]
fog_start_std = data["fog_params"]["fields"]["fog_start_std"]["default"]
fog_end_mean = data["fog_params"]["fields"]["fog_end_mean"]["default"]
fog_end_std = data["fog_params"]["fields"]["fog_end_std"]["default"]

#Prop Location parameters
# {'prop_preset_name':[list of locations]}
# len(prop_locations['gate']) = number of props of type
props_locations_means = {
    'gate': data["prop_positions"]["fields"]["props_locations_means"]["props"]["gate"]["default"], # Definitely not too many, idk what you're talking about
    'cutie': data["prop_positions"]["fields"]["props_locations_means"]["props"]["cutie"]["default"],
    'crucifix_tower': data["prop_positions"]["fields"]["props_locations_means"]["props"]["crucifix_tower"]["default"]
}
props_locations_stds = {
    'gate': data["prop_positions"]["fields"]["props_locations_stds"]["props"]["gate"]["default"], # Definitely not too many, idk what you're talking about
    'cutie': data["prop_positions"]["fields"]["props_locations_stds"]["props"]["cutie"]["default"], 
    'crucifix_tower': data["prop_positions"]["fields"]["props_locations_stds"]["props"]["crucifix_tower"]["default"]
}

#Light Parameters
light_pose_mean = data["light_params"]["fields"]["light_pose_mean"]["default"]
light_pose_std = data["light_params"]["fields"]["light_pose_std"]["default"]
light_diffuse_mean = data["light_params"]["fields"]["light_diffuse_mean"]["default"]
light_diffuse_std = data["light_params"]["fields"]["light_diffuse_std"]["default"]
light_direction_mean = data["light_params"]["fields"]["light_direction_mean"]["default"]
light_direction_std = data["light_params"]["fields"]["light_direction_std"]["default"]
light_specular_mean = data["light_params"]["fields"]["light_specular_mean"]["default"]
light_specular_std = data["light_params"]["fields"]["light_specular_std"]["default"]

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