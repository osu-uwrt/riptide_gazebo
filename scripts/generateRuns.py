#!/usr/bin/env python3

'''Do not edit this script for general usage'''

from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.etree import ElementTree
import numpy as np
from math import log10, floor
from xml.dom import minidom
from scipy.stats import norm

def round_sig(x, sig=1):
    return round(x, sig-int(floor(log10(abs(x))))-1)

#Generate random ambiences around standard deviation and mean
def generate_rdm_amb(num_runs, amb_params):
    amb_mean = amb_params[0]
    amb_std = amb_params[1]
    ambient_dist = []
    i = 0
    for a_comp in amb_mean:
        a_comp_dist = np.random.normal(loc=a_comp, scale=amb_std[i], size=num_runs)
        ambient_dist.append(a_comp_dist)
        i += 1
    return ambient_dist

#Generate random fogs around standard deviation and mean
def generate_rdm_fogs(num_runs, fog_params):
    fog_profile = fog_params[0]
    fog_color_mean = fog_params[1]
    fog_color_std = fog_params[2]
    fog_start_mean = fog_params[3]
    fog_start_std = fog_params[4]
    fog_end_mean = fog_params[5]
    fog_end_std = fog_params[6]
    run_fog_starts = []
    run_fog_ends = []
    i = 0
    fog_color_dist = []
    for f_c_comp in fog_color_mean:
        f_c_comp_dist = np.random.normal(loc=f_c_comp, scale=fog_color_std[i], size=num_runs)
        fog_color_dist.append(f_c_comp_dist)
        i += 1
    run_fog_starts = np.random.normal(loc=fog_start_mean, scale=fog_start_std, size=num_runs)
    run_fog_ends = np.random.normal(loc=fog_end_mean, scale=fog_end_std, size=num_runs)
    return fog_profile, fog_color_dist, run_fog_starts, run_fog_ends

#Generate random prop locations around standard deviation and mean
def generate_rdm_props(num_runs, props_params):
    props_locations_means = props_params[0]
    props_locations_stds = props_params[1]
    props_locations_distributions_all = [] #list of dictionaries
    for prop in props_locations_means:
        props_locations_distributions = {}
        prop_locations_mean = props_locations_means[prop]
        prop_locations_std = props_locations_stds[prop]
        prop_locations_distribution = []
        index = 0
        # prop_locations_mean = [[x1 y1 z1 roll1 pitch1 yaw1],[x2 y2 z2 roll2 pitch2 yaw2]
        for prop_location_mean in prop_locations_mean:
            index2 = 0
            prop_location_distribution = []
            # prop_location_mean = [x1 y1 z1 roll1 pitch1 yaw1]
            for component in prop_location_mean:
                component_distribution = []
                component_distribution = np.random.normal(loc=component, scale=prop_locations_std[index][index2], size=num_runs)
                prop_location_distribution.append(component_distribution)
                index2 += 1
            index += 1
            prop_locations_distribution.append(prop_location_distribution)
        props_locations_distributions[prop] = prop_locations_distribution
        props_locations_distributions_all.append(props_locations_distributions)
    return props_locations_distributions_all
    
#Generate random lights around standard deviation and mean
def generate_rdm_lights(num_runs, light_params):
    light_pose_mean = light_params[0]
    light_pose_std = light_params[1]
    light_diffuse_mean = light_params[2]
    light_diffuse_std = light_params[3]
    light_direction_mean = light_params[4]
    light_direction_std = light_params[5]
    light_specular_mean = light_params[6]
    light_specular_std = light_params[7]
    i = 0
    light_pose_dist = []
    for l_p_comp in light_pose_mean:
        l_p_comp_dist = np.random.normal(loc=l_p_comp, scale=light_pose_std[i], size=num_runs)
        light_pose_dist.append(l_p_comp_dist)
        i += 1
    i = 0
    light_diffuse_dist = []
    for l_d_comp in light_diffuse_mean:
        l_d_comp_dist = np.random.normal(loc=l_d_comp, scale=light_diffuse_std[i], size=num_runs)
        light_diffuse_dist.append(l_d_comp_dist)
        i += 1
    i = 0
    light_specular_dist = []
    for l_s_comp in light_specular_mean:
        l_s_comp_dist = np.random.normal(loc=l_s_comp, scale=light_specular_std[i], size=num_runs)
        light_specular_dist.append(l_s_comp_dist)
        i += 1
    i = 0
    light_direction_dist = []
    for l_d_comp in light_direction_mean:
        l_d_comp_dist = np.random.normal(loc=l_d_comp, scale=light_direction_std[i], size=num_runs)
        light_direction_dist.append(l_d_comp_dist)
        i += 1
    return light_pose_dist, light_diffuse_dist, light_specular_dist, light_direction_dist

#update run_config xml file to reflect all parameters
def write_run_config(run_config, num_runs, amb_params, fog_params, props_params, light_params):

    #unpackage listed parameters
    ambient_dist = generate_rdm_amb(num_runs, amb_params)
    fog_profile, fog_color_dist, run_fog_starts, run_fog_ends = generate_rdm_fogs(num_runs, fog_params)
    props_locations_distributions_all = generate_rdm_props(num_runs, props_params)
    light_pose_dist, light_diffuse_dist, light_specular_dist, light_direction_dist = generate_rdm_lights(num_runs, light_params)

    tree = ElementTree.parse(run_config)
    runs = tree.getroot()
    for run in runs.findall('run'):
        runs.remove(run)
    for ri in range(num_runs):
        run_name = 'run' + str(ri)
        run = ElementTree.SubElement(runs, 'run', {'name':run_name})

        light = ElementTree.SubElement(run, 'light',  {'name':'sun', 'type':'directional'})
        light_pose = ElementTree.SubElement(light, 'pose')
        light_pose.text = str(light_pose_dist[0][ri]) + ' ' + str(light_pose_dist[1][ri]) + ' ' + str(light_pose_dist[2][ri]) + ' ' + str(light_pose_dist[3][ri])  + ' ' + str(light_pose_dist[4][ri])  + ' ' + str(light_pose_dist[5][ri])
        light_diffuse = ElementTree.SubElement(light, 'diffuse')
        light_diffuse.text = str(light_diffuse_dist[0][ri]) + ' ' + str(light_diffuse_dist[1][ri]) + ' ' + str(light_diffuse_dist[2][ri]) + ' ' + str(light_diffuse_dist[3][ri])
        light_specular = ElementTree.SubElement(light, 'specular')
        light_specular.text = str(light_specular_dist[0][ri]) + ' ' + str(light_specular_dist[1][ri]) + ' ' + str(light_specular_dist[2][ri]) + ' ' + str(light_specular_dist[3][ri])
        light_direction = ElementTree.SubElement(light, 'direction')
        light_direction.text = str(light_direction_dist[0][ri]) + ' ' + str(light_direction_dist[1][ri]) + ' ' + str(light_direction_dist[2][ri])

        for prop in props_locations_distributions_all:
            for prop_preset_name in prop:
                props_locations = prop[prop_preset_name]
                prop_element = ElementTree.SubElement(run, 'prop', {'preset':prop_preset_name})
                #print(len(props_locations))
                for prop_location in props_locations:
                    #print(prop_location)
                    pose = ElementTree.SubElement(prop_element, 'pose')
                    pose.text = str(prop_location[0][ri]) + ' ' + str(prop_location[1][ri]) + ' ' + str(prop_location[2][ri]) + ' ' + str(prop_location[3][ri]) + ' ' + str(prop_location[4][ri])+ ' ' + str(prop_location[5][ri])


        scene = ElementTree.SubElement(run, 'scene')
        ambient = ElementTree.SubElement(scene, 'ambient') 
        ambient.text = str(ambient_dist[0][ri]) + ' ' + str(ambient_dist[1][ri]) + ' ' + str(ambient_dist[2][ri]) + ' ' + str(ambient_dist[3][ri])
        
        if(run_fog_starts[ri] < 0):
            run_fog_starts[ri] = 0

        fog = ElementTree.SubElement(scene, 'fog')
        fog_color = ElementTree.SubElement(fog, 'color')
        fog_color.text = str(fog_color_dist[0][ri]) + ' ' + str(fog_color_dist[1][ri]) + ' ' + str(fog_color_dist[2][ri]) + ' ' + str(fog_color_dist[3][ri])
        fog_type = ElementTree.SubElement(fog, 'type')
        fog_type.text = fog_profile
        fog_start = ElementTree.SubElement(fog, 'start')
        fog_start.text = str(int(run_fog_starts[ri]))
        fog_end = ElementTree.SubElement(fog, 'end')
        fog_end.text = str(int(run_fog_ends[ri]))

    #add the xml version and encoding at top of run config file
    openFile = open(run_config, "w")
    tree.write(run_config)
    openFile.close()
    openFile = open(run_config, "r")
    lines = openFile.readlines()
    openFile.close()  
    openFile = open(run_config, "w")  
    line1 = ["<?xml version='1.0' encoding='UTF-8'?>\n"]
    line1.append(lines)
    openFile.writelines(lines)
    openFile.close()