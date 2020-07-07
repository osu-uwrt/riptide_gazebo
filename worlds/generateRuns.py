#!/usr/bin/env python

from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.etree import ElementTree
import numpy as np
from math import log10, floor
from xml.dom import minidom
from scipy.stats import norm

def round_sig(x, sig=1):
    return round(x, sig-int(floor(log10(abs(x))))-1)

#this script generates runs with varying random ambiences

#total number of runs
num_runs = 3

#Ambience parameters
amb_anchor = [0.01, 0.01, 0.01, 1]
amb_variation = 0.8
run_ambiences = []

#Fogs parameters
fog_profile = 'linear'
fog_start_mean = 0
fog_start_std = 200
fog_end_mean = 10000
fog_end_std = 5000
run_fog_starts = []
run_fog_ends = []

#generate fogs
run_fog_starts = np.random.normal(loc=fog_start_mean, scale=fog_start_std, size=num_runs)
run_fog_ends = np.random.normal(loc=fog_end_mean, scale=fog_end_std, size=num_runs)

#Prop location parameters
# {'prop_preset_name':[list of locations]}
# len(prop_locations['gate']) = number of props of type
props_locations_means = {'gate': [[7, 22, -1.5, 0, 0, 0]], 'cutie': [[2, 22, 0, 0, 0, 0],[2, 32, 0, 0, 0, 0]]}
props_locations_stds = {'gate': [[2, 2, 2, 1, 1, 1]], 'cutie': [[2, 2, 1, 1, 1, 1],[2, 2, 1, 1, 1, 1]]}


#generate location variations
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
    
#light
light_pose_mean = [0, 0, 0, 0, 0, 0]
light_pose_std = [1, 1, 1, 1, 1, 1]
light_diffuse_mean = [1, 1, 1, 1]
light_diffuse_std = [.1, .1, .1, .1]
light_direction_mean = [0, 0, -1]
light_direction_std = [0.2, 0.2, 0.2]
light_specular_mean = [0.1, 0.1, 0.1, 1]
light_specular_std = [0.01, 0.01, 0.01, 0.01]

#generate random lights
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

for i in range(num_runs):
    #generate ambiences
    ambience = []
    for anch_amb_comp in amb_anchor:
        if (anch_amb_comp - amb_variation <= 0):
            lower_bound = 0
        else:
            lower_bound = anch_amb_comp - amb_variation
        if (anch_amb_comp + amb_variation >= 1):
            upper_bound = 1
        else:
            upper_bound = anch_amb_comp + amb_variation
        amb_comp = np.random.uniform(lower_bound, upper_bound)
        ambience.append(round_sig(amb_comp))
    #ambience.append(1.0)
    run_ambiences.append(ambience)

#update run_config.xml
fileName = "run_config2.xml"
tree = ElementTree.parse(fileName)
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
    ambient_str = str(run_ambiences[ri])
    ambient_str = ambient_str.replace("[","")
    ambient_str = ambient_str.replace("]","")
    ambient_str = ambient_str.replace(",","")
    ambient.text = ambient_str
    
    if(run_fog_starts[ri] < 0):
        run_fog_starts[ri] = 0

    fog = ElementTree.SubElement(scene, 'fog')
    fog_type = ElementTree.SubElement(fog, 'type')
    fog_type.text = fog_profile
    fog_start = ElementTree.SubElement(fog, 'start')
    fog_start.text = str(int(run_fog_starts[ri]))
    fog_end = ElementTree.SubElement(fog, 'end')
    fog_end.text = str(int(run_fog_ends[ri]))

openFile = open(fileName, "w")
#tree = prettify(tostring(tree))
tree.write(openFile)
openFile.close()

with open(fileName, "r")  as f:
    lines = f.readlines()

newlines = []
newlines.append("<?xml version='1.0' encoding='UTF-8'?>\n")
newlines += lines

with open(fileName, "w") as f:
    f.writelines(newlines)