#!/usr/bin/env python3

'''Do not edit this script for general usage'''

from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.etree import ElementTree
import shutil
import os

''' 
This script uses the run_template.xml and run_config.xml to generate n number of worlds. 

runs: list of runs defined in run_config.xml
ambiences: a list of xml snippets for ambiences for each run
lights: a list of xml snippets for lights for each run
prop_poses: a list of lists of poses for each run
    #len(props_poses) = number of prop presets
    #len(props_poses[0]) = number of poses of a prop preset
    #props_poses[0][0] = first pose of first prop of preset
presets: a list of strings, each the name of a preset model 
    #xml for the world file

'''

def buildWorldFile(run_config, index, ambiences, lights, props_poses, presets, fog):
    file_name = "/worlds/run" + str(index) + ".world"
    currentPath = os.getcwd()
    world_file_path = os.path.abspath(os.path.join(currentPath,os.pardir)) + file_name
    runWorldFile = open(world_file_path, "w")
    print('--- Run ' + str(index) + ' ---')
    tree = ElementTree.parse('run_template.xml')
    world = tree.getroot()

    #light
    world.append(lights[index])

    for scene in world.findall('scene'):
        #Ambient
        ambient = ElementTree.SubElement(scene, 'ambient')
        ambient.text = ambiences[index]
        print('ambient: ' + ambient.text)

        #Fog
        scene.append(fog)

    props_template = ElementTree.parse('props_template.xml')
    models = props_template.getroot()
    
    #Props
    for i in range(len(props_poses)):
        for preset in models.findall('model'):
            preset_inst = 0
            if (preset.attrib['name'] == presets[i]['preset']):
                pose_count = 0
                for pose in props_poses[i]:
                    props_template = ElementTree.parse('props_template.xml')
                    models = props_template.getroot()
                    for preset_ in props_template.findall('model'):
                        #print(preset_.attrib['name'])
                        #print(presets[i]['preset'])
                        if (preset_.attrib['name'] == presets[i]['preset']):
                            prop_pose = ElementTree.SubElement(preset_, 'pose', {'frame':''})
                            prop_pose.text = pose
                            pose_count += 1
                            name = presets[i]['preset'] + str(pose_count)
                            preset_.set('name',name)
                            world.append(preset_)
                            print('' + name + ':  ' + str(pose))
                preset_inst += 1

    #change from xml to sdf format
    tree.write(world_file_path)
    runWorldFile.close()
    with open(world_file_path, "r")  as f:
        lines = f.readlines()
    newlines = []
    newlines.append("<sdf version='1.6'>\n")
    newlines += lines
    newlines.append("</sdf>")
    with open(world_file_path, "w") as f:
        f.writelines(newlines)
    runWorldFile.close()

# read the configuration file in scripts/run_config
def read_run_config_and_build(run_config):
    run_config = ElementTree.parse(run_config)
    runs = run_config.getroot()
    n_runs = 0
    ambiences = []
    lights = []
    props_poses = []
    presets = []

    for run in runs.findall('run'):
        n_runs += 1
        
        #light
        light = run.findall('light')[0]
        lights.append(light)

        #Ambience
        scene = run.findall('scene')[0]
        ambient = str(scene.findall('ambient')[0].text)
        ambiences.append(ambient)

        #Fog
        fog = scene.findall('fog')[0]

        #Props
        for prop in run.findall('prop'):
            preset = prop.attrib
            presets.append(preset)
            poses = []
            for pose in prop.findall('pose'):
                pose = str(pose.text)
                poses.append(pose)
            props_poses.append(poses)
        #print(presets)

    for i in range(n_runs):
        buildWorldFile(run_config, i, ambiences, lights, props_poses, presets, fog)
