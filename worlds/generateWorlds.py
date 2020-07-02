#!/usr/bin/env python

from xml.dom import minidom
from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.etree import ElementTree
import shutil

#using a template world file, adds ambient0

#runs: list of runs defined in run_config.xml

#index: outside iterator

#ambiences: a list of ambiences for each run

#props_poses: a list of lists of poses for each run
    #len(props_poses) = number of prop presets
    #len(props_poses[0]) = number of poses of a prop preset
    #props_poses[0][0] = first pose of first prop of preset

#presets: a list of strings, each the name of a preset model 
    #xml for the world file

def buildWorldFile(index, ambiences, props_poses, presets, fog):
    fileName = "run" + str(index) + ".world"
    runWorldFile = open(fileName, "w")

    tree = ElementTree.parse('run_template.xml')
    world = tree.getroot()

    for scene in world.findall('scene'):
        #Ambient
        ambient = ElementTree.SubElement(scene, 'ambient')
        ambient.text = ambiences[index]

        #Fog
        scene.append(fog)

    models_template = ElementTree.parse('models_template.xml')
    models = models_template.getroot()
    
    #Props
    for i in range(len(props_poses)):
        for preset in models.findall('model'):
            preset_inst = 0
            if (preset.attrib['name'] == presets[i]['preset']):
                pose_count = 0
                for pose in props_poses[i]:
                    models_template = ElementTree.parse('models_template.xml')
                    models = models_template.getroot()
                    for preset_ in models_template.findall('model'):
                        #print(preset_.attrib['name'])
                        print(presets[i]['preset'])

                        if (preset_.attrib['name'] == presets[i]['preset']):
                            prop_pose = ElementTree.SubElement(preset_, 'pose', {'frame':''})
                            prop_pose.text = pose
                            pose_count += 1
                            name = presets[i]['preset'] + str(pose_count)
                            preset_.set('name',name)
                            world.append(preset_)
                            print('     ' + name + ':  ' + str(pose))
                preset_inst += 1



    tree.write(fileName)
    runWorldFile.close()

    with open(fileName, "r")  as f:
        lines = f.readlines()
    
    newlines = []
    newlines.append("<sdf version='1.6'>\n")
    newlines += lines
    newlines.append("</sdf>")
    
    with open(fileName, "w") as f:
        f.writelines(newlines)

    runWorldFile.close()

# read the configuration file in scripts/run_config.xml
run_config = ElementTree.parse('run_config.xml')
runs = run_config.getroot()

ambiences = []

n_runs = 0

props_poses = []
presets = []

for run in runs.findall('run'):
    
    n_runs += 1
    
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
    print(presets)

    #TODO: Lights

for i in range(n_runs):
    buildWorldFile(i, ambiences, props_poses, presets, fog)
