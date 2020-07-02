#!/usr/bin/env python

from xml.etree.ElementTree import Element, SubElement, Comment, tostring
from xml.etree import ElementTree
import numpy as np
from math import log10, floor
from xml.dom import minidom
from scipy.stats import norm

def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

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
print(run_fog_starts)
print(run_fog_ends)

#Prop location parameters


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