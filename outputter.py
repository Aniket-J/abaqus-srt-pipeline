import sys
import regionToolset
import __main__
import section
import regionToolset
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import connectorBehavior
import odbAccess
import numpy as np
import os
from pyexpat import model
from abaqus import *
from abaqusConstants import *
from operator import add

step_name = "dynamic"
job_name = 'job_from_script'

odbloc = 'C:\\Users\\acj249\\Documents\\GitHub\\abaqus-srt-pipeline\'
step_name = "dynamic"
job_name = 'job_from_script'
def odb_history(outputloc, jobname, step):
    odb = jobname + '.odb'
    odb = odbAccess.openOdb(odb)
    rp_region = 'Node PIN_FROM_SCRIPT1.697' #Based on the mesh size we gave, no need to change this
    filename = 'rp_output_function'
    u2 = np.array(odb.steps[step].historyRegions[rp_region].historyOutputs['U2'].data)
    r2 = np.array(odb.steps[step].historyRegions[rp_region].historyOutputs['RF2'].data)
    rp_output = np.concatenate((u2, r2), axis = 1) #Axis = 1 for column-wise appending
    np.savetxt(filename + '.csv', rp_output, delimiter = ',') #CSV is now Time, U2, Time, RF2

odb_history(odbloc, job_name, step_name)

def odb_field(outputloc, jobname, step, nodal_set):
    odb = jobname + '.odb'
    odb = odbAccess.openOdb(odb)
    last = odb.steps[step].frames[-1]
    for x in range(len(last.data)):