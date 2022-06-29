from pyexpat import model
from abaqus import *
from abaqusConstants import *
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
from operator import add
import numpy as np
import os

# Change the working director first of all.
os.chdir(r"C:\Users\acj249\Work Folders\Desktop\Abaqus_explicit\SRT_Abaqus")

# functions

# Create the ring quarter section based on the input given.
# Input thickness has to be the full thickness of the ring, routine takes care of calculations.
# Same goes for inner and outer radius, just give the inner and outer diameter. 
def SRTring(inner_rad,outer_rad,thickness,part,modelname):
    s = mdb.models[modelname].ConstrainedSketch(name='__profile__', sheetSize=20.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(0.0, inner_rad), point2=(inner_rad, 0.0), direction=CLOCKWISE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(0.0, outer_rad), point2=(outer_rad, 0.0), direction=CLOCKWISE)
    s.Line(point1=(0.0, outer_rad), point2=(0.0, inner_rad))
    s.VerticalConstraint(entity=g[4], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[3], entity2=g[4], addUndoState=False)
    s.Line(point1=(outer_rad, 0.0), point2=(inner_rad, 0.0))
    s.HorizontalConstraint(entity=g[5], addUndoState=False)
    s.PerpendicularConstraint(entity1=g[3], entity2=g[5], addUndoState=False)
    p = mdb.models[modelname].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[modelname].parts[part]
    p.BaseSolidExtrude(sketch=s, depth=thickness)
    s.unsetPrimaryObject()
    p = mdb.models[modelname].parts[part]
    del mdb.models[modelname].sketches['__profile__']

# Creates a full separate set of the Ring for outputting
def Create_full_set(setname,part,modelname):
    p = mdb.models[modelname].parts[part]
    c = p.cells[:]
    p.Set(cells=c, name=setname)

# Creates the pin. This ideally wouldn't need to be changed once set up.
# Function also turns the rigid body to a shell (removes cells)
def SRTpin(pin_radius, pin_thickness, pin_name, modelname):
    s = mdb.models[modelname].ConstrainedSketch(name='__profile__', sheetSize=20.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(pin_radius, 0.0))
    p = mdb.models[modelname].Part(name=pin_name, dimensionality=THREE_D, type=DISCRETE_RIGID_SURFACE)
    p = mdb.models[modelname].parts[pin_name]
    p.BaseSolidExtrude(sketch=s, depth=pin_thickness)
    s.unsetPrimaryObject()
    p = mdb.models[modelname].parts[pin_name]
    del mdb.models[modelname].sketches['__profile__']
    p = mdb.models[modelname].parts[pin_name]
    c1 = p.cells
    p.RemoveCells(cellList = c1[0:1])

# Creates the reference point that we need on the pin for force-displacement comparison
def Create_Reference_Point(x, y, z, modelname, partname, setname):
    a = mdb.models[modelname].parts[partname]
    myRP = a.ReferencePoint(point=(x, y, z))
    r = a.referencePoints
    myRP_Position = r.findAt((x, y, z),)
    refPoints1=(myRP_Position, )
    a.Set(referencePoints=refPoints1, name=setname)
    return myRP,myRP_Position

# Material parameters. Ramberg Osgood for implicit analysis.
def rambergosgood_oneparam(density,youngs,poissons,yieldstr,yieldoffset,hardexpo,mat_name,partname_ring,modelname):
    p1 = mdb.models[modelname].parts[partname_ring]
    mdb.models[modelname].Material(name=mat_name)
    mdb.models[modelname].materials[mat_name].Density(table=((density, ), ))
    mdb.models[modelname].materials[mat_name].DeformationPlasticity(table=((youngs, poissons, yieldstr, hardexpo, yieldoffset), ))

# Creates the "front" surface of the ring, i.e., the one we "see" in DIC.
def create_set_face_ring(modelname, partname, setname, x, y, z):
    face = ()
    p = mdb.models[modelname].parts[partname]
    f = p.faces
    ringface = f.findAt((x,y,z), )
    face = face + (f[ringface.index:ringface.index+1], )
    p.Set(faces=face, name=setname)
    return ringface

# Creates and assigns the associated section to the ring.
def RingSec_create_and_assign(setname, matname, sectionname,partname,modelname):
    p = mdb.models[modelname].parts[partname]
    mdb.models[modelname].HomogeneousSolidSection(name=sectionname, material=matname, thickness=None)
    p = mdb.models[modelname].parts[partname]
    c = p.cells
    regions = p.Set(cells=c, name=setname)
    p = mdb.models[modelname].parts[partname]
    p.SectionAssignment(region=regions, sectionName=sectionname, offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

# Creates the full assembly, along with the required movement of parts for standardization.
def full_assembly(modelname, part_ring, part_pin, dependency, ass_ring, ass_pin, zt_ring, zt_pin, yt_pin):
    a = mdb.models[modelname].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models[modelname].parts[part_ring]
    a.Instance(name=ass_ring, part=p, dependent=dependency)
    p = mdb.models[modelname].parts[part_pin]
    a.Instance(name=ass_pin, part=p, dependent=dependency)
    a = mdb.models[modelname].rootAssembly
    a.translate(instanceList=(ass_ring, ), vector=(0.0, 0.0, zt_ring))
    a = mdb.models[modelname].rootAssembly
    a.translate(instanceList=(ass_pin, ), vector=(0.0, 0.0, zt_pin))
    a = mdb.models[modelname].rootAssembly
    a.translate(instanceList=(ass_pin, ), vector=(0.0, yt_pin, 0.0))

# Creates analysis step.
def analysis_step(modelname, stepname, prevstep, tp, max_inc, initinc, min_inc, nlgeom_param):
    a = mdb.models[modelname].ImplicitDynamicsStep(name=stepname, previous=prevstep, timePeriod=tp, maxNumInc=max_inc, initialInc=initinc, minInc=min_inc, nlgeom=nlgeom_param)

# Defines contact property. Define fric_coeff as 0 if you want frictionless.
def contact_property(modelname, contactname, fric_coeff):
    a = mdb.models[modelname].ContactProperty(contactname)
    a = mdb.models[modelname].interactionProperties[contactname].TangentialBehavior(formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, pressureDependency=OFF, temperatureDependency=OFF, dependencies=0, table=((fric_coeff, ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, fraction=0.005, elasticSlipStiffness=None)

# Creates the surface sets on the ring to allot the XY, YZ, and XZ symmetry faces.
def Create_Set_Surface(modelname, partname, set_name, x,y,z):
    face = ()
    p = mdb.models[modelname].parts[partname]
    s = p.faces
    createdsurface = s.findAt((x,y,z),)
    face = face + (s[createdsurface.index:createdsurface.index+1], )
    p.Surface(side1Faces=face, name=set_name)
    return createdsurface

# Creates contact between ring and pin.
# Pin is primary surface, ring is secondary surface since pin is a rigid body. Do not change this.
def create_contact(modelname, pin, ring, contactname, pinsurf, ringsurf, firststep):
    a = mdb.models[modelname].rootAssembly
    region1=a.instances[pin].surfaces[pinsurf]
    a = mdb.models[modelname].rootAssembly
    region2=a.instances[ring].surfaces[ringsurf]
    mdb.models[modelname].SurfaceToSurfaceContactStd(
        name=contactname, createStepName=firststep, master=region1, slave=region2, 
        sliding=FINITE, thickness=ON, interactionProperty='tangential_friction', 
        adjustMethod=NONE, initialClearance=OMIT, datumAxis=None, 
        clearanceRegion=None)
    #: The interaction "fromhuiiii" has been created.

# Creates the symmetric BCs on the ring.
def BC_symm(modelname, ringname, name_Xsymm, name_Ysymm, name_Zsymm, surface_YZsymm, surface_XZsymm, surface_XYsymm, firststep):
    a = mdb.models[modelname].rootAssembly
    region = a.instances[ringname].sets[surface_YZsymm]
    mdb.models[modelname].XsymmBC(name=name_Xsymm, 
        createStepName=firststep, region=region, localCsys=None)
    a = mdb.models[modelname].rootAssembly
    region = a.instances[ringname].sets[surface_XZsymm]
    mdb.models[modelname].YsymmBC(name=name_Ysymm, 
        createStepName=firststep, region=region, localCsys=None)
    a = mdb.models[modelname].rootAssembly
    region = a.instances[ringname].sets[surface_XYsymm]
    mdb.models[modelname].ZsymmBC(name=name_Zsymm, 
        createStepName=firststep, region=region, localCsys=None)

# Creates the applied velocity load exerted by the pin on the ring.
def load_pin(modelname, pin, referencepoint, loadname, init_step, anal_step, velocity_up):
    a = mdb.models[modelname].rootAssembly
    region = a.instances[pin].sets[referencepoint]
    mdb.models[modelname].VelocityBC(name=loadname, 
        createStepName=init_step, region=region, v1=0.0, v2=0.0, v3=0.0, vr1=0.0, 
        vr2=0.0, vr3=0.0, amplitude=UNSET, localCsys=None, 
        distributionType=UNIFORM, fieldName='')
    mdb.models[modelname].boundaryConditions[loadname].setValuesInStep(
        stepName=anal_step, v2=velocity_up)

def field_and_history_outputs(modelname, ass_pin, ass_ring, set_ring, set_pin, stepname, timedump):
    a = mdb.models[modelname].rootAssembly
    mdb.models[modelname].fieldOutputRequests['F-Output-1'].setValues(
        variables=('S', 'MISES', 'MISESMAX', 'TRIAX', 'SEQUT', 'E', 'PE', 'PEEQ', 
        'PEEQT', 'PEMAG', 'LE', 'TE', 'TEEQ', 'TEVOL', 'EEQUT', 'UT', 'VT', 'AT', 
        'RT', 'CF', 'CSTRESS', 'CDISP', 'MVF'), timeInterval=timedump, region=MODEL, 
        exteriorOnly=OFF, sectionPoints=DEFAULT, rebar=EXCLUDE)
    regionDef=mdb.models[modelname].rootAssembly.allInstances[ass_ring].sets[set_ring]
    mdb.models[modelname].FieldOutputRequest(name='field_ringfront', 
        createStepName=stepname, variables=('S', 'MISES', 'MISESMAX', 'TRIAX', 
        'E', 'PEEQ', 'PEEQMAX', 'EE', 'LE', 'UT', 'UR', 'VT', 'VR', 'AT', 'AR', 
        'RT', 'RM'), timeInterval=timedump, region=regionDef, sectionPoints=DEFAULT, 
        rebar=EXCLUDE)
    regionDef=mdb.models[modelname].rootAssembly.allInstances[ass_pin].sets[set_pin]
    mdb.models[modelname].FieldOutputRequest(name='field_pin', 
        createStepName=stepname, variables=('UT', 'UR', 'VT', 'VR', 'AT', 'AR', 
        'RT', 'RM', 'TF'), timeInterval=timedump, region=regionDef, 
        sectionPoints=DEFAULT, rebar=EXCLUDE)
    regionDef=mdb.models[modelname].rootAssembly.allInstances[ass_pin].sets[set_pin]
    mdb.models[modelname].HistoryOutputRequest(name='history_pin', 
    createStepName='dynamic', variables=('U1', 'U2', 'U3', 'UR1', 'UR2', 'UR3', 
    'V1', 'V2', 'V3', 'VR1', 'VR2', 'VR3', 'A1', 'A2', 'A3', 'AR1', 'AR2', 
    'AR3', 'RF1', 'RF2', 'RF3', 'RM1', 'RM2', 'RM3', 'RT'), timeInterval=timedump, 
    region=regionDef, sectionPoints=DEFAULT, rebar=EXCLUDE)
    mdb.models[modelname].historyOutputRequests['H-Output-1'].setValues(
    timeInterval=timedump)

# Meshes the pin. Assembly is dependent on mesh so no extra steps are needed after this.
def pin_mesh(modelname, pin, seedsize):
    p = mdb.models[modelname].parts[pin]
    elemType1 = mesh.ElemType(elemCode=R3D4, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=R3D3, elemLibrary=STANDARD)
    faces = p.faces[:]
    pickedRegions =(faces, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))
    p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()

# Meshes the ring. 
def ring_mesh(modelname, ring, seedsize):
    p = mdb.models[modelname].parts[ring]
    elemType1 = mesh.ElemType(elemCode=C3D20R, elemLibrary=STANDARD, elemDeletion=ON)
    elemType2 = mesh.ElemType(elemCode=C3D15, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D10, elemLibrary=STANDARD)
    cells = p.cells
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    p.seedPart(size=seedsize, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()

# Job created. This is the function to modify if you want to run multi-threaded on HPC.
def job_create(modelname, jobname):
    a = mdb.models[modelname].rootAssembly
    mdb.Job(name=jobname, model=modelname, description='', 
        type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, 
        memory=90, memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
        explicitPrecision=SINGLE, nodalOutputPrecision=FULL, echoPrint=OFF, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
        scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=2, 
        numDomains=2, numGPUs=1)

# Submit the job. 
def job_submit(jobname):
    mdb.jobs[jobname].submit()
    mdb.jobs[jobname].waitForCompletion()


# variables

#-----------------------------------
# Pin and ring dimensions
#-----------------------------------
inn_rad = 5.0
out_rad = 6.0
thick = 1.0
part_fromcall = "SRT_frompython"
setname_fromcall = "All_cells_selected"
pin_fromcall = "pin_from_script"
radius_for_pin = 1.0
thickness_for_pin = 3.0
modelname_fromcall = "Model_name_for_python"
#-----------------------------------
# Material parameters
#-----------------------------------
density = 7.85e-6
elastic_modulus = 290000
poissons_ratio = 0.33
yield_strength = 245
yield_offset = 0.16
hardening_exponent = 6.45
material_name = "SingleParam_RambergOsgood"
#-----------------------------------
# Pin and ring face surface set coords
#-----------------------------------
x_ringfront = out_rad-((out_rad-inn_rad)/2)
y_ringfront = 0.0
z_ringfront = thick
facename_ringfront = "ring_frontface"

x_ringback = x_ringfront
y_ringback = 0.0
z_ringback = 0.0
facename_ringback = "ring_backface_XYsymm"

x_ring_YZ = 0.0
y_ring_YZ = out_rad-((out_rad-inn_rad)/2)
z_ring_YZ = (thick/2)
facename_ring_YZ = "ring_sideface_YZsymm"

x_ring_XZ = out_rad-((out_rad-inn_rad)/2)
y_ring_XZ = 0.0
z_ring_XZ = (thick/2)
facename_ring_XZ = "ring_sideface_XZsymm"

x_ring_innercurved = inn_rad
y_ring_innercurved = 0.0
z_ring_innercurved = (thick/2)
facename_ring_innercurved = "ring_curved_inner"

x_pin_outercurved = 0.0
y_pin_outercurved = radius_for_pin
z_pin_outercurved = (thickness_for_pin/2)
facename_pin_outercurved = "pin_curvedsurface"
#-----------------------------------
# Section and assembly creation
#-----------------------------------
section_set_name = "Set_Sec"
section_name = "Section_from_python"

assembly_ringname = "SRT_frompython1"
assembly_pinname = "pin_from_script1"
depend = True
ytrans_pin = (inn_rad-radius_for_pin)
ztrans_pin = (-(thickness_for_pin/2))
ztrans_ring = (-(thick/2))
#-----------------------------------
# Step Created
#-----------------------------------
step_name = "dynamic"
first_step = "Initial"
total_time = 100.0 # in seconds
maximum_increment = 10000 # number of increments allowed
minimum_increment = 0.0001 # in seconds
initial_increment = 0.01 # in seconds
nlgeom_paramater = True # do we want model buckling? True for yes
#-----------------------------------
# Contact
#-----------------------------------
contact_name = "tangential_friction"
friction_coefficient = 0.2
#-----------------------------------
# Interaction
#-----------------------------------
part_contact_property = "pin_ring_interaction"
#-----------------------------------
# Boundary condition
#-----------------------------------
name_YZsymm = "Xsymm_about_YZ"
name_XZsymm = "Ysymm_about_XZ"
name_XYsymm = "Zsymm_about_XY"
#-----------------------------------
# Reference point
#-----------------------------------
refpos = []
rp_name = "Reference_pt_fromscript"
#-----------------------------------
# Load
#-----------------------------------
pin_velocity = 0.2 # in mm/s
BC_loadname = "pin_moving_up"
#-----------------------------------
# Mesh params
#-----------------------------------
seed_pin = 0.2
seed_ring = 0.1
#-----------------------------------
# Job params
#-----------------------------------
job_name = 'job_from_script'
#-----------------------------------
# Output requests
#-----------------------------------
timeperiod = 0.001
#-----------------------------------
# Model created
#-----------------------------------
the_model = mdb.Model(name=modelname_fromcall)
#-----------------------------------
# Function calls
#-----------------------------------

SRTring(inn_rad, out_rad, thick, part_fromcall, modelname_fromcall)
SRTpin(radius_for_pin,thickness_for_pin,pin_fromcall,modelname_fromcall)
Create_full_set(setname_fromcall,part_fromcall,modelname_fromcall)
ringface_front = create_set_face_ring(modelname_fromcall,part_fromcall,facename_ringfront,x_ringfront,y_ringfront,z_ringfront)
ringface_back_symmXY = create_set_face_ring(modelname_fromcall,part_fromcall,facename_ringback,x_ringback,y_ringback,z_ringback)
ringface_side_symmYZ = create_set_face_ring(modelname_fromcall,part_fromcall,facename_ring_YZ,x_ring_YZ,y_ring_YZ,z_ring_YZ)
ringface_side_symmXZ = create_set_face_ring(modelname_fromcall,part_fromcall,facename_ring_XZ,x_ring_XZ,y_ring_XZ,z_ring_XZ)
ringface_curved_inner = create_set_face_ring(modelname_fromcall,part_fromcall,facename_ring_innercurved,x_ring_innercurved,y_ring_innercurved,z_ring_innercurved)
pinface_curved_outer = create_set_face_ring(modelname_fromcall,pin_fromcall,facename_pin_outercurved,x_pin_outercurved, y_pin_outercurved, z_pin_outercurved)
surface_ringface_front = Create_Set_Surface(modelname_fromcall,part_fromcall,facename_ringfront,x_ringfront,y_ringfront,z_ringfront)
surface_ringface_back_symmXY = Create_Set_Surface(modelname_fromcall,part_fromcall,facename_ringback,x_ringback,y_ringback,z_ringback)
surface_ringface_side_symmYZ = Create_Set_Surface(modelname_fromcall,part_fromcall,facename_ring_YZ,x_ring_YZ,y_ring_YZ,z_ring_YZ)
surface_ringface_side_symmXZ = Create_Set_Surface(modelname_fromcall,part_fromcall,facename_ring_XZ,x_ring_XZ,y_ring_XZ,z_ring_XZ)
surface_ringface_curved_inner = Create_Set_Surface(modelname_fromcall,part_fromcall,facename_ring_innercurved,x_ring_innercurved,y_ring_innercurved,z_ring_innercurved)
surface_pinface_curved_outer = Create_Set_Surface(modelname_fromcall,pin_fromcall,facename_pin_outercurved,x_pin_outercurved, y_pin_outercurved, z_pin_outercurved)
rambergosgood_oneparam(density, elastic_modulus, poissons_ratio, yield_strength, yield_offset, hardening_exponent, material_name, part_fromcall, modelname_fromcall)
RingSec_create_and_assign(section_set_name, material_name, section_name, part_fromcall, modelname_fromcall)
full_assembly(modelname_fromcall, part_fromcall, pin_fromcall, depend, assembly_ringname, assembly_pinname, ztrans_ring, ztrans_pin,ytrans_pin)
analysis_step(modelname_fromcall, step_name, first_step, total_time, maximum_increment, initial_increment, minimum_increment, nlgeom_paramater)
contact_property(modelname_fromcall, contact_name, friction_coefficient)
create_contact(modelname_fromcall, assembly_pinname, assembly_ringname, part_contact_property, facename_pin_outercurved, facename_ring_innercurved, first_step)
BC_symm(modelname_fromcall, assembly_ringname, name_YZsymm, name_XZsymm, name_XYsymm, facename_ring_YZ, facename_ring_XZ, facename_ringback, first_step)
rp, refpos = Create_Reference_Point(0.0,0.0,thickness_for_pin,modelname_fromcall,pin_fromcall,rp_name)
load_pin(modelname_fromcall, assembly_pinname, rp_name, BC_loadname, first_step, step_name, pin_velocity)
field_and_history_outputs(modelname_fromcall, assembly_pinname, assembly_ringname, facename_ringfront, rp_name, step_name, timeperiod)
pin_mesh(modelname_fromcall, pin_fromcall, seed_pin)
ring_mesh(modelname_fromcall, part_fromcall, seed_ring)
job_create(modelname_fromcall, job_name)
job_submit(job_name)
