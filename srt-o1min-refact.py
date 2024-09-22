# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

def create_sketch_ring(mdb, model_name):
    """Creates the sketch for the deformable Ring part."""
    sketch_name = '__profile__'
    model = mdb.models[model_name]
    model.ConstrainedSketch(name=sketch_name, sheetSize=20.0)
    sketch = model.sketches[sketch_name]

    # Define arcs and lines
    sketch.ArcByCenterEnds(center=(0.0, 0.0), direction=CLOCKWISE, 
                           point1=(0.0, 6.0), point2=(6.25, 0.0))
    sketch.ArcByCenterEnds(center=(0.0, 0.0), direction=COUNTERCLOCKWISE, 
                           point1=(5.0, 0.0), point2=(0.0, 5.0))
    sketch.Line(point1=(0.0, 6.0), point2=(0.0, 5.0))
    sketch.VerticalConstraint(entity=sketch.geometry[4])
    sketch.PerpendicularConstraint(entity1=sketch.geometry[2], entity2=sketch.geometry[4])
    sketch.Line(point1=(5.0, 0.0), point2=(6.0, 0.0))
    sketch.HorizontalConstraint(entity=sketch.geometry[5])
    sketch.PerpendicularConstraint(entity1=sketch.geometry[3], entity2=sketch.geometry[5])

    # Clean up the first sketch
    del model.sketches[sketch_name]

    # Create a new sketch with additional dimensions
    model.ConstrainedSketch(name=sketch_name, sheetSize=20.0)
    sketch = model.sketches[sketch_name]

    sketch.ArcByCenterEnds(center=(0.0, 0.0), direction=COUNTERCLOCKWISE, 
                           point1=(5.0, 0.0), point2=(0.0, 4.875))
    sketch.ArcByCenterEnds(center=(0.0, 0.0), direction=CLOCKWISE, 
                           point1=(0.0, 6.0), point2=(5.875, 0.0))
    sketch.Line(point1=(5.0, 0.0), point2=(6.0, 0.0))
    sketch.HorizontalConstraint(entity=sketch.geometry[4])
    sketch.PerpendicularConstraint(entity1=sketch.geometry[2], entity2=sketch.geometry[4])
    sketch.Line(point1=(0.0, 5.0), point2=(0.0, 6.0))
    sketch.VerticalConstraint(entity=sketch.geometry[5])
    sketch.PerpendicularConstraint(entity1=sketch.geometry[2], entity2=sketch.geometry[5])

    # Add dimensions
    sketch.RadialDimension(curve=sketch.geometry[2], radius=5.0, 
                           textPoint=(1.18, 2.61))
    sketch.RadialDimension(curve=sketch.geometry[3], radius=6.0, 
                           textPoint=(5.69, 4.79))
    sketch.ObliqueDimension(textPoint=(5.48, -1.30), value=1.0, 
                            vertex1=sketch.vertices[0], vertex2=sketch.vertices[4])
    sketch.undo()

def create_sketch_pin(mdb, model_name):
    """Creates the sketch for the rigid Pin part."""
    sketch_name = '__profile__'
    model = mdb.models[model_name]
    model.ConstrainedSketch(name=sketch_name, sheetSize=20.0)
    sketch = model.sketches[sketch_name]

    # Define circle
    sketch.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(0.0, 0.5))
    sketch.RadialDimension(curve=sketch.geometry[2], radius=1.0, 
                           textPoint=(-2.57, 0.16))

def create_part_ring(mdb, model_name):
    """Creates the deformable Ring part."""
    create_sketch_ring(mdb, model_name)
    model = mdb.models[model_name]
    sketch = model.sketches['__profile__']
    model.Part(dimensionality=THREE_D, name='Ring', type=DEFORMABLE_BODY)
    ring_part = model.parts['Ring']
    ring_part.BaseSolidExtrude(depth=1.0, sketch=sketch)
    del model.sketches['__profile__']

def create_part_pin(mdb, model_name):
    """Creates the rigid Pin part."""
    create_sketch_pin(mdb, model_name)
    model = mdb.models[model_name]
    sketch = model.sketches['__profile__']
    model.Part(dimensionality=THREE_D, name='Pin', type=DISCRETE_RIGID_SURFACE)
    pin_part = model.parts['Pin']
    pin_part.BaseSolidExtrude(depth=4.0, sketch=sketch)
    del model.sketches['__profile__']
    # Remove unnecessary cells
    pin_part.RemoveCells(cellList=pin_part.cells.getSequenceFromMask(mask=('[#1 ]', )))

def define_materials(mdb, model_name):
    """Defines materials and sections."""
    model = mdb.models[model_name]
    # Define material for Ring
    model.Material(name='Ring_StainlessSteel')
    model.materials['Ring_StainlessSteel'].DeformationPlasticity(
        table=((210000.0, 0.3, 450.0, 6.73, 0.15), )
    )
    model.HomogeneousSolidSection(material='Ring_StainlessSteel', 
                                  name='RingSec', thickness=None)

def assign_sections(mdb, model_name):
    """Assigns sections to parts."""
    model = mdb.models[model_name]
    ring_part = model.parts['Ring']
    # Assign section to Ring
    ring_part.Set(cells=ring_part.cells.getSequenceFromMask(('[#1 ]', )), name='WholeRing')
    ring_part.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE, 
                               region=ring_part.sets['WholeRing'], sectionName='RingSec', 
                               thicknessAssignment=FROM_SECTION)

def create_assembly(mdb, model_name):
    """Assembles the parts into the root assembly."""
    model = mdb.models[model_name]
    assembly = model.rootAssembly
    # Define Datum coordinate system
    assembly.DatumCsysByDefault(CARTESIAN)
    # Create instances
    pin_instance = assembly.Instance(dependent=ON, name='Pin-1', part=model.parts['Pin'])
    ring_instance = assembly.Instance(dependent=ON, name='Ring-1', part=model.parts['Ring'])
    # Translate Pin
    assembly.translate(instanceList=('Pin-1', ), vector=(0.0, 0.0, -1.5))
    assembly.translate(instanceList=('Pin-1', ), vector=(0.0, 3.9, 0.0))
    # Create Reference Points
    ring_part = model.parts['Ring']
    pin_part = model.parts['Pin']
    ring_part.ReferencePoint(point=ring_part.vertices[3])
    pin_part.ReferencePoint(point=pin_part.InterestingPoint(pin_part.edges[0], CENTER))
    # Define Sets
    ring_instance = assembly.instances['Ring-1']
    pin_instance = assembly.instances['Pin-1']
    assembly.Set(faces=ring_instance.faces.getSequenceFromMask(('[#2 ]', )), name='XSymm')
    assembly.Set(faces=ring_instance.faces.getSequenceFromMask(('[#20 ]', )), name='ZSymm')
    assembly.Set(faces=ring_instance.faces.getSequenceFromMask(('[#8 ]', )), name='YSymm')
    assembly.Set(faces=ring_instance.faces.getSequenceFromMask(('[#1 ]', )), name='RingInner')
    assembly.Set(faces=pin_instance.faces.getSequenceFromMask(('[#1 ]', )), name='PinOuter')

def define_steps(mdb, model_name):
    """Defines analysis steps."""
    model = mdb.models[model_name]
    model.ImplicitDynamicsStep(initialInc=0.02, maxNumInc=5000, minInc=0.02, 
                               name='SRT_simstep', nlgeom=ON, previous='Initial', 
                               timePeriod=100.0)

def define_interactions(mdb, model_name):
    """Defines contact interactions."""
    model = mdb.models[model_name]
    # Define contact property
    model.ContactProperty('ContactProperty_Friction')
    model.interactionProperties['ContactProperty_Friction'].TangentialBehavior(
        dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None, 
        formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION, 
        pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
        table=((0.2, ), ), temperatureDependency=OFF)
    # Define surfaces
    assembly = model.rootAssembly
    assembly.Surface(name='RingOuter', side1Faces=assembly.instances['Pin-1'].faces.getSequenceFromMask(('[#1 ]', )))
    assembly.Surface(name='RingInner', side1Faces=assembly.instances['Ring-1'].faces.getSequenceFromMask(('[#1 ]', )))
    # Define contact interaction
    model.SurfaceToSurfaceContactStd(adjustMethod=NONE, clearanceRegion=None, 
                                     createStepName='SRT_simstep', datumAxis=None, 
                                     initialClearance=OMIT, interactionProperty='ContactProperty_Friction', 
                                     main=assembly.surfaces['RingOuter'], name='RingPin_Interaction', 
                                     secondary=assembly.surfaces['RingInner'], sliding=FINITE, thickness=ON)

def define_boundary_conditions(mdb, model_name):
    """Defines boundary conditions and symmetry conditions."""
    model = mdb.models[model_name]
    assembly = model.rootAssembly
    # Define symmetry sets
    assembly.Set(faces=assembly.instances['Ring-1'].faces.getSequenceFromMask(('[#2 ]', )), name='BC_XSymm')
    assembly.Set(faces=assembly.instances['Ring-1'].faces.getSequenceFromMask(('[#8 ]', )), name='BC_YSymm')
    assembly.Set(faces=assembly.instances['Ring-1'].faces.getSequenceFromMask(('[#20 ]', )), name='BC_ZSymm')
    # Apply symmetry boundary conditions
    model.XsymmBC(createStepName='Initial', localCsys=None, name='XSymm', 
                 region=assembly.sets['BC_XSymm'])
    model.YsymmBC(createStepName='Initial', localCsys=None, name='YSymm', 
                 region=assembly.sets['BC_YSymm'])
    model.ZsymmBC(createStepName='Initial', localCsys=None, name='ZSymm', 
                 region=assembly.sets['BC_ZSymm'])
    # Define velocity boundary condition for Pin
    assembly.Set(name='BC_PinRP_Vel', referencePoints=(assembly.instances['Pin-1'].referencePoints[3], ))
    model.VelocityBC(amplitude=UNSET, createStepName='Initial', distributionType=UNIFORM, 
                    fieldName='', localCsys=None, name='Pin_Vel', 
                    region=assembly.sets['BC_PinRP_Vel'], v1=UNSET, v2=0.0, 
                    v3=UNSET, vr1=UNSET, vr2=UNSET, vr3=UNSET)
    # Update velocity in simulation step
    model.boundaryConditions['Pin_Vel'].setValuesInStep(stepName='SRT_simstep', v2=0.008333)

def mesh_parts(mdb, model_name):
    """Meshes the Ring and Pin parts."""
    model = mdb.models[model_name]
    # Mesh Ring
    ring_part = model.parts['Ring']
    ring_part.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=0.2)
    ring_part.generateMesh()
    ring_part.setElementType(elemTypes=(ElemType(elemCode=C3D20R, elemLibrary=STANDARD), 
                                        ElemType(elemCode=C3D15, elemLibrary=STANDARD), 
                                        ElemType(elemCode=C3D10, elemLibrary=STANDARD)), 
                             regions=(ring_part.cells.getSequenceFromMask(('[#1 ]', ), ), ))
    # Mesh Pin
    pin_part = model.parts['Pin']
    pin_part.seedPart(deviationFactor=0.1, minSizeFactor=0.1, size=0.3)
    pin_part.generateMesh()
    pin_part.setElementType(elemTypes=(ElemType(elemCode=R3D4, elemLibrary=STANDARD), 
                                       ElemType(elemCode=R3D3, elemLibrary=STANDARD)), 
                            regions=(pin_part.faces.getSequenceFromMask(('[#7 ]', ), ), ))

def create_job(mdb, model_name, job_name='JobbyJob'):
    """Creates the Abaqus job."""
    mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
            explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF, 
            memory=90, memoryUnits=PERCENTAGE, model=model_name, modelPrint=OFF, 
            name=job_name, nodalOutputPrecision=SINGLE, queue=None, resultsFormat=ODB, 
            scratch='', type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
    mdb.jobs['JobbyJob'].submit(consistencyChecking=OFF)

def main():
    """Main function to set up and create the Abaqus model."""
    model_name = 'Model-1'
    job_name = 'JobbyJob'
    
    # Create parts
    create_part_ring(mdb, model_name)
    create_part_pin(mdb, model_name)
    
    # Define materials and assign sections
    define_materials(mdb, model_name)
    assign_sections(mdb, model_name)
    
    # Assemble the model
    create_assembly(mdb, model_name)
    
    # Define analysis steps
    define_steps(mdb, model_name)
    
    # Define interactions
    define_interactions(mdb, model_name)
    
    # Define boundary conditions
    define_boundary_conditions(mdb, model_name)
    
    # Mesh the parts
    mesh_parts(mdb, model_name)
    
    # Regenerate assembly to update changes
    mdb.models[model_name].rootAssembly.regenerate()
    
    # Create the job
    create_job(mdb, model_name, job_name)

# Execute the main function
if __name__ == '__main__':
    main()
