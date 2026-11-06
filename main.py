# -*- coding: mbcs -*-
# Abaqus 2018 Student Edition
# Riveted plate joint: plate with two rivets under distributed load.
# Static linear perturbation analysis with contact.
#
# Run from Abaqus Command:
#   abaqus cae noGUI=main.py
#
# Lab: Calculate stress at point E (center of plate) and verify contact closure.

from abaqus import *
from abaqusConstants import *
import regionToolset
import mesh
import visualization
import os
import traceback
from math import hypot

# ============================================================
# USER CONFIGURATION
# ============================================================

MODEL_NAME = 'RivetedPlate'
JOB_NAME = 'RivetedPlate_Job'

# Geometry, mm (Unit system: mm-N-MPa)
PLATE_LENGTH = 150.0
PLATE_HEIGHT = 50.0
PLATE_THICKNESS = 5.0           # out-of-plane thickness

# Hole geometry
HOLE_RADIUS = 10.0              # Ø 20 mm
RIVET_OUTER_RADIUS = 10.0       # Ø 20 mm
RIVET_INNER_RADIUS = 4.0        # Ø 8 mm (central hole)
RIVET_THICKNESS = 5.0           # Same as plate

# Hole positions (symmetric)
HOLE_LEFT_X = 30.0              # 3 cm from left edge
HOLE_LEFT_Y = 25.0              # 2.5 cm from bottom edge
HOLE_RIGHT_X = PLATE_LENGTH - 30.0  # 3 cm from right edge (120 mm)
HOLE_RIGHT_Y = 25.0             # 2.5 cm from bottom edge (symmetric)

# Point E (midpoint of plate, for stress extraction)
POINT_E_X = PLATE_LENGTH / 2.0  # 75 mm
POINT_E_Y = HOLE_LEFT_Y         # 25 mm (center line horizontally)

# Material (N-mm-MPa)
YOUNG_MODULUS = 200000.0   # 200e9 Pa = 200000 MPa
POISSON_RATIO = 0.3

# Load: distributed pressure on top edge (AB)
PRESSURE = 1000.0               # N/mm^2 on edge AB
TOTAL_FORCE = PRESSURE * PLATE_LENGTH  # Total force in N

# Analysis type
ANALYSIS_TYPE = 'STATIC_LINEAR_PERTURBATION'

# Mesh
MESH_SIZE = 4.0                 # mm, to keep node count < 1000

# Output
OUTPUT_DIRECTORY = os.getcwd()

# ============================================================
# HELPERS
# ============================================================

def log(msg):
    print('[RivetedPlate] ' + str(msg))

def ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)

def ring_edges(inst, cx, cy, r):
    """Rёbra (edges) of instance inst lying on circle radius r centered at (cx, cy)."""
    pts = []
    for e in inst.edges:
        p = e.pointOn[0]
        if abs(hypot(p[0] - cx, p[1] - cy) - r) < 0.5:
            pts.append(p)
    return inst.edges.findAt(coordinates=tuple(pts))

def create_circle_sketch(sketch, cx, cy, r, label='Circle'):
    """Create a circle centered at (cx, cy) with radius r."""
    sketch.Arc3Points(point1=(cx + r, cy), point2=(cx - r, cy),
                      point3=(cx, cy + r))
    sketch.Arc3Points(point1=(cx - r, cy), point2=(cx + r, cy),
                      point3=(cx, cy - r))

def create_circle_with_hole_sketch(sketch, cx, cy, r_outer, r_inner):
    """Create annulus (ring) for rivet: outer radius r_outer, inner hole r_inner."""
    # Outer circle
    sketch.Arc3Points(point1=(cx + r_outer, cy), point2=(cx - r_outer, cy),
                      point3=(cx, cy + r_outer))
    sketch.Arc3Points(point1=(cx - r_outer, cy), point2=(cx + r_outer, cy),
                      point3=(cx, cy - r_outer))
    # Inner circle (hole)
    sketch.Arc3Points(point1=(cx + r_inner, cy), point2=(cx - r_inner, cy),
                      point3=(cx, cy + r_inner))
    sketch.Arc3Points(point1=(cx - r_inner, cy), point2=(cx + r_inner, cy),
                      point3=(cx, cy - r_inner))

def create_model():
    if MODEL_NAME in mdb.models:
        del mdb.models[MODEL_NAME]
    model = mdb.Model(name=MODEL_NAME)

    # ----------------------------
    # PART 1: Plate with two holes
    # ----------------------------
    log('Creating plate with holes...')
    sk_plate = model.ConstrainedSketch(name='PlateSketch',
                                       sheetSize=PLATE_LENGTH * 1.5)
    
    # Outer boundary (rectangle)
    sk_plate.Line(point1=(0.0, 0.0), point2=(PLATE_LENGTH, 0.0))
    sk_plate.Line(point1=(PLATE_LENGTH, 0.0), point2=(PLATE_LENGTH, PLATE_HEIGHT))
    sk_plate.Line(point1=(PLATE_LENGTH, PLATE_HEIGHT), point2=(0.0, PLATE_HEIGHT))
    sk_plate.Line(point1=(0.0, PLATE_HEIGHT), point2=(0.0, 0.0))
    
    # Inner circles (holes) - create as separate closed profiles
    create_circle_sketch(sk_plate, HOLE_LEFT_X, HOLE_LEFT_Y, HOLE_RADIUS)
    create_circle_sketch(sk_plate, HOLE_RIGHT_X, HOLE_RIGHT_Y, HOLE_RADIUS)
    
    plate = model.Part(name='Plate', dimensionality=TWO_D_PLANAR,
                       type=DEFORMABLE_BODY)
    plate.BaseShell(sketch=sk_plate)
    del model.sketches['PlateSketch']
    
    # ----------------------------
    # PART 2: Rivet (single part, two instances)
    # ----------------------------
    log('Creating rivet...')
    sk_rivet = model.ConstrainedSketch(name='RivetSketch', sheetSize=50.0)
    create_circle_with_hole_sketch(sk_rivet, 0.0, 0.0,
                                    RIVET_OUTER_RADIUS, RIVET_INNER_RADIUS)
    
    rivet = model.Part(name='Rivet', dimensionality=TWO_D_PLANAR,
                       type=DEFORMABLE_BODY)
    rivet.BaseShell(sketch=sk_rivet)
    del model.sketches['RivetSketch']
    
    # ----------------------------
    # Material
    # ----------------------------
    mat = model.Material(name='Material')
    mat.Elastic(table=((YOUNG_MODULUS, POISSON_RATIO),))
    
    model.HomogeneousSolidSection(name='PlateSection', material='Material',
                                  thickness=PLATE_THICKNESS)
    model.HomogeneousSolidSection(name='RivetSection', material='Material',
                                  thickness=RIVET_THICKNESS)
    
    # Assign sections
    plate_region = regionToolset.Region(faces=plate.faces[:])
    plate.SectionAssignment(region=plate_region, sectionName='PlateSection')
    
    rivet_region = regionToolset.Region(faces=rivet.faces[:])
    rivet.SectionAssignment(region=rivet_region, sectionName='RivetSection')
    
    # Partition plate at midpoint x = POINT_E_X to ensure node at point E
    log('Partitioning plate at x = ' + str(POINT_E_X) + '...')
    dp = plate.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=POINT_E_X)
    plate.PartitionFaceByDatumPlane(datumPlane=plate.datums[dp.id],
                                    faces=plate.faces[:])
    
    # ----------------------------
    # Assembly
    # ----------------------------
    log('Creating assembly...')
    a = model.rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    
    plate_inst = a.Instance(name='Plate-1', part=plate, dependent=ON)
    rivet_left_inst = a.Instance(name='Rivet-Left', part=rivet, dependent=ON)
    rivet_right_inst = a.Instance(name='Rivet-Right', part=rivet, dependent=ON)
    
    # Position rivets at hole centers
    a.translate(instanceList=('Rivet-Left',),
                vector=(HOLE_LEFT_X, HOLE_LEFT_Y, 0.0))
    a.translate(instanceList=('Rivet-Right',),
                vector=(HOLE_RIGHT_X, HOLE_RIGHT_Y, 0.0))
    
    # ----------------------------
    # Steps
    # ----------------------------
    log('Creating analysis steps...')
    if ANALYSIS_TYPE == 'STATIC_LINEAR_PERTURBATION':
        # Base step: Static General (to close contact)
        model.StaticStep(name='BaseStep', previous='Initial',
                         nlgeom=OFF, timePeriod=1.0,
                         initialInc=0.1, minInc=1e-8, maxInc=1.0)
        # Perturbation step: load applied here
        model.StaticLinearPerturbationStep(name='LoadStep', previous='BaseStep')
    else:
        raise ValueError('Only STATIC_LINEAR_PERTURBATION supported.')
    
    # ----------------------------
    # Contact property
    # ----------------------------
    log('Creating contact property...')
    cp = model.ContactProperty('RivetPlateContact')
    cp.NormalBehavior(pressureOverclosure=HARD,
                      allowSeparation=ON,
                      constraintEnforcementMethod=DEFAULT)
    cp.TangentialBehavior(formulation=FRICTIONLESS)
    
    # ----------------------------
    # Contact surfaces and interactions (surface-to-surface pairs)
    # ----------------------------
    log('Creating surface-to-surface contact pairs...')
    
    for tag, inst, cx, cy in (('L', rivet_left_inst,  HOLE_LEFT_X,  HOLE_LEFT_Y),
                              ('R', rivet_right_inst, HOLE_RIGHT_X, HOLE_RIGHT_Y)):
        a.Surface(name='RivetOuter_' + tag,
                  side1Edges=ring_edges(inst, cx, cy, RIVET_OUTER_RADIUS))
        a.Surface(name='PlateHole_' + tag,
                  side1Edges=ring_edges(plate_inst, cx, cy, HOLE_RADIUS))
        model.SurfaceToSurfaceContactStd(
            name='Contact-' + tag, createStepName='Initial',
            master=a.surfaces['RivetOuter_' + tag],
            slave=a.surfaces['PlateHole_' + tag],
            sliding=SMALL,
            interactionProperty='RivetPlateContact')
    
    # ----------------------------
    # Boundary conditions: clamp central holes of rivets
    # ----------------------------
    log('Assigning boundary conditions...')
    
    # Select inner edges (diameter 8 mm) of both rivets
    rivet_left_inner = rivet_left_inst.edges.getByBoundingBox(
        xMin=HOLE_LEFT_X - RIVET_INNER_RADIUS - 0.1,
        xMax=HOLE_LEFT_X + RIVET_INNER_RADIUS + 0.1,
        yMin=HOLE_LEFT_Y - RIVET_INNER_RADIUS - 0.1,
        yMax=HOLE_LEFT_Y + RIVET_INNER_RADIUS + 0.1)
    
    rivet_right_inner = rivet_right_inst.edges.getByBoundingBox(
        xMin=HOLE_RIGHT_X - RIVET_INNER_RADIUS - 0.1,
        xMax=HOLE_RIGHT_X + RIVET_INNER_RADIUS + 0.1,
        yMin=HOLE_RIGHT_Y - RIVET_INNER_RADIUS - 0.1,
        yMax=HOLE_RIGHT_Y + RIVET_INNER_RADIUS + 0.1)
    
    a.Set(name='RivetLeftInner', edges=rivet_left_inner)
    a.Set(name='RivetRightInner', edges=rivet_right_inner)
    
    model.DisplacementBC(name='BC-RivetLeftInner', createStepName='Initial',
                         region=a.sets['RivetLeftInner'],
                         u1=0.0, u2=0.0, ur3=0.0)
    model.DisplacementBC(name='BC-RivetRightInner', createStepName='Initial',
                         region=a.sets['RivetRightInner'],
                         u1=0.0, u2=0.0, ur3=0.0)
    
    # ----------------------------
    # Load: distributed pressure on top edge (AB)
    # ----------------------------
    log('Applying distributed load...')
    
    # Find top edge of plate (y = PLATE_HEIGHT)
    top_edge = plate_inst.edges.getByBoundingBox(
        xMin=-0.1, xMax=PLATE_LENGTH + 0.1,
        yMin=PLATE_HEIGHT - 0.1, yMax=PLATE_HEIGHT + 0.1)
    
    if len(top_edge) == 0:
        raise RuntimeError('Could not find top edge of plate for loading.')
    
    # Create surface on top edge for pressure load
    a.Surface(name='PlateTopSurface', side1Edges=top_edge)
    
    # Preload in base step (to close contact and avoid rigid body motion)
    model.Pressure(name='PreLoad', createStepName='BaseStep',
                   region=a.surfaces['PlateTopSurface'], magnitude=1.0)
    
    # Apply distributed load (pressure) in LoadStep
    model.Pressure(name='TopPressure', createStepName='LoadStep',
                   region=a.surfaces['PlateTopSurface'], magnitude=PRESSURE)
    
    # ----------------------------
    # Mesh
    # ----------------------------
    log('Meshing...')
    plate.seedPart(size=MESH_SIZE, deviationFactor=0.1, minSizeFactor=0.1)
    rivet.seedPart(size=MESH_SIZE, deviationFactor=0.1, minSizeFactor=0.1)
    
    elemType = mesh.ElemType(elemCode=CPS4R, elemLibrary=STANDARD)
    plate.setElementType(regions=(plate.faces[:],), elemTypes=(elemType,))
    rivet.setElementType(regions=(rivet.faces[:],), elemTypes=(elemType,))
    
    plate.generateMesh()
    rivet.generateMesh()
    
    a.regenerate()
    
    # Count nodes
    total_nodes = 0
    for inst_name in ['Plate-1', 'Rivet-Left', 'Rivet-Right']:
        inst = a.instances[inst_name]
        n_nodes = len(inst.nodes)
        total_nodes += n_nodes
        log('Instance ' + inst_name + ': ' + str(n_nodes) + ' nodes')
    
    log('Total nodes: ' + str(total_nodes))
    if total_nodes > 1000:
        raise RuntimeError('Node count ' + str(total_nodes) +
                          ' exceeds Student Edition limit (1000).')
    
    # ----------------------------
    # Field output
    # ----------------------------
    model.fieldOutputRequests['F-Output-1'].setValues(
        variables=('S', 'U', 'RF'))
    
    return model, a

def run_job():
    log('Submitting job...')
    if JOB_NAME in mdb.jobs:
        del mdb.jobs[JOB_NAME]
    
    job = mdb.Job(name=JOB_NAME, model=MODEL_NAME,
                  description='Riveted plate with distributed load',
                  type=ANALYSIS,
                  memory=90, memoryUnits=PERCENTAGE,
                  multiprocessingMode=DEFAULT, numCpus=1, numDomains=1)
    
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()
    
    log('Job status: ' + str(job.status))
    if job.status != COMPLETED:
        raise RuntimeError('Job did not complete. Check .msg file.')

def extract_results():
    """Extract and visualize results from ODB."""
    odb_path = os.path.join(OUTPUT_DIRECTORY, JOB_NAME + '.odb')
    if not os.path.exists(odb_path):
        raise RuntimeError('ODB not found: ' + odb_path)
    
    log('Opening ODB: ' + odb_path)
    odb = visualization.openOdb(path=odb_path, readOnly=True)
    
    try:
        # Get last frame of LoadStep
        step_name = 'LoadStep'
        if step_name not in odb.steps:
            step_name = odb.steps.keys()[-1]
        
        step = odb.steps[step_name]
        frame = step.frames[-1]
        
        # Get plate instance
        inst = odb.rootAssembly.instances['PLATE-1']
        
        # Find stress and displacement at/near point E
        s_field = frame.fieldOutputs['S'].getSubset(region=inst,
                                                    position=ELEMENT_NODAL)
        u_field = frame.fieldOutputs['U'].getSubset(region=inst)
        
        # Build node coordinate map
        node_coords = {}
        for node in inst.nodes:
            node_coords[node.label] = (node.coordinates[0], node.coordinates[1])
        
        # Find closest node to point E
        min_dist = 1e10
        closest_node_label = None
        for label, (x, y) in node_coords.items():
            dist = (x - POINT_E_X)**2 + (y - POINT_E_Y)**2
            if dist < min_dist:
                min_dist = dist
                closest_node_label = label
        
        log('Closest node to E (%.1f, %.1f): label=%d, distance=%.3f mm' % 
            (POINT_E_X, POINT_E_Y, closest_node_label,
             min_dist**0.5))
        
        # Extract stress and displacement at closest node
        # For ELEMENT_NODAL position, average stress values across elements sharing the node
        s_vals = [v for v in s_field.values if v.nodeLabel == closest_node_label]
        if s_vals:
            n = float(len(s_vals))
            s11 = sum([v.data[0] for v in s_vals]) / n
            s22 = sum([v.data[1] for v in s_vals]) / n
            s33 = sum([v.data[2] for v in s_vals]) / n
            s12 = sum([v.data[3] for v in s_vals]) / n
            mises = sum([v.mises for v in s_vals]) / n
            stress_at_e = True
        else:
            stress_at_e = None
            s11 = s22 = s33 = s12 = mises = None
        
        disp_at_e = None
        for val in u_field.values:
            if val.nodeLabel == closest_node_label:
                disp_at_e = val
                break
        
        # Write summary
        summary_file = os.path.join(OUTPUT_DIRECTORY, 'summary.txt')
        with open(summary_file, 'w') as f:
            f.write('RIVETED PLATE ANALYSIS RESULTS\n')
            f.write('='*50 + '\n\n')
            
            f.write('GEOMETRY\n')
            f.write('Plate: %.1f x %.1f mm, thickness %.1f mm\n' %
                   (PLATE_LENGTH, PLATE_HEIGHT, PLATE_THICKNESS))
            f.write('Holes: radius %.1f mm at (%.1f, %.1f) and (%.1f, %.1f)\n' %
                   (HOLE_RADIUS, HOLE_LEFT_X, HOLE_LEFT_Y,
                    HOLE_RIGHT_X, HOLE_RIGHT_Y))
            f.write('Rivets: Ø%.1f mm with central hole Ø%.1f mm\n\n' %
                   (2*RIVET_OUTER_RADIUS, 2*RIVET_INNER_RADIUS))
            
            f.write('MATERIAL\n')
            f.write('E = %.1f MPa, nu = %.1f\n\n' %
                   (YOUNG_MODULUS, POISSON_RATIO))
            
            f.write('LOAD\n')
            f.write('Distributed pressure on top edge: %.1f N/mm^2\n' % PRESSURE)
            f.write('Total force: %.0f N\n\n' % TOTAL_FORCE)
            
            f.write('MESH\n')
            f.write('Element size: %.1f mm\n' % MESH_SIZE)
            f.write('Total nodes: < 1000 (Student Edition limit)\n\n')
            
            f.write('RESULTS AT POINT E (x=%.1f, y=%.1f)\n' %
                   (POINT_E_X, POINT_E_Y))
            f.write('Closest node distance: %.6f mm\n' % (min_dist**0.5))
            f.write('='*50 + '\n')
            
            if stress_at_e is not None:
                f.write('Stress components (MPa):\n')
                f.write('  S11 (normal, x-direction):  %.3f\n' % s11)
                f.write('  S22 (normal, y-direction):  %.3f\n' % s22)
                f.write('  S33 (normal, z-direction):  %.3f\n' % s33)
                f.write('  S12 (shear):                 %.3f\n' % s12)
                f.write('  Von Mises (equiv):           %.3f\n' % mises)
                f.write('\n')
            else:
                f.write('WARNING: Stress not found at node\n\n')
            
            if disp_at_e is not None:
                f.write('Displacement (mm):\n')
                f.write('  U1 (x-direction):  %.6f\n' % disp_at_e.data[0])
                f.write('  U2 (y-direction):  %.6f\n' % disp_at_e.data[1])
                f.write('\n')
            else:
                f.write('WARNING: Displacement not found at node\n\n')
            
            f.write('='*50 + '\n')
            f.write('Analysis type: Static Linear Perturbation\n')
            f.write('Contact: frictionless, surface-to-surface (rivet outer edge vs plate hole)\n')
        
        log('Summary written to ' + summary_file)
        
    finally:
        odb.close()

def main():
    ensure_dir(OUTPUT_DIRECTORY)
    log('Starting riveted plate analysis...')
    
    model, assembly = create_model()
    log('Model created successfully.')
    
    mdb.saveAs(pathName=os.path.join(OUTPUT_DIRECTORY, MODEL_NAME + '.cae'))
    log('CAE saved.')
    
    run_job()
    log('Job completed.')
    
    extract_results()
    log('ALL DONE.')

if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()
        raise
