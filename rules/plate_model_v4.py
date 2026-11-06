# -*- coding: mbcs -*-
# Abaqus 2018 Student Edition
# Plate on two rounded supports: model + job + postprocessing in one file.
#
# Run from Abaqus Command:
#   abaqus cae noGUI=plate_model.py
#
# IMPORTANT:
# 1) This is a first-test template. Verify geometry/contact/BCs against your course model.
# 2) Units: N-mm-MPa. E=2e8 Pa = 200 MPa.
# 3) A 2D planar model uses section thickness as the out-of-plane thickness.
# 4) For a point load in 2D, a geometric vertex is created at the plate midpoint.
# 5) Student Edition model-size limits apply.

from abaqus import *
from abaqusConstants import *
import regionToolset
import mesh
import visualization
import os
import csv
import traceback

# ============================================================
# USER CONFIGURATION
# ============================================================

MODEL_NAME = 'PlateOnSupports'
JOB_NAME = 'PlateOnSupports_Job'

# Geometry, mm
PLATE_LENGTH = 1000.0
PLATE_HEIGHT = 20.0
PLATE_THICKNESS = 20.0       # out-of-plane section thickness, mm

SUPPORT_WIDTH = 100.0
SUPPORT_HEIGHT = 100.0
SUPPORT_RADIUS = 20.0

# Supports are placed 0 mm from each plate end in this interpretation.
# Change to 100.0 if your drawing/assignment means 100 mm inset from each end.
SUPPORT_LEFT_X = 0.0
SUPPORT_RIGHT_X = PLATE_LENGTH - SUPPORT_WIDTH

# Material, N-mm-MPa
YOUNG_MODULUS = 200.0
POISSON_RATIO = 0.3

# Load
FORCE_Y = -100.0              # N, negative = downward
LOAD_X = PLATE_LENGTH / 2.0

# Analysis:
# 'STATIC_GENERAL' is recommended for first contact test.
# 'STATIC_LINEAR_PERTURBATION' requires a valid base state; see notes below.
ANALYSIS_TYPE = 'STATIC_GENERAL'

# Contact
NORMAL_BEHAVIOR = 'HARD'
TANGENTIAL_BEHAVIOR = 'ROUGH'

# Mesh
MESH_SIZE = 10.0
ELEMENT_CODE = 'CPS4R'

# Output graph choices:
# Stress options: 'S11', 'S22', 'S12', 'MISES'
STRESS_COMPONENT = 'S11'

# Which plate edge to sample:
STRESS_EDGE = 'BOTTOM'         # 'BOTTOM' or 'TOP'
DISPLACEMENT_EDGE = 'TOP'      # 'BOTTOM' or 'TOP'

# Output
OUTPUT_DIRECTORY = os.getcwd()
EXPORT_PNG = True
EXPORT_CSV = True
SHOW_GUI_PLOTS = False

# ============================================================
# HELPERS
# ============================================================

def log(msg):
    print('[plate_model] ' + str(msg))

def ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)

def get_edge_by_y(part, y_value, tol=1e-6):
    edges = []
    for e in part.edges:
        p = e.pointOn[0]
        if abs(p[1] - y_value) < tol:
            edges.append(e)
    return edges

def find_vertex_at(part, x, y, tol=1e-6):
    for v in part.vertices:
        p = v.pointOn
        if abs(p[0] - x) < tol and abs(p[1] - y) < tol:
            return v
    return None

def make_rounded_support_sketch(sketch, w, h, r):
    # Rounded rectangle: all FOUR corners have radius r.
    # Coordinate system: support bottom at y=0, top at y=h.
    # Draw straight segments between tangent points and four quarter-circle arcs.
    # Counterclockwise arcs follow the boundary in sequence.
    sketch.Line(point1=(r, 0.0), point2=(w-r, 0.0))
    sketch.ArcByCenterEnds(center=(w-r, r), point1=(w-r, 0.0),
                           point2=(w, r), direction=COUNTERCLOCKWISE)
    sketch.Line(point1=(w, r), point2=(w, h-r))
    sketch.ArcByCenterEnds(center=(w-r, h-r), point1=(w, h-r),
                           point2=(w-r, h), direction=COUNTERCLOCKWISE)
    sketch.Line(point1=(w-r, h), point2=(r, h))
    sketch.ArcByCenterEnds(center=(r, h-r), point1=(r, h),
                           point2=(0.0, h-r), direction=COUNTERCLOCKWISE)
    sketch.Line(point1=(0.0, h-r), point2=(0.0, r))
    sketch.ArcByCenterEnds(center=(r, r), point1=(0.0, r),
                           point2=(r, 0.0), direction=COUNTERCLOCKWISE)

def create_model():
    if MODEL_NAME in mdb.models:
        del mdb.models[MODEL_NAME]
    model = mdb.Model(name=MODEL_NAME)

    # ----------------------------
    # Plate: partition at midpoint to create a load vertex.
    # ----------------------------
    sk = model.ConstrainedSketch(name='PlateSketch', sheetSize=PLATE_LENGTH*1.2)
    sk.Line(point1=(0.0, 0.0), point2=(PLATE_LENGTH, 0.0))
    sk.Line(point1=(PLATE_LENGTH, 0.0), point2=(PLATE_LENGTH, PLATE_HEIGHT))
    sk.Line(point1=(PLATE_LENGTH, PLATE_HEIGHT), point2=(LOAD_X, PLATE_HEIGHT))
    sk.Line(point1=(LOAD_X, PLATE_HEIGHT), point2=(0.0, PLATE_HEIGHT))
    sk.Line(point1=(0.0, PLATE_HEIGHT), point2=(0.0, 0.0))

    plate = model.Part(name='Plate', dimensionality=TWO_D_PLANAR,
                       type=DEFORMABLE_BODY)
    plate.BaseShell(sketch=sk)
    del model.sketches['PlateSketch']

    # ----------------------------
    # Supports: same part definition, two instances.
    # ----------------------------
    sks = model.ConstrainedSketch(name='SupportSketch', sheetSize=SUPPORT_WIDTH*1.5)
    make_rounded_support_sketch(sks, SUPPORT_WIDTH, SUPPORT_HEIGHT, SUPPORT_RADIUS)

    support = model.Part(name='Support', dimensionality=TWO_D_PLANAR,
                         type=DEFORMABLE_BODY)
    support.BaseShell(sketch=sks)
    del model.sketches['SupportSketch']

    # ----------------------------
    # Material and sections
    # ----------------------------
    mat = model.Material(name='Material')
    mat.Elastic(table=((YOUNG_MODULUS, POISSON_RATIO),))

    model.HomogeneousSolidSection(name='PlateSection', material='Material',
                                  thickness=PLATE_THICKNESS)
    model.HomogeneousSolidSection(name='SupportSection', material='Material',
                                  thickness=PLATE_THICKNESS)

    plate_region = regionToolset.Region(faces=plate.faces[:])
    plate.SectionAssignment(region=plate_region, sectionName='PlateSection')

    support_region = regionToolset.Region(faces=support.faces[:])
    support.SectionAssignment(region=support_region, sectionName='SupportSection')

    # ----------------------------
    # Assembly
    # ----------------------------
    a = model.rootAssembly
    a.DatumCsysByDefault(CARTESIAN)

    p_inst = a.Instance(name='Plate-1', part=plate, dependent=ON)
    left_inst = a.Instance(name='Support-Left', part=support, dependent=ON)
    right_inst = a.Instance(name='Support-Right', part=support, dependent=ON)

    # Plate from x=0 to L, y=0 to H.
    # Support tops are at y=0; support bodies extend downward.
    a.translate(instanceList=('Support-Left',),
                vector=(SUPPORT_LEFT_X, -SUPPORT_HEIGHT, 0.0))
    a.translate(instanceList=('Support-Right',),
                vector=(SUPPORT_RIGHT_X, -SUPPORT_HEIGHT, 0.0))

    # ----------------------------
    # Step
    # ----------------------------
    if ANALYSIS_TYPE == 'STATIC_GENERAL':
        model.StaticStep(name='LoadStep', previous='Initial',
                         nlgeom=OFF, timePeriod=1.0,
                         initialInc=0.1, minInc=1e-8, maxInc=1.0)
    elif ANALYSIS_TYPE == 'STATIC_LINEAR_PERTURBATION':
        # Abaqus linear perturbation steps require a base state.
        # This script creates a small preload step, followed by perturbation.
        model.StaticStep(name='BaseStep', previous='Initial',
                         nlgeom=OFF, timePeriod=1.0,
                         initialInc=0.1, minInc=1e-8, maxInc=1.0)
        model.StaticLinearPerturbationStep(name='LoadStep', previous='BaseStep')
    else:
        raise ValueError('Unknown ANALYSIS_TYPE: ' + ANALYSIS_TYPE)

    # ----------------------------
    # Contact property
    # ----------------------------
    cp = model.ContactProperty('PlateSupportContact')
    cp.NormalBehavior(pressureOverclosure=HARD,
                      allowSeparation=ON,
                      constraintEnforcementMethod=DEFAULT)
    cp.TangentialBehavior(formulation=ROUGH)

    # Surface definitions: plate bottom edge and support top edges.
    plate_bottom_edges = p_inst.edges.getByBoundingBox(
        xMin=-1e-3, xMax=PLATE_LENGTH+1e-3,
        yMin=-1e-3, yMax=1e-3)
    left_top_edges = left_inst.edges.getByBoundingBox(
        xMin=SUPPORT_LEFT_X-1e-3, xMax=SUPPORT_LEFT_X+SUPPORT_WIDTH+1e-3,
        yMin=-1e-3, yMax=1e-3)
    right_top_edges = right_inst.edges.getByBoundingBox(
        xMin=SUPPORT_RIGHT_X-1e-3, xMax=SUPPORT_RIGHT_X+SUPPORT_WIDTH+1e-3,
        yMin=-1e-3, yMax=1e-3)

    a.Surface(name='PlateBottom', side1Edges=plate_bottom_edges)
    a.Surface(name='LeftSupportTop', side1Edges=left_top_edges)
    a.Surface(name='RightSupportTop', side1Edges=right_top_edges)

    # General contact is more robust for this first template.
    model.ContactStd(name='GeneralContact', createStepName='Initial')
    model.interactions['GeneralContact'].includedPairs.setValuesInStep(
        stepName='Initial', useAllstar=ON)
    model.interactions['GeneralContact'].contactPropertyAssignments.appendInStep(
        stepName='Initial',
        assignments=((GLOBAL, SELF, 'PlateSupportContact'),))

    # ----------------------------
    # Boundary conditions: bottom support edges fixed.
    # ----------------------------
    left_bottom_edges = left_inst.edges.getByBoundingBox(
        xMin=SUPPORT_LEFT_X-1e-3,
        xMax=SUPPORT_LEFT_X+SUPPORT_WIDTH+1e-3,
        yMin=-SUPPORT_HEIGHT-1e-3,
        yMax=-SUPPORT_HEIGHT+1e-3)
    right_bottom_edges = right_inst.edges.getByBoundingBox(
        xMin=SUPPORT_RIGHT_X-1e-3,
        xMax=SUPPORT_RIGHT_X+SUPPORT_WIDTH+1e-3,
        yMin=-SUPPORT_HEIGHT-1e-3,
        yMax=-SUPPORT_HEIGHT+1e-3)

    a.Set(name='LeftSupportBottom', edges=left_bottom_edges)
    a.Set(name='RightSupportBottom', edges=right_bottom_edges)

    model.DisplacementBC(name='BC-LeftSupport', createStepName='Initial',
                         region=a.sets['LeftSupportBottom'],
                         u1=0.0, u2=0.0, ur3=0.0)
    model.DisplacementBC(name='BC-RightSupport', createStepName='Initial',
                         region=a.sets['RightSupportBottom'],
                         u1=0.0, u2=0.0, ur3=0.0)

    # ----------------------------
    # Concentrated force at plate midpoint TOP vertex.
    # ----------------------------
    load_vertex = p_inst.vertices.getByBoundingBox(
        xMin=LOAD_X-1e-3, xMax=LOAD_X+1e-3,
        yMin=PLATE_HEIGHT-1e-3, yMax=PLATE_HEIGHT+1e-3)
    if len(load_vertex) == 0:
        raise RuntimeError('Could not find plate midpoint vertex for concentrated load.')
    load_region = regionToolset.Region(vertices=load_vertex)
    model.ConcentratedForce(name='CenterForce', createStepName='LoadStep',
                            region=load_region, cf2=FORCE_Y)

    # ----------------------------
    # Mesh
    # ----------------------------
    plate.seedPart(size=MESH_SIZE, deviationFactor=0.1, minSizeFactor=0.1)
    support.seedPart(size=MESH_SIZE, deviationFactor=0.1, minSizeFactor=0.1)

    elemType = mesh.ElemType(elemCode=CPS4R, elemLibrary=STANDARD)
    plate.setElementType(regions=(plate.faces[:],), elemTypes=(elemType,))
    support.setElementType(regions=(support.faces[:],), elemTypes=(elemType,))

    plate.generateMesh()
    support.generateMesh()

    # Rebuild assembly mesh instances if dependent instances.
    a.regenerate()

    # ----------------------------
    # Field output
    # ----------------------------
    # Keep the field-output request to variables valid for the structural step.
    # Contact-specific outputs (CSTATUS, CPRESS, CSHEAR) can be invalid in
    # some Abaqus 2018 step/output contexts; they are not needed for the two
    # requested graphs. Abaqus can still solve the contact interaction.
    model.fieldOutputRequests['F-Output-1'].setValues(
        variables=('S', 'U', 'RF'))

    return model

def run_job():
    if JOB_NAME in mdb.jobs:
        del mdb.jobs[JOB_NAME]

    job = mdb.Job(name=JOB_NAME, model=MODEL_NAME,
                  description='2D plate on two rounded supports',
                  type=ANALYSIS,
                  memory=90, memoryUnits=PERCENTAGE,
                  multiprocessingMode=DEFAULT, numCpus=1, numDomains=1)

    log('Submitting job: ' + JOB_NAME)
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()
    log('Job status: ' + str(job.status))

    if job.status != COMPLETED:
        raise RuntimeError('Job did not complete successfully. Check .msg/.dat/.sta files.')

def extract_xy_from_odb():
    odb_path = os.path.join(OUTPUT_DIRECTORY, JOB_NAME + '.odb')
    if not os.path.exists(odb_path):
        raise RuntimeError('ODB not found: ' + odb_path)

    odb = visualization.openOdb(path=odb_path, readOnly=True)
    try:
        step_name = 'LoadStep'
        if step_name not in odb.steps:
            step_name = odb.steps.keys()[-1]
        step = odb.steps[step_name]
        frame = step.frames[-1]

        # Select plate instance.
        inst = odb.rootAssembly.instances['PLATE-1']

        # Determine requested stress component.
        s_field = frame.fieldOutputs['S'].getSubset(region=inst)
        u_field = frame.fieldOutputs['U'].getSubset(region=inst)

        stress_rows = []
        disp_rows = []

        # Build node coordinate lookup.
        node_xy = {}
        for node in inst.nodes:
            node_xy[node.label] = (node.coordinates[0], node.coordinates[1])

        # For stress: collect element-associated values, then reduce by x.
        # Values are element integration-point data; for graph visualization,
        # use nodal-averaged output where available.
        if STRESS_COMPONENT == 'S11':
            comp_index = 0
        elif STRESS_COMPONENT == 'S22':
            comp_index = 1
        elif STRESS_COMPONENT == 'S12':
            comp_index = 3
        elif STRESS_COMPONENT == 'MISES':
            comp_index = None
        else:
            raise ValueError('Unsupported STRESS_COMPONENT: ' + STRESS_COMPONENT)

        # Get nodal-averaged stress if Abaqus provides it.
        try:
            s_nodal = s_field.getSubset(position=NODAL)
        except:
            s_nodal = s_field

        for val in s_nodal.values:
            if val.nodeLabel is None:
                continue
            xy = node_xy.get(val.nodeLabel, None)
            if xy is None:
                continue
            x, y = xy
            if STRESS_EDGE == 'BOTTOM' and abs(y - 0.0) > (MESH_SIZE * 0.6):
                continue
            if STRESS_EDGE == 'TOP' and abs(y - PLATE_HEIGHT) > (MESH_SIZE * 0.6):
                continue
            if STRESS_COMPONENT == 'MISES':
                value = val.mises
            else:
                value = val.data[comp_index]
            stress_rows.append((x, value))

        for val in u_field.values:
            if val.nodeLabel is None:
                continue
            xy = node_xy.get(val.nodeLabel, None)
            if xy is None:
                continue
            x, y = xy
            if DISPLACEMENT_EDGE == 'BOTTOM' and abs(y - 0.0) > (MESH_SIZE * 0.6):
                continue
            if DISPLACEMENT_EDGE == 'TOP' and abs(y - PLATE_HEIGHT) > (MESH_SIZE * 0.6):
                continue
            disp_rows.append((x, val.data[1]))

        stress_rows.sort(key=lambda row: row[0])
        disp_rows.sort(key=lambda row: row[0])

        # Export raw CSV.
        if EXPORT_CSV:
            stress_csv = os.path.join(OUTPUT_DIRECTORY, 'stress_' + STRESS_COMPONENT + '.csv')
            with open(stress_csv, 'wb') as f:
                writer = csv.writer(f)
                writer.writerow(['x_mm', 'stress_MPa'])
                writer.writerows(stress_rows)

            disp_csv = os.path.join(OUTPUT_DIRECTORY, 'displacement_U2.csv')
            with open(disp_csv, 'wb') as f:
                writer = csv.writer(f)
                writer.writerow(['x_mm', 'U2_mm'])
                writer.writerows(disp_rows)

            log('CSV exported.')

        # Plot and save PNG using matplotlib if available in Abaqus Python.
        if EXPORT_PNG:
            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt

                if len(stress_rows) > 0:
                    xs = [r[0] for r in stress_rows]
                    ys = [r[1] for r in stress_rows]
                    plt.figure()
                    plt.plot(xs, ys, '-')
                    plt.xlabel('x, mm')
                    plt.ylabel('Stress ' + STRESS_COMPONENT + ', MPa')
                    plt.title('Stress along plate ' + STRESS_EDGE.lower() + ' edge')
                    plt.grid(True)
                    plt.tight_layout()
                    plt.savefig(os.path.join(OUTPUT_DIRECTORY,
                                             'stress_' + STRESS_COMPONENT + '.png'),
                                dpi=160)
                    plt.close()

                if len(disp_rows) > 0:
                    xs = [r[0] for r in disp_rows]
                    ys = [r[1] for r in disp_rows]
                    plt.figure()
                    plt.plot(xs, ys, '-')
                    plt.xlabel('x, mm')
                    plt.ylabel('U2, mm')
                    plt.title('Vertical displacement along plate ' +
                              DISPLACEMENT_EDGE.lower() + ' edge')
                    plt.grid(True)
                    plt.tight_layout()
                    plt.savefig(os.path.join(OUTPUT_DIRECTORY,
                                             'displacement_U2.png'), dpi=160)
                    plt.close()

                log('PNG plots exported.')
            except Exception as plot_error:
                log('Plotting failed (CSV data should still be available): ' + str(plot_error))

    finally:
        odb.close()

def main():
    ensure_dir(OUTPUT_DIRECTORY)
    log('Creating model...')
    create_model()
    log('Saving CAE...')
    mdb.saveAs(pathName=os.path.join(OUTPUT_DIRECTORY, MODEL_NAME + '.cae'))
    run_job()
    extract_xy_from_odb()
    log('ALL DONE.')

if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc()
        raise
