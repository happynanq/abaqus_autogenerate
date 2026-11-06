# -*- coding: mbcs -*-
# Abaqus 2018 Student Edition
# Extract results from completed ODB (riveted plate analysis)

from abaqus import *
from abaqusConstants import *
import visualization
import os

# Paths
ODB_PATH = '/Users/kirill/Documents/code/2026/abaqus/result/RivetedPlate_Job.odb'
OUTPUT_DIRECTORY = '/Users/kirill/Documents/code/2026/abaqus/result'

# Point E coordinates
POINT_E_X = 75.0
POINT_E_Y = 25.0

# Geometry for summary
PLATE_LENGTH = 150.0
PLATE_HEIGHT = 50.0
PLATE_THICKNESS = 5.0
HOLE_RADIUS = 10.0
RIVET_OUTER_RADIUS = 10.0
RIVET_INNER_RADIUS = 4.0
RIVET_THICKNESS = 5.0
HOLE_LEFT_X = 30.0
HOLE_LEFT_Y = 25.0
HOLE_RIGHT_X = 120.0
HOLE_RIGHT_Y = 25.0

# Material
YOUNG_MODULUS = 200000.0
POISSON_RATIO = 0.3
PRESSURE = 1000.0
TOTAL_FORCE = PRESSURE * PLATE_LENGTH
MESH_SIZE = 4.0

def log(msg):
    print('[ExtractResults] ' + str(msg))

def extract_results():
    """Extract and visualize results from ODB."""
    if not os.path.exists(ODB_PATH):
        raise RuntimeError('ODB not found: ' + ODB_PATH)
    
    log('Opening ODB: ' + ODB_PATH)
    odb = visualization.openOdb(path=ODB_PATH, readOnly=True)
    
    try:
        # Get last frame of LoadStep
        step_name = 'LoadStep'
        if step_name not in odb.steps:
            step_name = odb.steps.keys()[-1]
            log('Using step: ' + step_name)
        
        step = odb.steps[step_name]
        frame = step.frames[-1]
        
        # Get plate instance
        inst = odb.rootAssembly.instances['PLATE-1']
        
        log('Total nodes in PLATE-1: ' + str(len(inst.nodes)))
        
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
            log('Found %d stress values at node %d' % (len(s_vals), closest_node_label))
        else:
            stress_at_e = None
            s11 = s22 = s33 = s12 = mises = None
            log('WARNING: No stress values found at node ' + str(closest_node_label))
        
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
            f.write('Total nodes in plate: %d\n\n' % len(inst.nodes))
            
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

if __name__ == '__main__':
    try:
        extract_results()
        log('DONE.')
    except:
        import traceback
        traceback.print_exc()
        raise
