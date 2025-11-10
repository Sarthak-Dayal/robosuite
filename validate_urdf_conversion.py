#!/usr/bin/env python3
"""
Validation script to verify that URDF values were correctly transferred to MJCF XML.

This script compares:
- Inertial properties (mass, position, inertia)
- Joint limits (lower, upper)
- Joint axes
- Actuator effort limits

Reports any discrepancies between URDF and generated MJCF.
"""

import xml.etree.ElementTree as ET
from pathlib import Path
import math
import sys

def rpy_to_quat(rpy):
    """Convert roll-pitch-yaw (in radians) to quaternion (w, x, y, z)."""
    roll, pitch, yaw = rpy
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return (w, x, y, z)

def parse_float_list(s, expected_len=None):
    """Parse a space-separated string of floats."""
    parts = s.strip().split()
    floats = [float(x) for x in parts]
    if expected_len and len(floats) != expected_len:
        raise ValueError(f"Expected {expected_len} values, got {len(floats)}")
    return floats

def quat_to_rpy(quat):
    """Convert quaternion (list or string) to RPY (approximate, for comparison)."""
    if isinstance(quat, str):
        q = parse_float_list(quat, 4)
    else:
        q = quat
    w, x, y, z = q
    
    # Convert quaternion to RPY
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(2*(w*y - z*x))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    
    return (roll, pitch, yaw)

def parse_urdf(urdf_path):
    """Parse URDF and extract all relevant data."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    ns = {'urdf': 'http://www.ros.org/wiki/urdf'} if root.tag.startswith('{') else {}
    
    links = {}
    joints = {}
    
    # Parse links
    for link in root.findall('.//link', ns) if ns else root.findall('.//link'):
        link_name = link.get('name')
        links[link_name] = {}
        
        inertial = link.find('inertial', ns) if ns else link.find('inertial')
        if inertial is not None:
            origin = inertial.find('origin', ns) if ns else inertial.find('origin')
            mass_elem = inertial.find('mass', ns) if ns else inertial.find('mass')
            inertia_elem = inertial.find('inertia', ns) if ns else inertial.find('inertia')
            
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                links[link_name]['inertial_pos'] = parse_float_list(xyz, 3)
                links[link_name]['inertial_rpy'] = parse_float_list(rpy, 3)
            
            if mass_elem is not None:
                links[link_name]['mass'] = float(mass_elem.get('value', '0'))
            
            if inertia_elem is not None:
                links[link_name]['inertia'] = {
                    'ixx': float(inertia_elem.get('ixx', '0')),
                    'ixy': float(inertia_elem.get('ixy', '0')),
                    'ixz': float(inertia_elem.get('ixz', '0')),
                    'iyy': float(inertia_elem.get('iyy', '0')),
                    'iyz': float(inertia_elem.get('iyz', '0')),
                    'izz': float(inertia_elem.get('izz', '0')),
                }
    
    # Parse joints
    for joint in root.findall('.//joint', ns) if ns else root.findall('.//joint'):
        joint_name = joint.get('name')
        joints[joint_name] = {}
        
        axis = joint.find('axis', ns) if ns else joint.find('axis')
        if axis is not None:
            xyz = axis.get('xyz', '1 0 0')
            joints[joint_name]['axis'] = parse_float_list(xyz, 3)
        
        limit = joint.find('limit', ns) if ns else joint.find('limit')
        if limit is not None:
            joints[joint_name]['lower'] = float(limit.get('lower', '0'))
            joints[joint_name]['upper'] = float(limit.get('upper', '0'))
            joints[joint_name]['effort'] = float(limit.get('effort', '0'))
    
    return links, joints

def parse_mjcf(mjcf_path):
    """Parse MJCF and extract relevant data."""
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    
    bodies = {}
    joints = {}
    actuators = {}
    
    worldbody = root.find('worldbody')
    if worldbody is not None:
        # Parse bodies and their inertial properties
        for body in worldbody.iter('body'):
            body_name = body.get('name')
            if body_name:
                bodies[body_name] = {}
                inertial = body.find('inertial')
                if inertial is not None:
                    if 'pos' in inertial.attrib:
                        bodies[body_name]['inertial_pos'] = parse_float_list(inertial.get('pos'), 3)
                    if 'quat' in inertial.attrib:
                        bodies[body_name]['inertial_quat'] = parse_float_list(inertial.get('quat'), 4)
                    if 'mass' in inertial.attrib:
                        bodies[body_name]['mass'] = float(inertial.get('mass'))
                    if 'diaginertia' in inertial.attrib:
                        bodies[body_name]['diaginertia'] = parse_float_list(inertial.get('diaginertia'), 3)
        
        # Parse joints
        for joint in worldbody.iter('joint'):
            joint_name = joint.get('name')
            if joint_name:
                joints[joint_name] = {}
                if 'axis' in joint.attrib:
                    joints[joint_name]['axis'] = parse_float_list(joint.get('axis'), 3)
                if 'range' in joint.attrib:
                    range_vals = parse_float_list(joint.get('range'), 2)
                    joints[joint_name]['lower'] = range_vals[0]
                    joints[joint_name]['upper'] = range_vals[1]
    
    # Parse actuators
    actuator = root.find('actuator')
    if actuator is not None:
        for motor in actuator.findall('motor'):
            joint_name = motor.get('joint')
            if joint_name:
                if 'ctrlrange' in motor.attrib:
                    ctrlrange = parse_float_list(motor.get('ctrlrange'), 2)
                    # Effort is the maximum absolute value
                    actuators[joint_name] = max(abs(ctrlrange[0]), abs(ctrlrange[1]))
    
    return bodies, joints, actuators

def map_urdf_to_mjcf_names():
    """Get the name mapping used in build_gr1t1.py."""
    joint_mapping = {
        'waist_yaw': 'torso_waist_yaw',
        'waist_pitch': 'torso_waist_pitch',
        'waist_roll': 'torso_waist_roll',
        'l_hip_roll': 'l_leg_hip_roll',
        'l_hip_yaw': 'l_leg_hip_yaw',
        'l_hip_pitch': 'l_leg_hip_pitch',
        'l_knee_pitch': 'l_leg_knee_pitch',
        'l_ankle_pitch': 'l_leg_ankle_pitch',
        'l_ankle_roll': 'l_leg_ankle_roll',
        'r_hip_roll': 'r_leg_hip_roll',
        'r_hip_yaw': 'r_leg_hip_yaw',
        'r_hip_pitch': 'r_leg_hip_pitch',
        'r_knee_pitch': 'r_leg_knee_pitch',
        'r_ankle_pitch': 'r_leg_ankle_pitch',
        'r_ankle_roll': 'r_leg_ankle_roll',
    }
    
    link_mapping = {
        'base': 'base',
        'torso_waist_yaw': 'waist_yaw',
        'torso_waist_pitch': 'waist_pitch',
        'torso_waist_roll': 'waist_roll',
    }
    
    return joint_mapping, link_mapping

def compare_values(urdf_val, mjcf_val, name, tolerance=1e-6):
    """Compare two values and return if they match."""
    if urdf_val is None or mjcf_val is None:
        return False, f"Missing value: URDF={urdf_val}, MJCF={mjcf_val}"
    
    if isinstance(urdf_val, (list, tuple)) and isinstance(mjcf_val, (list, tuple)):
        if len(urdf_val) != len(mjcf_val):
            return False, f"Length mismatch: URDF={len(urdf_val)}, MJCF={len(mjcf_val)}"
        for i, (u, m) in enumerate(zip(urdf_val, mjcf_val)):
            if abs(u - m) > tolerance:
                return False, f"Element {i} mismatch: URDF={u}, MJCF={m}, diff={abs(u-m)}"
        return True, "Match"
    else:
        diff = abs(urdf_val - mjcf_val)
        if diff > tolerance:
            return False, f"Value mismatch: URDF={urdf_val}, MJCF={mjcf_val}, diff={diff}"
        return True, "Match"

def validate_conversion(urdf_path, mjcf_path, tolerance=1e-4):
    """Validate that URDF values were correctly transferred to MJCF."""
    print(f"Validating conversion...")
    print(f"  URDF: {urdf_path}")
    print(f"  MJCF: {mjcf_path}")
    print(f"  Tolerance: {tolerance}\n")
    
    # Parse files
    urdf_links, urdf_joints = parse_urdf(urdf_path)
    mjcf_bodies, mjcf_joints, mjcf_actuators = parse_mjcf(mjcf_path)
    
    joint_mapping, link_mapping = map_urdf_to_mjcf_names()
    
    errors = []
    warnings = []
    successes = []
    
    # Validate links/bodies
    print("=" * 60)
    print("VALIDATING BODIES (Inertial Properties)")
    print("=" * 60)
    
    for mjcf_body_name, urdf_link_name in link_mapping.items():
        if mjcf_body_name not in mjcf_bodies:
            warnings.append(f"Body '{mjcf_body_name}' not found in MJCF")
            continue
        if urdf_link_name not in urdf_links:
            warnings.append(f"Link '{urdf_link_name}' not found in URDF")
            continue
        
        urdf_data = urdf_links[urdf_link_name]
        mjcf_data = mjcf_bodies[mjcf_body_name]
        
        print(f"\nBody: {mjcf_body_name} (URDF: {urdf_link_name})")
        
        # Check mass
        if 'mass' in urdf_data and 'mass' in mjcf_data:
            match, msg = compare_values(urdf_data['mass'], mjcf_data['mass'], 'mass', tolerance)
            if match:
                successes.append(f"{mjcf_body_name}.mass: {msg}")
                print(f"  ✓ Mass: {urdf_data['mass']} == {mjcf_data['mass']}")
            else:
                errors.append(f"{mjcf_body_name}.mass: {msg}")
                print(f"  ✗ Mass: {msg}")
        
        # Check position
        if 'inertial_pos' in urdf_data and 'inertial_pos' in mjcf_data:
            match, msg = compare_values(urdf_data['inertial_pos'], mjcf_data['inertial_pos'], 'pos', tolerance)
            if match:
                successes.append(f"{mjcf_body_name}.pos: {msg}")
                print(f"  ✓ Position: {urdf_data['inertial_pos']} == {mjcf_data['inertial_pos']}")
            else:
                errors.append(f"{mjcf_body_name}.pos: {msg}")
                print(f"  ✗ Position: {msg}")
        
        # Check orientation (convert quat to RPY for comparison)
        if 'inertial_rpy' in urdf_data and 'inertial_quat' in mjcf_data:
            urdf_rpy = urdf_data['inertial_rpy']
            mjcf_quat = mjcf_data['inertial_quat']
            mjcf_rpy = quat_to_rpy(mjcf_quat)
            # Compare RPY (with larger tolerance for quaternion conversion)
            match, msg = compare_values(urdf_rpy, mjcf_rpy, 'orientation', tolerance * 10)
            if match:
                successes.append(f"{mjcf_body_name}.orientation: {msg}")
                print(f"  ✓ Orientation (RPY): {urdf_rpy} ≈ {mjcf_rpy}")
            else:
                warnings.append(f"{mjcf_body_name}.orientation: {msg} (quaternion conversion may introduce small errors)")
                print(f"  ⚠ Orientation: {msg}")
        
        # Check inertia (diagonal only for now)
        if 'inertia' in urdf_data and 'diaginertia' in mjcf_data:
            urdf_diag = [urdf_data['inertia']['ixx'], urdf_data['inertia']['iyy'], urdf_data['inertia']['izz']]
            mjcf_diag = mjcf_data['diaginertia']
            match, msg = compare_values(urdf_diag, mjcf_diag, 'inertia', tolerance)
            if match:
                successes.append(f"{mjcf_body_name}.inertia: {msg}")
                print(f"  ✓ Inertia (diagonal): {urdf_diag} == {mjcf_diag}")
            else:
                errors.append(f"{mjcf_body_name}.inertia: {msg}")
                print(f"  ✗ Inertia: {msg}")
    
    # Validate joints
    print("\n" + "=" * 60)
    print("VALIDATING JOINTS (Limits and Axes)")
    print("=" * 60)
    
    for urdf_joint_name, mjcf_joint_name in joint_mapping.items():
        if mjcf_joint_name not in mjcf_joints:
            warnings.append(f"Joint '{mjcf_joint_name}' not found in MJCF")
            continue
        if urdf_joint_name not in urdf_joints:
            warnings.append(f"Joint '{urdf_joint_name}' not found in URDF")
            continue
        
        urdf_data = urdf_joints[urdf_joint_name]
        mjcf_data = mjcf_joints[mjcf_joint_name]
        
        print(f"\nJoint: {mjcf_joint_name} (URDF: {urdf_joint_name})")
        
        # Check axis
        if 'axis' in urdf_data and 'axis' in mjcf_data:
            match, msg = compare_values(urdf_data['axis'], mjcf_data['axis'], 'axis', tolerance)
            if match:
                successes.append(f"{mjcf_joint_name}.axis: {msg}")
                print(f"  ✓ Axis: {urdf_data['axis']} == {mjcf_data['axis']}")
            else:
                errors.append(f"{mjcf_joint_name}.axis: {msg}")
                print(f"  ✗ Axis: {msg}")
        
        # Check limits
        if 'lower' in urdf_data and 'upper' in urdf_data:
            if 'lower' in mjcf_data and 'upper' in mjcf_data:
                lower_match, lower_msg = compare_values(urdf_data['lower'], mjcf_data['lower'], 'lower', tolerance)
                upper_match, upper_msg = compare_values(urdf_data['upper'], mjcf_data['upper'], 'upper', tolerance)
                if lower_match and upper_match:
                    successes.append(f"{mjcf_joint_name}.range: Match")
                    print(f"  ✓ Range: [{urdf_data['lower']}, {urdf_data['upper']}] == [{mjcf_data['lower']}, {mjcf_data['upper']}]")
                else:
                    if not lower_match:
                        errors.append(f"{mjcf_joint_name}.lower: {lower_msg}")
                        print(f"  ✗ Lower limit: {lower_msg}")
                    if not upper_match:
                        errors.append(f"{mjcf_joint_name}.upper: {upper_msg}")
                        print(f"  ✗ Upper limit: {upper_msg}")
            else:
                errors.append(f"{mjcf_joint_name}: Missing range in MJCF")
                print(f"  ✗ Range: Missing in MJCF")
    
    # Validate actuators
    print("\n" + "=" * 60)
    print("VALIDATING ACTUATORS (Effort Limits)")
    print("=" * 60)
    
    for urdf_joint_name, mjcf_joint_name in joint_mapping.items():
        if mjcf_joint_name not in mjcf_actuators:
            continue
        if urdf_joint_name not in urdf_joints or 'effort' not in urdf_joints[urdf_joint_name]:
            continue
        
        urdf_effort = urdf_joints[urdf_joint_name]['effort']
        mjcf_effort = mjcf_actuators[mjcf_joint_name]
        
        match, msg = compare_values(urdf_effort, mjcf_effort, 'effort', tolerance)
        if match:
            successes.append(f"{mjcf_joint_name}.effort: {msg}")
            print(f"  ✓ Effort: {urdf_effort} == {mjcf_effort}")
        else:
            errors.append(f"{mjcf_joint_name}.effort: {msg}")
            print(f"  ✗ Effort: {msg}")
    
    # Summary
    print("\n" + "=" * 60)
    print("VALIDATION SUMMARY")
    print("=" * 60)
    print(f"✓ Successful matches: {len(successes)}")
    print(f"✗ Errors: {len(errors)}")
    print(f"⚠ Warnings: {len(warnings)}")
    
    if errors:
        print("\nERRORS:")
        for error in errors:
            print(f"  - {error}")
    
    if warnings:
        print("\nWARNINGS:")
        for warning in warnings:
            print(f"  - {warning}")
    
    return len(errors) == 0

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Validate URDF to MJCF conversion')
    parser.add_argument('--urdf', type=str, default='GR1T1.urdf', help='Path to URDF file')
    parser.add_argument('--mjcf', type=str, default='robosuite/models/assets/robots/gr1/GR1T1.xml', help='Path to MJCF file')
    parser.add_argument('--tolerance', type=float, default=1e-4, help='Tolerance for value comparisons')
    
    args = parser.parse_args()
    
    urdf_path = Path(args.urdf)
    mjcf_path = Path(args.mjcf)
    
    if not urdf_path.exists():
        print(f"Error: URDF file {urdf_path} not found!")
        sys.exit(1)
    
    if not mjcf_path.exists():
        print(f"Error: MJCF file {mjcf_path} not found!")
        sys.exit(1)
    
    success = validate_conversion(urdf_path, mjcf_path, args.tolerance)
    sys.exit(0 if success else 1)

