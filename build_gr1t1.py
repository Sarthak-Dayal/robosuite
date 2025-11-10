#!/usr/bin/env python3
"""
Script to build GR1T1.xml from URDF and robot.xml template.
- Uses robot.xml as structure template (hierarchy, naming, etc.)
- Extracts values (inertial, joint limits, etc.) from URDF
- Maps URDF links/joints to robot.xml bodies/joints
- Keeps meshes from robot.xml
"""

import xml.etree.ElementTree as ET
from pathlib import Path
import math

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
    
    return f"{w} {x} {y} {z}"

def xyz_to_pos(xyz_str):
    """Convert URDF xyz string to MJCF pos string."""
    parts = xyz_str.strip().split()
    return f"{parts[0]} {parts[1]} {parts[2]}"

def parse_urdf(urdf_path):
    """Parse URDF file and extract link and joint data."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Extract namespace if present
    ns = {'urdf': 'http://www.ros.org/wiki/urdf'} if root.tag.startswith('{') else {}
    
    links = {}
    joints = {}
    
    # Parse links
    for link in root.findall('.//link', ns) if ns else root.findall('.//link'):
        link_name = link.get('name')
        links[link_name] = {}
        
        # Parse inertial
        inertial = link.find('inertial', ns) if ns else link.find('inertial')
        if inertial is not None:
            origin = inertial.find('origin', ns) if ns else inertial.find('origin')
            mass_elem = inertial.find('mass', ns) if ns else inertial.find('mass')
            inertia_elem = inertial.find('inertia', ns) if ns else inertial.find('inertia')
            
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0')
                rpy = origin.get('rpy', '0 0 0')
                links[link_name]['inertial_pos'] = xyz_to_pos(xyz)
                links[link_name]['inertial_rpy'] = [float(x) for x in rpy.split()]
            
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
        
        origin = joint.find('origin', ns) if ns else joint.find('origin')
        if origin is not None:
            xyz = origin.get('xyz', '0 0 0')
            rpy = origin.get('rpy', '0 0 0')
            joints[joint_name]['origin_xyz'] = xyz_to_pos(xyz)
            joints[joint_name]['origin_rpy'] = [float(x) for x in rpy.split()]
        
        axis = joint.find('axis', ns) if ns else joint.find('axis')
        if axis is not None:
            xyz = axis.get('xyz', '1 0 0')
            joints[joint_name]['axis'] = xyz
        
        limit = joint.find('limit', ns) if ns else joint.find('limit')
        if limit is not None:
            joints[joint_name]['lower'] = float(limit.get('lower', '0'))
            joints[joint_name]['upper'] = float(limit.get('upper', '0'))
            joints[joint_name]['effort'] = float(limit.get('effort', '0'))
    
    return links, joints

def map_urdf_to_mjcf_names():
    """Map URDF link/joint names to MJCF body/joint names."""
    # This mapping is based on the naming conventions
    # URDF uses names like "l_hip_roll", MJCF uses "l_leg_hip_roll"
    name_mapping = {
        # Joints
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
    
    # Links map to bodies (usually same name, but some exceptions)
    # Maps MJCF body names to URDF link names
    link_mapping = {
        'base': 'base',
        'torso_waist_yaw': 'waist_yaw',
        'torso_waist_pitch': 'waist_pitch',
        'torso_waist_roll': 'waist_roll',
        'l_thigh_roll': 'l_thigh_roll',
        'r_thigh_roll': 'r_thigh_roll',
        # Most other links have the same name in both formats
    }
    
    return name_mapping, link_mapping

def update_inertial_from_urdf(body_elem, link_name, urdf_links):
    """Update inertial properties of a body from URDF data."""
    if link_name not in urdf_links:
        return
    
    urdf_data = urdf_links[link_name]
    
    # Find or create inertial element
    inertial = body_elem.find('inertial')
    if inertial is None:
        inertial = ET.SubElement(body_elem, 'inertial')
    
    # Update position
    if 'inertial_pos' in urdf_data:
        inertial.set('pos', urdf_data['inertial_pos'])
    
    # Update quaternion from rpy
    if 'inertial_rpy' in urdf_data:
        quat = rpy_to_quat(urdf_data['inertial_rpy'])
        inertial.set('quat', quat)
    
    # Update mass
    if 'mass' in urdf_data:
        inertial.set('mass', str(urdf_data['mass']))
    
    # Update inertia (convert to diagonal + off-diagonal if needed)
    if 'inertia' in urdf_data:
        inertia = urdf_data['inertia']
        # For now, use diagonal approximation (can be improved)
        diaginertia = f"{inertia['ixx']} {inertia['iyy']} {inertia['izz']}"
        inertial.set('diaginertia', diaginertia)

def update_joint_from_urdf(joint_elem, joint_name, urdf_joints, name_mapping):
    """Update joint properties from URDF data."""
    # Map joint name
    urdf_name = joint_name
    for urdf_key, mjcf_key in name_mapping.items():
        if mjcf_key == joint_name:
            urdf_name = urdf_key
            break
    
    if urdf_name not in urdf_joints:
        return
    
    urdf_data = urdf_joints[urdf_name]
    
    # Update axis
    if 'axis' in urdf_data:
        axis_str = urdf_data['axis']
        axis_parts = axis_str.strip().split()
        joint_elem.set('axis', f"{axis_parts[0]} {axis_parts[1]} {axis_parts[2]}")
    
    # Update range
    if 'lower' in urdf_data and 'upper' in urdf_data:
        joint_elem.set('range', f"{urdf_data['lower']} {urdf_data['upper']}")
    
    # Note: actuatorfrcrange might need to be set separately in actuator section

def process_xml(urdf_path, template_path, output_path):
    """Process URDF and robot.xml template to create GR1T1.xml."""
    print(f"Reading URDF: {urdf_path}...")
    urdf_links, urdf_joints = parse_urdf(urdf_path)
    print(f"  Found {len(urdf_links)} links and {len(urdf_joints)} joints in URDF")
    
    print(f"Reading template: {template_path}...")
    tree = ET.parse(template_path)
    root = tree.getroot()
    
    # Change model name
    root.set('model', 'GR1T1')
    
    # Get name mappings
    joint_name_mapping, link_name_mapping = map_urdf_to_mjcf_names()
    
    # Process all bodies in worldbody
    worldbody = root.find('worldbody')
    if worldbody is not None:
        for body in worldbody.iter('body'):
            body_name = body.get('name')
            if body_name:
                # Try to find matching URDF link
                urdf_link_name = link_name_mapping.get(body_name, body_name)
                update_inertial_from_urdf(body, urdf_link_name, urdf_links)
        
        # Process all joints
        for joint in worldbody.iter('joint'):
            joint_name = joint.get('name')
            if joint_name:
                update_joint_from_urdf(joint, joint_name, urdf_joints, joint_name_mapping)
    
    # Update actuators with effort limits from URDF
    actuator = root.find('actuator')
    if actuator is not None:
        for motor in actuator.findall('motor'):
            joint_name = motor.get('joint')
            if joint_name:
                # Find URDF joint name
                urdf_name = joint_name
                for urdf_key, mjcf_key in joint_name_mapping.items():
                    if mjcf_key == joint_name:
                        urdf_name = urdf_key
                        break
                
                if urdf_name in urdf_joints and 'effort' in urdf_joints[urdf_name]:
                    effort = urdf_joints[urdf_name]['effort']
                    motor.set('ctrlrange', f"{-effort} {effort}")
    
    # Write output with proper formatting
    print(f"Writing {output_path}...")
    ET.indent(tree, space='\t')
    
    with open(output_path, 'wb') as f:
        f.write(b'<?xml version="1.0" encoding="utf-8"?>\n')
        tree.write(f, encoding='utf-8', xml_declaration=False)
    
    print(f"Successfully created {output_path}")
    
    # Validate XML structure
    try:
        ET.parse(output_path)
        print("XML validation: PASSED")
    except ET.ParseError as e:
        print(f"XML validation: FAILED - {e}")
        return False
    
    return True

if __name__ == '__main__':
    import sys
    
    base_dir = Path(__file__).parent
    
    # Default paths
    if len(sys.argv) >= 2:
        urdf_file = Path(sys.argv[1])
    else:
        urdf_file = base_dir / 'GR1T1.urdf'
    
    template_file = base_dir / 'robosuite' / 'models' / 'assets' / 'robots' / 'gr1' / 'robot.xml'
    output_file = base_dir / 'robosuite' / 'models' / 'assets' / 'robots' / 'gr1' / 'GR1T1.xml'
    
    if not urdf_file.exists():
        print(f"Error: URDF file {urdf_file} not found!")
        exit(1)
    
    if not template_file.exists():
        print(f"Error: Template file {template_file} not found!")
        exit(1)
    
    success = process_xml(urdf_file, template_file, output_file)
    exit(0 if success else 1)
