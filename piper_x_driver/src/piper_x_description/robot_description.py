"""Compose the project wrist attachment with an expanded upstream robot model."""
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro

ROOT = Path(__file__).parent


def with_wrist_attachment(urdf, srdf, attachment, *, include_sensor_frames=True):
    if attachment == 'none':
        return urdf, srdf
    if attachment != 'gemini336l':
        raise ValueError(f'Unknown wrist attachment: {attachment}')
    robot, semantic = ET.fromstring(urdf), ET.fromstring(srdf)
    links = {link.get('name') for link in robot.findall('link')}
    if not {'link6', 'gripper_base'} <= links:
        raise ValueError('Gemini 336L requires PiPER-X link6 and the AgileX gripper base')
    fragment = xacro.process_file(str(ROOT / 'urdf/gemini336l.urdf.xacro'),
                                  mappings={'include_sensor_frames': str(include_sensor_frames).lower()})
    additions = ET.fromstring(fragment.toxml())
    if links.intersection(link.get('name') for link in additions.findall('link')):
        raise ValueError('Gemini links already present in the robot model')
    robot.extend(additions)
    semantic.extend(ET.parse(ROOT / 'config/moveit/gemini336l.srdf').getroot())
    return ET.tostring(robot, encoding='unicode'), ET.tostring(semantic, encoding='unicode')
