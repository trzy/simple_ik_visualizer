#
# robot/util/urdf.py
# Bart Trzynadlowski, 2024
#
# URDF file parsing
#

from dataclasses import dataclass
from math import pi
from typing import Dict

import xml.etree.ElementTree as ET


class URDF:
    @dataclass
    class Joint:
        lower_limit: float = 0
        upper_limit: float = 0
        velocity_limit: float = 0

        @property
        def revolutions_per_minute(self):
            # Assume this is for rotational joint, where limits are rads/sec
            revolutions_per_second = self.velocity_limit / (2 * pi)
            return revolutions_per_second * 60

    joints_by_name: Dict[str, Joint]

    def __init__(self, filepath: str):
        tree = ET.parse(source=filepath)

        # Get all joints
        self.joints_by_name = {}
        for child in tree.getroot():
            if child.tag == "joint":
                name = child.get("name")
                if name is None or len(name) == 0:
                    print(f"Error: Skipping joint without name in {filepath}")
                    continue
                joint = URDF.Joint()
                for attribute in child:
                    if attribute.tag == "limit":
                        joint.lower_limit = float(attribute.get("lower")) if not None else 0
                        joint.upper_limit = float(attribute.get("upper")) if not None else 0
                        joint.velocity_limit = float(attribute.get("velocity")) if not None else 0
                self.joints_by_name[name] = joint