from pyrosim.commonFunctions import Save_Whitespace
import math


class JOINT:
    def __init__(self,
                 name,
                 parent,
                 child,
                 type,
                 position,
                 axis,
                 initial_rot=None,
                 lower_limit=-math.pi,
                 upper_limit=math.pi):
        if initial_rot is None:
            initial_rot = (0, 0, 0)

        assert isinstance(axis, (tuple, list)) and len(axis) == 3
        assert isinstance(initial_rot, (tuple, list)) and len(initial_rot) == 3

        self.name = name
        self.parent = parent
        self.child  = child
        self.type   = type
        self.position = position
        self.axis = axis
        self.initial_rot = initial_rot
        self.depth = 1
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

    def Save(self, f):
        Save_Whitespace(self.depth, f)
        f.write(f'<joint name="{self.name}" type="{self.type}">\n')
        Save_Whitespace(self.depth, f)
        f.write(f'   <parent link="{self.parent}"/>\n')

        Save_Whitespace(self.depth, f)
        f.write(f'   <child  link="{self.child}"/>\n')

        Save_Whitespace(self.depth, f)
        initial_rot_str = ' '.join(map(str, self.initial_rot))
        position_str = ' '.join(map(str, self.position))
        f.write(f'   <origin rpy="{initial_rot_str}" xyz="{position_str}" />\n')

        Save_Whitespace(self.depth, f)
        joint_axis_str = ' '.join(map(str, self.axis))
        f.write(f'   <axis xyz="{joint_axis_str}"/>\n')

        Save_Whitespace(self.depth, f)
        f.write(f'   <limit effort="0.0" lower="{self.lower_limit}" upper="{self.upper_limit}" velocity="0.0"/>\n')

        Save_Whitespace(self.depth, f)
        f.write('</joint>\n')
