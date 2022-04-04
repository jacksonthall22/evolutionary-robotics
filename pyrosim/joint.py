from pyrosim.commonFunctions import Save_Whitespace


class JOINT:
    def __init__(self, name, parent, child, type, position):
        self.name = name
        self.parent = parent
        self.child  = child
        self.type   = type
        self.position = position
        self.depth = 1

    def Save(self, f, joint_axis, initial_angle=None):
        if initial_angle is None:
            initial_angle = (0, 0, 0)

        assert isinstance(joint_axis, (tuple, list)) \
               and len(joint_axis) == 3
        assert isinstance(initial_angle, (tuple, list)) \
               and len(initial_angle) == 3

        Save_Whitespace(self.depth, f)
        f.write(f'<joint name="{self.name}" type="{self.type}">\n')
        Save_Whitespace(self.depth, f)
        f.write(f'   <parent link="{self.parent}"/>\n')

        Save_Whitespace(self.depth, f)
        f.write(f'   <child  link="{self.child}"/>\n')

        Save_Whitespace(self.depth, f)
        initial_angle_str = ' '.join(map(str, initial_angle))
        position_str = ' '.join(map(str, self.position))
        f.write(f'   <origin rpy="{initial_angle_str}" xyz="{position_str}" />\n')

        Save_Whitespace(self.depth, f)
        joint_axis_str = ' '.join(map(str, joint_axis))
        f.write(f'   <axis xyz="{joint_axis_str}"/>\n')

        Save_Whitespace(self.depth, f)
        f.write('   <limit effort="0.0" lower="-3.14159" upper="3.14159" velocity="0.0"/>\n')

        Save_Whitespace(self.depth, f)
        f.write('</joint>\n')
