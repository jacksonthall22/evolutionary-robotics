from pyrosim.commonFunctions import Save_Whitespace


class JOINT:
    def __init__(self, name, parent, child, type, position):
        self.name = name
        self.parent = parent
        self.child  = child
        self.type   = type
        self.position = position
        self.depth = 1

    def Save(self, f, joint_axis):
        Save_Whitespace(self.depth, f)
        f.write(f'<joint name="{self.name}" type="{self.type}">\n')
        Save_Whitespace(self.depth, f)
        f.write(f'   <parent link="{self.parent}"/>\n')

        Save_Whitespace(self.depth, f)
        f.write(f'   <child  link="{self.child}"/>\n')

        Save_Whitespace(self.depth, f)
        p = self.position
        f.write(f'   <origin rpy="0 0 0" xyz="{p[0]} {p[1]} {p[2]}" />\n')

        Save_Whitespace(self.depth, f)
        f.write(f'   <axis xyz="{joint_axis}"/>\n')

        Save_Whitespace(self.depth, f)
        f.write('   <limit effort="0.0" lower="-3.14159" upper="3.14159" velocity="0.0"/>\n')

        Save_Whitespace(self.depth, f)
        f.write('</joint>\n')
