import math
import pybullet
import CPG
import pyrosim.pyrosim as pyrosim
import pyrosim.constants as c


class NEURON:

    def __init__(self, line, robot_id=None, joint_index=None):
        self.Determine_Name(line)
        self.Determine_Type(line)
        self.Search_For_Link_Name(line)
        self.Search_For_Joint_Name(line)

        self.cpg = None
        self.Search_For_CPG_Params(line)

        self.value = 0.0

        self.robot_id = robot_id
        self.joint_index = joint_index

        # Added by JTH - keep track of {name: (dim0, dim1, dim2, ... dim6)}
        # to avoid duplicate calls to pys.getJointState for every
        # sensor neuron (name-dim0, name-dim1, name-dim2, etc.)
        # self.jointStates = {}
        # self.dim =
        #
        # self.name = None
        # self.value = None
        # self.type = None
        # self.jointName = None
        # self.linkName = None

    def Is_Sensor_Neuron(self):
        return self.type == c.SENSOR_NEURON

    def Is_Hidden_Neuron(self):
        return self.type == c.HIDDEN_NEURON

    def Is_Motor_Neuron(self):
        return self.type == c.MOTOR_NEURON

    def Is_CPG_Neuron(self):
        return self.type == c.CPG_NEURON

    def Update_Sensor_Neuron(self):
        if not self.Is_Sensor_Neuron():
            raise ValueError(f'Unsupported neuron type "{self.type}"')

        self.value = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    def Update_CPG_Neuron(self, t):
        if not self.Is_CPG_Neuron():
            raise ValueError(f'Unsupported neuron type "{self.type}"')

        self.value = self.cpg.values[t]

    def Update_Hidden_Or_Motor_Neuron(self, neurons, synapses):
        if not (self.Is_Motor_Neuron() or self.Is_Hidden_Neuron()):
            raise ValueError(f'Unsupported neuron type "{self.type}"')

        for (pre, post), synapse in synapses.items():
            if post == self.name:
                synapse_weight = synapse.Get_Weight()
                self.value += synapse_weight * neurons[pre].value
        self.Activation()

# -------------------------- Private methods -------------------------

    def Determine_Name(self, line):
        if "name" in line:
            splitLine = line.split('"')
            self.name = splitLine[1]

    def Determine_Type(self, line):
        if "sensor" in line:
            self.type = c.SENSOR_NEURON
        elif "motor" in line:
            self.type = c.MOTOR_NEURON
        elif "cpg" in line:
            self.type = c.CPG_NEURON
        else:
            self.type = c.HIDDEN_NEURON

    def Search_For_Joint_Name(self, line):
        if "jointName" in line:
            splitLine = line.split('"')
            self.jointName = splitLine[5]

    def Search_For_Link_Name(self, line):
        if "linkName" in line:
            splitLine = line.split('"')
            self.linkName = splitLine[5]

    def Search_For_CPG_Params(self, line):
        if '"cpg"' in line:
            s = line.split()
            f = lambda s: s.strip('"').strip("'")
            a = float(f(s[9]))
            p = float(f(s[12]))
            o = float(f(s[15]))
            t = int(f(s[18]))
            name = f(s[3])
            self.cpg = CPG.CPG(name=name, amplitude=a, period=p, offset=o, time_steps=t)

    def Activation(self):
        self.value = math.tanh(self.value)
