import pybullet as pyb
import pyrosim.pyrosim as ps
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import Sensor
from motor import Motor
import constants as c
import os
from math import sqrt
from statistics import mean, stdev
from icecream import ic


class Robot:
    def __init__(self, id, brain_filename=None):
        if brain_filename is None:
            brain_filename = f'brain{id}.nndf'

        self.id = id
        self.sensors = {}
        self.motors = {}
        self.nn = NEURAL_NETWORK(brain_filename)
        os.system(f'del brain{id}.nndf')

        self.robot_id = pyb.loadURDF('body.urdf')
        ps.Prepare_To_Simulate(self.robot_id)
        self.prepare_to_sense()
        self.prepare_to_act()

        # Fitness Tracking
        self.torso_z_coords = []
        self.torso_rotations = []
        self.y_velocities = []
        self.ang_velocities = []
        self.back_left_leg_angles = []
        self.back_right_leg_angles = []
        self.front_left_leg_angles = []
        self.front_right_leg_angles = []

    def prepare_to_sense(self):
        self.sensors = {}
        for link_name in ps.linkNamesToIndices:
            self.sensors[link_name] = Sensor(link_name)

    def sense(self, t):
        for sensor in self.sensors.values():
            sensor.get_value(t)

    def think(self, t):
        # pass
        self.nn.Update()

    def prepare_to_act(self):
        self.motors = {}
        for joint_name in ps.jointNamesToIndices:
            self.motors[joint_name] = Motor(joint_name)

    def act(self, t):
        # Fitness tracking
        # Standing fitness
        torso_state = pyb.getLinkState(self.robot_id, 0)
        self.torso_z_coords.append(torso_state[0][2])
        self.torso_rotations.append(torso_state[1])

        # Momentum & balancing fitness
        velocity, ang_velocity = pyb.getBaseVelocity(self.robot_id, 0)
        self.y_velocities.append(velocity[1])
        self.ang_velocities.append(ang_velocity)

        for neuron_name in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuron_name):
                joint_name = self.nn.Get_Motor_Neurons_Joint(neuron_name)
                desired_angle = c.JOINT_MOTOR_RANGE * self.nn.Get_Value_Of(neuron_name)
                link1, link2 = joint_name.split('_')


                # # Control legs on the same angle
                # if 'Lower' in joint_name:
                #     continue
                # if link1 == 'Torso' and link2.endswith('Leg'):
                #     lower_joint_name = f'{link2}_Lower{link2}'
                #     self.motors[lower_joint_name].set_value(self.robot_id, desired_angle*c.JOINT_MOTOR_RANGE)


                # Joint angle consistency fitness
                if link2 == 'BackLeftLeg':
                    self.back_left_leg_angles.append(desired_angle)
                elif link2 == 'BackRightLeg':
                    self.back_right_leg_angles.append(desired_angle)
                elif link2 == 'FrontLeftLeg':
                    self.front_left_leg_angles.append(desired_angle)
                elif link2 == 'FrontRightLeg':
                    self.front_right_leg_angles.append(desired_angle)


                self.motors[joint_name].set_value(self.robot_id, desired_angle)
                # print('test:', neuron_name, joint_name, desired_angle)

    def get_fitness(self):

        FORWARD_SCALE = 5
        HEIGHT_SCALE = 0
        HEIGHT_CONSISTENCY_SCALE = 3
        BALANCING_SCALE = 3
        LEG_MOVEMENT_SCALE = 1
        LEG_CONSISTENCY_SCALE = 3
        LEGS_FITNESS_SCALE = 1

        # Move forward
        torso_x_coord = pyb.getLinkState(self.robot_id, 0)[0][0]
        forward_fitness = FORWARD_SCALE * min(-torso_x_coord, -3 * torso_x_coord)

        # Maximize height
        height_fitness = HEIGHT_SCALE * mean(map(sqrt, self.torso_z_coords))

        # Maximize consistent height
        height_consistency_fitness = HEIGHT_CONSISTENCY_SCALE * -stdev(self.torso_z_coords)

        # Minimize angular momentum
        balancing_fitness = BALANCING_SCALE * -mean(map(lambda t: sum(sqrt(abs(i)) for i in t), self.ang_velocities))

        # Leg swing consistency
        leg_fitnesses = []
        for lst in (self.back_left_leg_angles,
                    self.back_right_leg_angles,
                    self.front_left_leg_angles,
                    self.front_right_leg_angles):
            # Reward changing angles a lot
            directions = []      # -1 or 1
            absolute_diffs = []  # abs(difference)
            for a1, a2 in zip(lst[:-1], lst[1:]):
                absolute_diffs.append(abs(a2 - a1))

                if a2 - a1 > 0:
                    directions.append(1)
                else:
                    directions.append(-1)
            leg_movement_fitness = LEG_MOVEMENT_SCALE * mean(absolute_diffs)

            # Penalize changing directions
            direction_changes = []
            for dir1, dir2 in zip(directions[:-1], directions[1:]):
                if dir1 == dir2:
                    direction_changes.append(0)
                else:
                    direction_changes.append(1)
            leg_consistency_fitness = LEG_CONSISTENCY_SCALE * -mean(direction_changes)

            # Summarize total fitness for this leg
            leg_fitnesses.append(sqrt(leg_movement_fitness) + leg_consistency_fitness)
        legs_fitness = LEGS_FITNESS_SCALE * mean(leg_fitnesses)

        # ic(forward_fitness)
        # ic(height_fitness)
        # ic(balancing_fitness)
        # ic(leg_movement_fitness)
        # ic(leg_consistency_fitness)
        # ic(legs_fitness)
        # 1/0

        fitness = forward_fitness \
                  + height_fitness \
                  + height_consistency_fitness \
                  + balancing_fitness \
                  + legs_fitness

        os.system(f'del fitness{self.id}.txt')
        with open(f'tmp{self.id}.txt', 'w') as f:
            f.write(str(fitness))
        os.system(f'rename tmp{self.id}.txt fitness{self.id}.txt')
