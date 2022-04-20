import pybullet as pyb
import pyrosim.pyrosim as ps
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import Sensor
from motor import Motor
import constants as c
import os
import math
from statistics import mean, stdev
from icecream import ic
from constants import CUBE_HEIGHT_SCALE,\
                      FORWARD_POS_SCALE, \
                      FORWARD_VEL_SCALE, \
                      HEIGHT_SCALE, \
                      HEIGHT_CONSISTENCY_SCALE, \
                      BALANCING_SCALE, \
                      UPRIGHT_SCALE, \
                      LEG_MOVEMENT_SCALE, \
                      LEG_CONSISTENCY_SCALE, \
                      LEGS_FITNESS_SCALE, \
                      CONTACT_SCALE
from utils import delete_files#, HideOutput


class Robot:
    def __init__(self, id, brain_filepath=None):
        if brain_filepath is None:
            brain_filepath = f'brain{id}.nndf'

        self.id = id
        self.sensors = {}
        self.motors = {}
        self.robot_id = pyb.loadURDF('body.urdf')
        self.cube_id = pyb.loadURDF('cube.urdf')
        self.nn = NEURAL_NETWORK(brain_filepath)  # , robot_id=self.robot_id
        delete_files(file=f'brain{id}.nndf')

        ps.Prepare_To_Simulate(self.robot_id)
        self.prepare_to_sense()
        self.prepare_to_act()

        # Fitness Tracking
        self.cube_z_coords = []
        self.torso_z_coords = []
        self.torso_rotations = []
        self.velocities = []
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

        # Pick up cube
        # cube_state = pyb.getLinkState()
        # ic(cube_state)
        # 1/0
        pos, ori = pyb.getBasePositionAndOrientation(self.cube_id, 0)
        cube_z = pos[2]
        self.cube_z_coords.append(cube_z)

        # Standing fitness
        torso_state = pyb.getLinkState(self.robot_id, 0)
        self.torso_z_coords.append(torso_state[0][2])
        self.torso_rotations.append(torso_state[1])

        # Momentum & balancing fitness
        velocity, ang_velocity = pyb.getBaseVelocity(self.robot_id, 0)
        self.velocities.append(velocity)
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
                #     self.motors[lower_joint_name].set_value(self.robot_id, desired_angle)


                # Control grabbers on the same angle
                if link2 == 'Grabber2':
                    continue
                if link2 == 'Grabber1':
                    lower_joint_name = f'{link1}_{link2[:-1]}2'
                    self.motors[lower_joint_name].set_value(self.robot_id, -desired_angle)


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

        # Maximize height of Cube
        # Exponentially scale reward (biggest reward for picking it up quickly)
        r_0 = 5                             # Reward scale at t=0
        r_f = 1                             # Reward scale at t=final
        t_final = len(self.cube_z_coords)   # Number of time steps to scale over
        scales = [math.floor(t_final / (t + (t_final / (r_0 - r_f))) + r_f) for t in range(t_final)]
        scaled_cube_heights = (z_coord * scale for z_coord, scale in zip(self.cube_z_coords, scales))
        cube_height_fitness = mean(scaled_cube_heights)

        # Do not exponentially scale reward
        # cube_height_fitness = mean(self.cube_z_coords) * min(self.cube_z_coords) + stdev(self.cube_z_coords)

        cube_height_fitness *= CUBE_HEIGHT_SCALE

        # # Maximize forward distance
        # torso_x_coord = pyb.getLinkState(self.robot_id, 0)[0][0]
        # forward_fitness = FORWARD_POS_SCALE * min(-torso_x_coord, -3 * torso_x_coord)
        #
        # # Maximize forward velocity
        # mean_x_velocity = mean(x for x, _, _ in self.velocities)
        # forward_vel_fitness = FORWARD_VEL_SCALE * min(-mean_x_velocity, -3 * mean_x_velocity)
        #
        # # Maximize height
        # height_fitness = HEIGHT_SCALE * mean(map(sqrt, self.torso_z_coords))
        #
        # # Maximize consistent height
        # height_consistency_fitness = HEIGHT_CONSISTENCY_SCALE * -stdev(self.torso_z_coords)
        #
        # # Minimize angular momentum
        # balancing_fitness = BALANCING_SCALE * -mean(map(lambda t: sum(sqrt(abs(i)) for i in t), self.ang_velocities))
        #
        # # Stay upright
        # sideways_amounts = []
        # for roll, pitch, _, _ in self.torso_rotations:
        #     sideways_amounts.append(sqrt(abs(roll)) + sqrt(abs(pitch)))
        # mean_sideways_amount = mean(sideways_amounts)
        # upright_fitness = UPRIGHT_SCALE * min(-mean_sideways_amount, -3 * mean_sideways_amount)
        #
        # # Leg swing consistency
        # leg_fitnesses = []
        # for lst in (self.back_left_leg_angles,
        #             self.back_right_leg_angles,
        #             self.front_left_leg_angles,
        #             self.front_right_leg_angles):
        #     # Reward changing angles
        #     directions = []      # -1 or 1
        #     absolute_diffs = []  # abs(difference)
        #     for a1, a2 in zip(lst[:-1], lst[1:]):
        #         absolute_diffs.append(abs(a2 - a1))
        #
        #         if a2 - a1 > 0:
        #             directions.append(1)
        #         else:
        #             directions.append(-1)
        #     leg_movement_fitness = LEG_MOVEMENT_SCALE * mean(absolute_diffs)
        #
        #     # Penalize changing direction of motion
        #     direction_changes = []
        #     for dir1, dir2 in zip(directions[:-1], directions[1:]):
        #         if dir1 == dir2:
        #             direction_changes.append(0)
        #         else:
        #             direction_changes.append(1)
        #     leg_consistency_fitness = LEG_CONSISTENCY_SCALE * -mean(direction_changes)
        #
        #     # Summarize total fitness for this leg
        #     leg_fitnesses.append(sqrt(leg_movement_fitness) + leg_consistency_fitness)
        # legs_fitness = LEGS_FITNESS_SCALE * mean(leg_fitnesses)
        #
        # # Keep lower legs in contact with the ground
        # contact_points = []
        # for sensor_name, sensor in self.sensors.items():
        #     if 'Lower' in sensor_name and 'Leg' in sensor_name:
        #         contact_points.extend(sensor.values)
        # contact_fitness = CONTACT_SCALE * mean(contact_points)

        # ic(cube_height_fitness)
        # ic(forward_fitness)
        # ic(forward_vel_fitness)
        # ic(height_fitness)
        # ic(balancing_fitness)
        # ic(upright_fitness)
        # ic(leg_movement_fitness)
        # ic(leg_consistency_fitness)
        # ic(legs_fitness)
        # ic(contact_fitness)
        # 1/0

        fitness = cube_height_fitness #\
                  # + forward_fitness \
                  # + forward_vel_fitness \
                  # + height_fitness \
                  # + height_consistency_fitness \
                  # + balancing_fitness \
                  # + upright_fitness \
                  # + legs_fitness \
                  # + contact_fitness

        with open(f'tmp{self.id}.txt', 'w') as f:
            f.write(str(fitness))
        os.rename(f'tmp{self.id}.txt', f'fitness{self.id}.txt')
