import copy

import pybullet as pyb

from solution import Solution
from constants import NUM_GENERATIONS


class HillClimber:
    def __init__(self):
        self.parent = Solution()
        self.child = None

    def evolve(self):
        self.parent.evaluate('DIRECT')

        for gen in range(NUM_GENERATIONS):
            s = f'\nGENERATION {gen + 1}/{NUM_GENERATIONS}'
            print(s)
            print('=' * len(s.strip()))
            self.evolve_for_one_generation()

    def evolve_for_one_generation(self):
        self.spawn()
        self.mutate()
        self.child.evaluate('DIRECT')
        self.print_()
        self.select()

    def spawn(self):
        self.child = copy.deepcopy(self.parent)

    def mutate(self):
        self.child.mutate()

    def select(self):
        if self.child.fitness < self.parent.fitness:
            self.parent = self.child

    def print_(self):
        print(f'\nparent: {self.parent.fitness:.4f}, child: {self.child.fitness:.4f}')

    def show_best(self):
        self.parent.evaluate('GUI')
