from solution import Solution
from constants import NUM_GENERATIONS, POPULATION_SIZE
import copy
import os


class ParallelHillClimber:
    def __init__(self):
        os.system('del brain*.nndf')
        os.system('del fitness*.txt')

        self.nextAvailableID = 0

        self.parents = {}
        for i in range(POPULATION_SIZE):
            self.parents[i] = Solution(self.nextAvailableID)
            self.nextAvailableID += 1

        self.children = None

    def evolve(self):
        self.evaluate(self.parents)

        for gen in range(NUM_GENERATIONS):
            s = f'\nGENERATION {gen + 1}/{NUM_GENERATIONS}'
            print(s)
            print('=' * len(s.strip()))
            self.evolve_for_one_generation()

    def evolve_for_one_generation(self):
        self.spawn()
        self.mutate()
        self.evaluate(self.children)
        print(self)
        self.select()

    def spawn(self):
        self.children = {}

        for i, parent in self.parents.items():
            self.children[i] = copy.deepcopy(parent)
            self.children[i].set_id(self.nextAvailableID)
            self.nextAvailableID += 1

    def mutate(self):
        for i, child in self.children.items():
            child.mutate()

    def select(self):
        for i, (child, parent) in enumerate(zip(self.children.values(), self.parents.values())):
            if child.fitness > parent.fitness:
                self.parents[i] = child

    def evaluate(self, solutions):
        for i, s in solutions.items():
            s.start_simulation('DIRECT')

        for i, s in solutions.items():
            s.wait_for_simulation_to_end()
            # print('test: solution fitness:', s.fitness)

    def __str__(self):
        return '\n' + '\n'.join((f'[{i}] {p.fitness:9.6f}' for i, p in self.parents.items())) + '\n'

    def show_best(self):
        best_parent = max(self.parents.values(), key=lambda p: p.fitness)
        best_parent.start_simulation('GUI')
        best_parent.create_brain(filename='best_brain.nndf')
