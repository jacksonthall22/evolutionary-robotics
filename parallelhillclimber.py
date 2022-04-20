from solution import Solution
from constants import NUM_GENERATIONS, POPULATION_SIZE
import copy
import os
from datetime import datetime
import constants as c
from utils import chunkify, pluralize, delete_files

BEST_BRAINS_PATH = 'best_brains'
BEST_BRAIN_FILENAME = 'best_brain.nndf'


class ParallelHillClimber:
    def __init__(self):
        delete_files(file_pattern='brain*.nndf')
        delete_files(file_pattern='fitness*.txt')
        delete_files(file_pattern='tmp*.txt')
        # os.system('del brain*.nndf')
        # os.system('del fitness*.txt')

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
            self.evolve_for_one_generation(gen)

    def evolve_for_one_generation(self, gen):
        self.spawn()
        self.mutate(gen)
        self.evaluate(self.children)
        print(self)
        self.select()

    def spawn(self):
        self.children = {}

        for i, parent in self.parents.items():
            self.children[i] = copy.deepcopy(parent)
            self.children[i].set_id(self.nextAvailableID)
            self.nextAvailableID += 1

    def mutate(self, gen):
        for i, (child, _) in self.children.items():
            '''https://www.desmos.com/calculator/bcasuzmogk'''
            m1 = c.START_NUM_MUTATIONS
            m2 = c.END_NUM_MUTATIONS
            t = gen
            t_final = c.NUM_GENERATIONS

            # End at 1 (linear)
            num_mutations = math.ceil(m1 - ((m1 - m2 + 1) * t) / t_final)

            # End at END_NUM_MUTATIONS (linear)
            # num_mutations = max(1, math.ceil(m1 * (1 - t/t_final)))

            # Decay quadratically
            num_mutations = math.floor(t_final / (t + (t_final / (m1 - m2))) + m2)

            child.mutate(num_mutations=num_mutations)

    def select(self):
        for i, (child, parent) in enumerate(zip(self.children.values(), self.parents.values())):
            if child.fitness > parent.fitness:
                self.parents[i] = child

    def evaluate(self, solutions, max_concurrent=c.MAX_CONCURRENT_SIMS):
        chunks = list(chunkify(solutions.items(), chunk_size=max_concurrent))
        for i, solutions_chunk in enumerate(chunks):
            print(f'\nEvaluating chunk {i+1}/{len(chunks)} ({pluralize(len(solutions_chunk), "robot")})')
            for _, s in solutions_chunk:
                s.start_simulation('DIRECT')

            for _, s in solutions_chunk:
                s.wait_for_simulation_to_end()
                # print('test: solution fitness:', s.fitness)

    def __str__(self):
        return '\n' + '\n'.join((f'[{i}] {p.fitness:9.6f}' for i, p in self.parents.items())) + '\n'

    def show_best(self):
        best_parent = max(self.parents.values(), key=lambda p: p.fitness)
        best_parent.start_simulation('GUI')

    def save_population(self):
        now = datetime.now().strftime("%Y-%m-%d_%I-%M-%S_%p")

        if not os.path.exists(f'{BEST_BRAINS_PATH}/{now}'):
            os.makedirs(f'{BEST_BRAINS_PATH}/{now}', exist_ok=True)

        with open(f'{BEST_BRAINS_PATH}/{now}/fitness_constants.txt', 'w') as f:
            f.write(f'TIME_STEPS = {c.TIME_STEPS}')
            f.write(f'TICKS_PER_SEC = {c.TICKS_PER_SEC}')
            f.write(f'NUM_GENERATIONS = {c.NUM_GENERATIONS}')
            f.write(f'POPULATION_SIZE = {c.POPULATION_SIZE}')
            f.write(f'FORWARD_POS_SCALE = {c.FORWARD_POS_SCALE}')
            f.write(f'FORWARD_VEL_SCALE = {c.FORWARD_VEL_SCALE}\n')
            f.write(f'HEIGHT_SCALE = {c.HEIGHT_SCALE}')
            f.write(f'HEIGHT_CONSISTENCY_SCALE = {c.HEIGHT_CONSISTENCY_SCALE}')
            f.write(f'BALANCING_SCALE = {c.BALANCING_SCALE}')
            f.write(f'UPRIGHT_SCALE = {c.UPRIGHT_SCALE}')
            f.write(f'LEG_MOVEMENT_SCALE = {c.LEG_MOVEMENT_SCALE}')
            f.write(f'LEG_CONSISTENCY_SCALE = {c.LEG_CONSISTENCY_SCALE}')
            f.write(f'LEGS_FITNESS_SCALE = {c.LEGS_FITNESS_SCALE}')
            f.write(f'CONTACT_SCALE = {c.CONTACT_SCALE}')

        for i, parent in enumerate(sorted(self.parents.values(),
                                          key=lambda r: r.fitness,
                                          reverse=True)):
            brain_filepath = f'{BEST_BRAINS_PATH}/{now}/brain_{i}___f={parent.fitness:.6f}.nndf'
            parent.create_brain(filename=brain_filepath)
