import shutil
from solution import Solution
from constants import NUM_GENERATIONS, POPULATION_SIZE
import copy
import os
from datetime import datetime
import constants as c
from utils import chunkify, pluralize, delete_files
import numpy as np
import math
from icecream import ic

BEST_BRAINS_PATH = 'best_brains'
BEST_BRAIN_FILENAME = 'best_brain.nndf'


class ParallelHillClimber:
    def __init__(self, seed_weights=None):
        delete_files(file_pattern='brain*.nndf')
        delete_files(file_pattern='fitness*.txt')
        delete_files(file_pattern='tmp*.txt')

        self.nextAvailableID = 0

        self.parents = {}
        for i in range(POPULATION_SIZE):
            if seed_weights is None:
                solution = Solution(self.nextAvailableID, preset_weights=None)
            else:
                # If starting from a single 'seed' brain, mutate parents right away
                solution = Solution(self.nextAvailableID, preset_weights=seed_weights.copy())
                solution.mutate(num_mutations=c.NUM_MUTATIONS_FROM_SEED)
            self.parents[i] = [solution, None]
            self.nextAvailableID += 1

        self.children = None
        self.fitness_record = np.empty((c.POPULATION_SIZE, c.NUM_GENERATIONS))

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
        self.record_fitnesses(self.children, gen)
        print(self)
        self.select()

    def spawn(self):
        self.children = {}

        for i, (parent, parent_fitness) in self.parents.items():
            child = copy.deepcopy(parent)
            child.set_id(self.nextAvailableID)
            self.children[i] = [child, None]
            self.nextAvailableID += 1

    def mutate(self, gen):
        for i, (child, _) in self.children.items():
            '''https://www.desmos.com/calculator/bcasuzmogk'''
            m1 = c.START_NUM_MUTATIONS
            m2 = c.END_NUM_MUTATIONS
            t = gen
            t_final = c.NUM_GENERATIONS

            # End at 1 (linear)
            # num_mutations = math.ceil(m1 - ((m1 - m2 + 1) * t) / t_final)

            # End at END_NUM_MUTATIONS (linear)
            # num_mutations = max(1, math.ceil(m1 * (1 - t/t_final)))

            # Decay exponentially
            # num_mutations = math.floor(t_final / (t + (t_final / (m1 - m2))) + m2)

            # Use constant value
            num_mutations = c.END_NUM_MUTATIONS

            child.mutate(num_mutations=num_mutations)

    def select(self):
        for i, ((child, child_fitness), (parent, parent_fitness)) in \
                enumerate(zip(self.children.values(), self.parents.values())):
            if child.fitness > parent.fitness:
                self.parents[i] = [child, None]
            else:
                self.parents[i][1] = parent.fitness

    def evaluate(self, solutions, max_concurrent=c.MAX_CONCURRENT_SIMS):
        chunks = list(chunkify(solutions.values(), chunk_size=max_concurrent))
        for i, solutions_chunk in enumerate(chunks):
            print(f'\nEvaluating chunk {i+1}/{len(chunks)} ({pluralize(len(solutions_chunk), "robot")})')
            for solution, fitness in solutions_chunk:
                if fitness is None:
                    solution.start_simulation('DIRECT')
                else:
                    print('test: skipping simulation')

            for j, (solution, fitness) in enumerate(solutions_chunk):
                if fitness is None:
                    solution.wait_for_simulation_to_end()

                solutions[i*max_concurrent + j][1] = solution.fitness
                # print('test: solution fitness:', s.fitness)

    def __str__(self):
        return '\n' + '\n'.join((f'[{i}] {p.fitness:9.6f} {"(updated)" if f is None else "(same)"}'
                                 for i, (p, f) in self.parents.items())) + '\n'

    def show_best(self):
        best_parent = max((p[0] for p in self.parents.values()), key=lambda p: p.fitness)
        best_parent.start_simulation('GUI')
        best_parent.wait_for_simulation_to_end()

    def save_population(self):
        now = datetime.now().strftime("%Y-%m-%d_%I-%M-%S_%p")

        output_path = f'{BEST_BRAINS_PATH}/{now}'
        if not os.path.exists(output_path):
            os.makedirs(output_path, exist_ok=True)

        shutil.copyfile('constants.py', f'{output_path}/constants.py')

        for i, (parent, fitness) in enumerate(self.parents.values()):
            weights_filepath = f'{output_path}/weights_{i}___f={parent.fitness:.6f}.npy'
            np.save(weights_filepath, parent.weights, allow_pickle=True)

        best_weights_filepath = f'{output_path}/best_weights.npy'
        np.save(best_weights_filepath,
                max(self.parents.values(), key=lambda p: p[0].fitness)[0].weights,
                allow_pickle=True)

        fitness_record_filepath = f'{output_path}/fitness_record.npy'
        np.save(fitness_record_filepath,
                self.fitness_record,
                allow_pickle=True)

        print(f'Saved {len(self.parents.items())} brain weights and fitness record to {output_path}.')

    def record_fitnesses(self, solutions, gen):
        assert len(solutions) == c.POPULATION_SIZE

        # Save fitness for every robot for this generation
        fitnesses = np.empty(c.POPULATION_SIZE)
        for i, (_, fitness) in solutions.items():
            fitnesses[i] = fitness
        self.fitness_record[:, gen] = fitnesses
