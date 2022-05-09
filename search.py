from parallelhillclimber import ParallelHillClimber
import solution
import constants as c

import os
import sys
import numpy as np
import pathlib
import time
from mutation_type import MutationType


DO_SEARCH = 1
USE_SEED_WEIGHTS = 0
USE_MOST_RECENT = 0
SHOW_BEST = 1
WAIT_BEFORE_SHOW_BEST = 1


# Get seed weights
if USE_SEED_WEIGHTS:
    if USE_MOST_RECENT:
        POPULATION_PATH = max(pathlib.Path('best_brains').glob('*/'), key=os.path.getmtime)
    else:
        POPULATION_PATH = 'best_brains/2022-04-25_10-20-44_PM'
    BEST_WEIGHTS_PATH = os.path.join(POPULATION_PATH, 'best_weights.npy')
    SEED_WEIGHTS = np.load(BEST_WEIGHTS_PATH, allow_pickle=True)
else:
    SEED_WEIGHTS = None

if DO_SEARCH:
    start_time = time.time()

    phc = ParallelHillClimber(seed_weights=SEED_WEIGHTS)
    phc.evolve()

    elapsed = time.time() - start_time
    if elapsed < 60:
        time_str = f'{elapsed:.2f}s'
    elif elapsed < 3600:
        time_str = f'{elapsed / 60:.2f}m'
    else:
        time_str = f'{elapsed / 3600:.2f}hr'
    print(f'Evolution finished in {time_str}.')

    phc.save_population()

    if SHOW_BEST:
        if WAIT_BEFORE_SHOW_BEST:
            input('Press enter to show the best solution!\n>>> ')

        print('Showing best...')
        phc.show_best()
else:
    print(f'Showing best robot from population "{POPULATION_PATH}"...')
    s = solution.Solution(0, preset_weights=SEED_WEIGHTS)
    s.start_simulation(direct_or_gui='GUI')

print('done!')
