from parallelhillclimber import ParallelHillClimber
import simulation
import solution
import numpy as np
import os
import pathlib

DO_SEARCH = 1
USE_SEED_WEIGHTS = 0
USE_MOST_RECENT = 0

if USE_MOST_RECENT:
    POPULATION_PATH = os.path.join('best_brains', '2022-04-19_09-00-57_PM')
    BEST_WEIGHTS_PATH = os.path.join(POPULATION_PATH, 'best_weights.npy')
else:
    BEST_WEIGHTS_PATH = os.path.join(max(pathlib.Path('best_brains').glob('*/'), key=os.path.getmtime), 'best_weights.npy')
SEED_WEIGHTS = np.load(BEST_WEIGHTS_PATH, allow_pickle=True)

if DO_SEARCH:
    phc = ParallelHillClimber(seed_weights=SEED_WEIGHTS if USE_SEED_WEIGHTS else None)
    phc.evolve()
    input('Press enter to show the best solution!\n>>> ')
    phc.show_best()
    phc.save_population()
else:
    print(f'Showing "{BEST_WEIGHTS_PATH}"...')
    s = solution.Solution(0, preset_weights=SEED_WEIGHTS)
    s.start_simulation(direct_or_gui='GUI')
    s.wait_for_simulation_to_end()

print('done!')
