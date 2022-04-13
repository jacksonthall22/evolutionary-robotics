from hillclimber import HillClimber
from parallelhillclimber import ParallelHillClimber
import simulation

phc = ParallelHillClimber()
phc.evolve()
# input('Press enter to show the best solution!\n>>> ')
phc.show_best()

# s = simulation.Simulation('GUI', 0, brain_filename='best_brain.nndf')
# s.run()

print('done!')
# 1/0

# for _ in range(1):
#     os.system('python generate.py')
#     os.system('python simulate.py')
