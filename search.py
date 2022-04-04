from hillclimber import HillClimber
from parallelhillclimber import ParallelHillClimber

phc = ParallelHillClimber()
phc.evolve()
# input('Press enter to show the best solution!\n>>> ')
phc.show_best()

print('done!')
# 1/0

# for _ in range(1):
#     os.system('python generate.py')
#     os.system('python simulate.py')
