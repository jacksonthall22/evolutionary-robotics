from parallelhillclimber import ParallelHillClimber
import simulation

phc = ParallelHillClimber()
phc.evolve()
# input('Press enter to show the best solution!\n>>> ')
phc.show_best()

# s = simulation.Simulation('GUI', 0, brain_filename='best_brain.nndf')
# s.run()

print('done!')
