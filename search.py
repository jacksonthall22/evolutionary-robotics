from parallelhillclimber import ParallelHillClimber
import simulation

DO_SEARCH = 1

if DO_SEARCH:
    phc = ParallelHillClimber()
    phc.evolve()
    input('Press enter to show the best solution!\n>>> ')
    phc.show_best()
    phc.save_population()
else:
    s = simulation.Simulation('GUI', 0, brain_filename='best_brain.nndf')
    s.run()

print('done!')
