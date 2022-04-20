from simulation import Simulation
import sys

direct_or_gui = sys.argv[1]
solution_id = sys.argv[2]
simulation = Simulation(direct_or_gui, solution_id)
simulation.run()
simulation.get_fitness()
