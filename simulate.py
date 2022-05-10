from simulation import Simulation
import sys

direct_or_gui = sys.argv[1]
solution_id = sys.argv[2]
temp_prefix = sys.argv[3]
simulation = Simulation(direct_or_gui, solution_id, temp_prefix=temp_prefix)
simulation.run()
simulation.get_fitness()
