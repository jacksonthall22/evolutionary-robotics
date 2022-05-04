import os

import numpy as np
import matplotlib.pyplot as plt

TEST_PATHS_FITNESS = {
    'constant': 'best_brains/2022-04-25_11-38-48_PM',
    'linear': 'best_brains/2022-04-25_11-43-21_PM',
    'exponential decay': 'best_brains/2022-04-26_12-01-45_AM',
    'negative exponential': 'best_brains/2022-04-26_12-06-43_AM',
}
TEST_PATHS_DISTANCE = {
    'constant': 'best_brains/2022-04-26_12-27-13_AM',
    'linear': 'best_brains/2022-04-26_12-22-20_AM',
    'exponential decay': 'best_brains/2022-04-26_12-34-21_AM',
    'negative exponential': 'best_brains/2022-04-26_12-30-48_AM',
}
DATA_FILENAME = 'fitness_record.npy'

TESTS = {
    'Fitness (Using 12 Heuristics) vs. Generation': TEST_PATHS_FITNESS,
    'Average Forward Displacement vs. Generation': TEST_PATHS_DISTANCE
}

for test_title, test_paths in TESTS.items():
    fig, ax = plt.subplots()
    plt.title(test_title)
    plt.legend(title='Mutation Type')
    for test, path in test_paths.items():
        data = np.load(os.path.join(path, DATA_FILENAME))
        avg = np.mean(data, axis=0)
        std = np.std(data, axis=0)

        # print('Test:', test)
        # print(avg.shape)
        # print(std.shape)
        # print(avg)
        # print(std)
        # print(avg+std)

        x = range(len(avg))
        ax.plot(x, avg, label=test)
        plt.fill_between(range(len(avg)), avg-std, avg+std, alpha=.1)
        leg = ax.legend()
    plt.show()


