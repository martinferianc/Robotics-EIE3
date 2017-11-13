import copy
import numpy as np
import random
""" testing resample() function
"""
# An array of sample points, weights need to add up to 1
state = [[[1, 2], 0.2], [[2, 2], 0.4], [[3, 3], 0.1], [[4, 4], 0.3]]
print("before resampling: {}".format(state))

new_state = []
# generate histogram and particle coordinates array
particle_coord = copy.deepcopy([point[0] for point in state])
histo = copy.deepcopy([point[1] for point in state])
# generate cumulative sum array
cum_histo = np.cumsum(histo)
# pick particles and form a new set of particles
for loop_no in xrange(len(state)):
    random_no = random.uniform(0, 1)
    chosen_idx = 0
    while (cum_histo[chosen_idx] < random_no):
        chosen_idx += 1
        if (chosen_idx >= len(cum_histo) - 1):
            break
    new_state.append(
        [copy.deepcopy(particle_coord[chosen_idx]), 1 / float(len(state))])
# update state
state = new_state
print("after resampling: {}".format(state))
