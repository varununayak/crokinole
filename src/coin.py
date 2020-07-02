'''
Class that characterizes a coin on the game board
'''


import numpy as np


class Coin:

    # identifiers
    ROBOT = 1  # white
    HUMAN = 2  # black
    CUE = 3  # white and in a certain position (predetermined)

    def __init__(self, x_pos=0.0, y_pos=0.0, identity=2):
        self.origin = np.array([x_pos, y_pos])
        self.r = 15.5  # mm
        self.direction = np.array([0, 0])
        self.identity = identity
        if identity == 2:
            self.color = 'k'
        else:
            self.color = 'w'
        self.speed = 0

    def __eq__(self, other):
        return np.array_equal(self.origin, other.origin)

    def __ne__(self, other):
        return not (np.array_equal(self.origin, other.origin))

    def set_position(self, x, y):
        self.origin = np.array([x, y])

    def set_identity(self, identity):
        if (identity not in [1, 2, 3]):
            raise Exception(
                'identifier not recognized, use 1 for ROBOT coin, 2 for HUMAN coin, 3 for CUE coin')
        else:
            self.identity = identity
            if identity == 2:
                self.color = 'k'
            else:
                self.color = 'w'


''' 
calculates the contact point and resulting unit velocities of the two coins
given the initial velocity direction of the moving coin

returns the contact point and updates the velocity vectors in the input coin
objects
'''
def collide(moving_coin, stationary_coin):
    u = moving_coin.direction  # unit vector

    if np.array_equal(u, np.array([0, 0])):  # moving_coin is not moving
        return None
    np.testing.assert_almost_equal(np.linalg.norm(u), 1)

    projected_dist = np.dot(-moving_coin.origin + stationary_coin.origin, u)
    if projected_dist < 0:  # coin moving in opposite direction
        return None
    l = projected_dist * u
    n = -stationary_coin.origin + moving_coin.origin + l

    if (moving_coin.r + stationary_coin.r) <= np.linalg.norm(n):
        return None

    dist_u = np.sqrt((moving_coin.r + stationary_coin.r)
                     ** 2 - np.linalg.norm(n)**2)
    cp_direction = n - dist_u * u
    np.testing.assert_almost_equal(np.linalg.norm(
        cp_direction), stationary_coin.r+moving_coin.r)

    cp = stationary_coin.origin + \
        (stationary_coin.r/(stationary_coin.r+moving_coin.r)) * cp_direction
    moving_coin.origin = stationary_coin.origin + cp_direction

    r = cp - stationary_coin.origin
    stationary_coin.direction = -r/np.linalg.norm(r)

    t = np.array([-r[1], r[0]])
    u = moving_coin.direction
    if np.dot(u, t) > 0:
        moving_coin.direction = t/np.linalg.norm(t)
    elif np.dot(u, t) < 0:
        moving_coin.direction = -t/np.linalg.norm(t)
    else:
        moving_coin.direction = np.array([0, 0])

    return cp
