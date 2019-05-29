
import numpy as np

class Disk:
    
    #identifiers
    ROBOT = 1 #black
    HUMAN = 2 #white
    CUE = 3 #white and in a certain position (predetermined)

    def __init__(self, x_pos = 0.0, y_pos = 0.0, identity = 2):
        self.origin = np.array([x_pos, y_pos])
        self.r = 15.5 # mm
        self.direction = np.array([0, 0])
        self.identity = identity
        if identity == ROBOT:
            self.color = 'k'
        else:
            self.color = 'r'

    def __eq__(self, other):
        return np.array_equal(self.origin, other.origin)

    def __ne__(self, other):
        return not (np.array_equal(self.origin, other.origin))

    def set_position(self,x,y):
        self.origin(np.array([x, y]))

    def set_identity(self,identity):
        if (identity not in [ROBOT, HUMAN, CUE]):
            raise Exception('identifier not recognized, use 1 for ROBOT disk, 2 for HUMAN disk, 3 for CUE disk')
        else:
            self.identity = identity


''' 
calculates the contact point and resulting unit velocities of the two disks
given the initial velocity direction of the moving disk

returns the contact point and updates the velocity vectors in the input disk
objects
'''

def collide(moving_disk, stationary_disk):
    u = moving_disk.direction # unit vector

    if np.array_equal(u, np.array([0, 0])): # moving_disk is not moving
        return None
    np.testing.assert_almost_equal(np.linalg.norm(u), 1)

    projected_dist = np.dot(-moving_disk.origin + stationary_disk.origin, u)
    if projected_dist < 0: # disk moving in opposite direction
        return None
    l = projected_dist * u
    n = -stationary_disk.origin + moving_disk.origin + l
    
    if (moving_disk.r + stationary_disk.r) <= np.linalg.norm(n):
        return None

    dist_u = np.sqrt((moving_disk.r + stationary_disk.r)**2 - np.linalg.norm(n)**2)
    cp_direction = n - dist_u * u
    np.testing.assert_almost_equal(np.linalg.norm(cp_direction), stationary_disk.r+moving_disk.r)

    cp = stationary_disk.origin + (stationary_disk.r/(stationary_disk.r+moving_disk.r)) * cp_direction
    moving_disk.origin = stationary_disk.origin + cp_direction

    r = cp - stationary_disk.origin
    stationary_disk.direction = -r/np.linalg.norm(r)

    t = np.array([-r[1], r[0]])
    u = moving_disk.direction
    if np.dot(u, t) > 0:
        moving_disk.direction = t/np.linalg.norm(t)
    elif np.dot(u, t) < 0:
        moving_disk.direction = -t/np.linalg.norm(t)
    else:
        moving_disk.direction = np.array([0, 0])

    return cp

