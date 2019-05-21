'''
    shot_planner.py
    
    given the location of all the disks on the board, choose a shot path and output the
    corresponding cue coin position, impact velocity and impact angle.

    assume elastic collisions. all coordinates are given in the board frame.
'''

from disk import *
import numpy as np
import matplotlib.pyplot as plt
import copy


BOARD_R = 500

def is_viable_path(cue, target, obstacles):
    # make a copy so original data is not mutated
    cue_cp = copy.deepcopy(cue)
    target_cp = copy.deepcopy(target)

    '''TODO: what if obstacle is in the way of cue disk and target disk?'''

    fig = plt.gcf()
    ax = fig.gca()

    cp = collide(cue_cp, target_cp)
    if cp is None: # cue disk does not collide with target
        print "cue does not collide with target"
        return False

    # check if target disk (which is now moving) collides with other obstacles
    for i in range(len(obstacles)):
        if target_cp != obstacles[i]:
            cp_obs = collide(target_cp, obstacles[i]) # loop only continues if no collisions occur, i.e. target.direction is not mutated
            if cp_obs is not None: # target disk collides with obstacle
                print "target collides with obstacle", i
                return False

    # target disk does not collide with any obstacle
    print "target reaches edge of board"

    # plot 
    ax.add_artist(plt.Circle(cue_cp.origin, cue_cp.r, color=cue_cp.color, linestyle='--', fill=False))
    plt.quiver(cue_cp.origin[0], cue_cp.origin[1], cue_cp.direction[0], cue_cp.direction[1], width=0.005, zorder=10)
    plt.quiver(target_cp.origin[0], target_cp.origin[1], target_cp.direction[0], target_cp.direction[1], width=0.005, zorder=10)
    return True


if __name__ == "__main__":
    # initialize disks    
    opponent_disks = [Disk(100, 100, 'r'), Disk(40, 290, 'r'), Disk(380, 122, 'r')]
    
    fig = plt.gcf()
    ax = fig.gca()

    # add circle objects for current disks
    for disk in opponent_disks:
        ax.add_artist(plt.Circle(disk.origin, disk.r, color=disk.color, fill=False))
    
    # initialize cue disk position
    cue_disk = Disk(-180, -180, 'b')
    ax.add_artist(plt.Circle(cue_disk.origin, cue_disk.r, color=cue_disk.color, fill=False))

    # find possible paths
    for i in range(len(opponent_disks)):
        print "\ntrying target", i
        u = opponent_disks[i].origin - cue_disk.origin
        cue_disk.direction = u/np.linalg.norm(u)

        is_viable_path(cue_disk, opponent_disks[i], opponent_disks)

    # plot circles
    ax.set_xlim(-BOARD_R, BOARD_R)
    ax.set_ylim(-BOARD_R, BOARD_R)
    ax.set_aspect('equal')
    plt.show(fig)

