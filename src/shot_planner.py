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
from random import randint


BOARD_R = 276.225
STARTING_ARC_R = 255.5875
ANGLE_EPSILON = 0.01

'''
    rotation about z axis
'''
def rotate(vector, theta):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return R.dot(vector)

''' 
    given a cue and a target disk (initial position and velocity specified), 
    determine whether or not disks can travel without running into 
    obstacles

    returns True if path is viable, returns False and error code otherwise
    does not alter the data in the cue and target Disk objects
'''

def is_viable_path(cue, target, obstacles):
    # make a copy so original data is not mutated
    cue_cp = copy.deepcopy(cue)
    cue_cp1 = copy.deepcopy(cue)
    target_cp = copy.deepcopy(target)
    center_hole = Disk(0, 0, 0)

    fig = plt.gcf()
    ax = fig.gca()
    ax.add_artist(plt.Circle(center_hole.origin, center_hole.r, color=center_hole.color, fill=False))

    # check if cue disk collides with target disk
    contactpt = collide(cue_cp, target_cp)
    if contactpt is None:
        # print "cue does not collide with target"
        return False, 1

    # check if obstacle is in between cue disk and target disk
    for obstacle in obstacles:
        if target_cp != obstacle:
            contactpt_obs = collide(cue_cp1, obstacle)
            if contactpt_obs is not None:
                obs_l = np.linalg.norm(contactpt_obs - cue.origin)
                target_l = np.linalg.norm(contactpt - cue.origin)
                if obs_l <= target_l: # hits obstacle before target
                    # print obstacle
                    return False, 2

    # check if cue disk reaches 20 point hole
    cue_cp2 = copy.deepcopy(cue_cp)
    cue_cp3 = copy.deepcopy(cue_cp)
    contactpt_ctr = collide(cue_cp2, center_hole)
    if contactpt_ctr is None:
        # print "cue does not reach 20-pt hole"
        return False, 3

    # check if target disk (which is now moving) collides with other obstacles
    for i in range(len(obstacles)):
        if target_cp != obstacles[i]:
            contactpt_obs = collide(target_cp, obstacles[i]) # loop only continues if no collisions occur, i.e. target.direction is not mutated
            if contactpt_obs is not None: # target disk collides with obstacle
                # print "target collides with obstacle", i
                return False, 4

    # target disk does not collide with any obstacle
    # print "target reaches edge of board"

    # check if cue disk runs into obstacle
    for obstacle in obstacles:
        if obstacle != target_cp:
            contactpt_obs = collide(cue_cp3, obstacle)
            if contactpt_obs is not None:
                if contactpt_obs[1] < 0:
                    return False, 5

    # plot 
    ax.add_artist(plt.Circle(cue_cp.origin, cue_cp.r, color=cue_cp.color, linestyle='--', fill=False))
    plt.quiver(cue_cp.origin[0], cue_cp.origin[1], cue_cp.direction[0], cue_cp.direction[1], width=0.005, zorder=10)
    plt.quiver(target_cp.origin[0], target_cp.origin[1], target_cp.direction[0], target_cp.direction[1], width=0.005, zorder=10)
    return True, 0

''' 
    create np array of points along a quarter arc
    input parameters are the radius of the arc and the number of points to generate
'''
def generate_start_pos(radius, n):
    points_along_arc = []
    midpoint = np.array([0, -radius])
    points_along_arc.append(midpoint)
    unit_angle = (np.pi/4)/int((n-1)/2)
    for i in range(1, 1+int((n-1)/2)):
        points_along_arc.append(rotate(midpoint, i*unit_angle))
        points_along_arc.append(rotate(midpoint, -i*unit_angle))
    return points_along_arc


''' 
    given a list of all the disks on the board, choose the optimal shot path
    return the cue disk starting position and angle
'''
def plan_shot(disks):
    fig = plt.gcf()
    ax = fig.gca()

    ax.add_artist(plt.Circle(np.array([0,0]), BOARD_R, color='k', fill=False))

    ''' TODO: remove cue disk from disks '''
    ''' TODO: speed '''

    # separate out opponent disks
    opponent_disks = [];
    for disk in disks:
        if disk.identity == 2:
            opponent_disks.append(disk) 

        # show all existing disks on plot
        ax.add_artist(plt.Circle(disk.origin, disk.r, color=disk.color, fill=False))

    # add posts as obstacles
    obstacles = disks
    post_position = rotate(np.array([0, -80.9625]), np.pi/8)
    for i in range(8):
        post = Disk(post_position[0], post_position[1], 0)
        post.r = 3
        obstacles.append(post)
        ax.add_artist(plt.Circle(post.origin, post.r, color=post.color, fill=False))
        post_position = rotate(post_position, np.pi/4)
    
    # initialize cue disk position
    cue_disk = Disk(-156, -186, 3)
    starting_positions = generate_start_pos(STARTING_ARC_R, 50)

    possible_paths = []

    # find possible path for each starting position
    for start_pos in starting_positions:
        cue_disk.origin = start_pos

        for i in range(len(opponent_disks)):

            # find direct shot to each opponent disk
            u = opponent_disks[i].origin - cue_disk.origin
            cue_disk.direction = u/np.linalg.norm(u)

            # vary the shooting angle
            is_viable, error_code = is_viable_path(cue_disk, opponent_disks[i], obstacles)
            while error_code != 1:
            # while is_viable_path(cue_disk, opponent_disks[i], obstacles):
                if is_viable:
                    angle = np.arctan2(cue_disk.direction[1], cue_disk.direction[0])
                    possible_paths.append((start_pos, angle))

                    ax.add_artist(plt.Circle(cue_disk.origin, cue_disk.r, color=cue_disk.color, fill=False))
                    plt.quiver(cue_disk.origin[0], cue_disk.origin[1], cue_disk.direction[0], cue_disk.direction[1], width=0.005, zorder=10)

                cue_disk.direction = rotate(cue_disk.direction, ANGLE_EPSILON)
                is_viable, error_code = is_viable_path(cue_disk, opponent_disks[i], obstacles)

            cue_disk.direction = rotate(u/np.linalg.norm(u), -ANGLE_EPSILON)

            is_viable, error_code = is_viable_path(cue_disk, opponent_disks[i], obstacles)
            while error_code != 1:
                if is_viable:
                    angle = np.arctan2(cue_disk.direction[1], cue_disk.direction[0])
                    possible_paths.append((start_pos, angle))
                    
                    ax.add_artist(plt.Circle(cue_disk.origin, cue_disk.r, color=cue_disk.color, fill=False))
                    plt.quiver(cue_disk.origin[0], cue_disk.origin[1], cue_disk.direction[0], cue_disk.direction[1], width=0.005, zorder=10)

                cue_disk.direction = rotate(cue_disk.direction, -ANGLE_EPSILON)
                is_viable, error_code = is_viable_path(cue_disk, opponent_disks[i], obstacles)
                
    print "number of possible paths:", len(possible_paths)
    # print possible_start_pos

    # plot circles
    ax.set_xlim(-BOARD_R-50, BOARD_R+50)
    ax.set_ylim(-BOARD_R-50, BOARD_R+50)
    ax.set_aspect('equal')
    plt.show(fig)

    ''' TODO: choose angle based on some cost function'''
    ''' TODO: if possible paths is empty'''
    if not possible_paths:
        path = (np.array([0, -STARTING_ARC_R]), 0)
    else:
        path = possible_paths[randint(0, len(possible_paths)-1)]
    return path


if __name__ == "__main__":
    disks = [Disk(50, 230, 2), Disk(-240, -30, 2)]
    path = plan_shot(disks)
    print path
