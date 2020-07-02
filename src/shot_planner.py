'''
    shot_planner.py
    
    given the location of all the coins on the board, choose a shot path and output the
    corresponding cue coin position, impact velocity and impact angle.

    assume elastic collisions. all coordinates are given in the board frame.
'''

from coin import *
import numpy as np
import matplotlib.pyplot as plt
import copy
from random import randint


BOARD_R = 276.225
STARTING_ARC_R = 255.5875
ANGLE_EPSILON = 0.01
MIN_PSI = np.pi/3.5 # 45 deg

'''
    rotation about z axis
'''
def rotate(vector, theta):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return R.dot(vector)

''' 
    given a cue and a target coin (initial position and velocity specified), 
    determine whether or not coins can travel without running into 
    obstacles

    returns True if path is viable, returns False and error code otherwise
    does not alter the data in the cue and target Coin objects
'''

def is_viable_path(cue, target, obstacles, plot=False):
    # make a copy so original data is not mutated
    cue_cp = copy.deepcopy(cue)
    cue_cp1 = copy.deepcopy(cue)
    target_cp = copy.deepcopy(target)
    center_hole = Coin(0, 0, 0)

    fig = plt.gcf()
    ax = fig.gca()

    # check if cue coin collides with target coin
    contactpt = collide(cue_cp, target_cp)
    if contactpt is None:
        # print "cue does not collide with target"
        return False, 1

    # check if obstacle is in between cue coin and target coin
    for obstacle in obstacles:
        if target_cp != obstacle:
            contactpt_obs = collide(cue_cp1, obstacle)
            if contactpt_obs is not None:
                obs_l = np.linalg.norm(contactpt_obs - cue.origin)
                target_l = np.linalg.norm(contactpt - cue.origin)
                if obs_l <= target_l: # hits obstacle before target
                    # print obstacle
                    return False, 2

    # check if cue coin reaches 20 point hole
    cue_cp2 = copy.deepcopy(cue_cp)
    cue_cp3 = copy.deepcopy(cue_cp)
    contactpt_ctr = collide(cue_cp2, center_hole)
    if contactpt_ctr is None:
        # print "cue does not reach 20-pt hole"
        return False, 3

    # check if target coin (which is now moving) collides with other obstacles
    for i in range(len(obstacles)):
        if target_cp != obstacles[i]:
            contactpt_obs = collide(target_cp, obstacles[i]) # loop only continues if no collisions occur, i.e. target.direction is not mutated
            if contactpt_obs is not None: # target coin collides with obstacle
                # print "target collides with obstacle", i
                return False, 4

    # target coin does not collide with any obstacle
    # print "target reaches edge of board"

    # check if cue coin runs into obstacle
    for obstacle in obstacles:
        if obstacle != target_cp:
            contactpt_obs = collide(cue_cp3, obstacle)
            if contactpt_obs is not None:
                # if contactpt_obs[1] < 0:
                return False, 5

    # plot 
    if plot is True:
        # print "plotting"
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
    unit_angle = (np.pi/6)/int((n-1)/2)
    for i in range(1, 1+int((n-1)/2)):
        points_along_arc.append(rotate(midpoint, i*unit_angle))
        points_along_arc.append(rotate(midpoint, -i*unit_angle))
    return points_along_arc


''' 
    given a list of all the coins on the board, choose the optimal shot path
    return the cue coin starting position and angle
'''
def plan_shot(coins):
    fig = plt.gcf()
    ax = fig.gca()

    ax.add_artist(plt.Circle(np.array([0,0]), BOARD_R, color=(191/255.0, 128/255.0, 64/255.0), zorder=-1))

    ''' TODO: remove cue coin from coins '''
    ''' TODO: speed '''

    # separate out opponent coins
    opponent_coins = [];
    obstacles = [];
    for coin in coins:
        if coin.identity != 3:
            obstacles.append(coin)
            # show all existing coins on plot
            ax.add_artist(plt.Circle(coin.origin, coin.r, color=coin.color))
        if coin.identity == 2:
            opponent_coins.append(coin) 


    if not opponent_coins: # aim for center hole
        default_path = (np.array([0, -STARTING_ARC_R]), np.pi/2)
        print "no opponent coin; aiming for center"
        return default_path

    # add posts as obstacles
    # post_position = rotate(np.array([0, -80.9625]), np.pi/8)
    # for i in range(8):
    #     post = Coin(post_position[0], post_position[1], 0)
    #     post.r = 5
    #     obstacles.append(post)
    #     ax.add_artist(plt.Circle(post.origin, post.r, color=post.color, fill=False))
    #     post_position = rotate(post_position, np.pi/4)
    post_position = rotate(np.array([0, -80.9625]), 3*np.pi/8)
    for i in range(4):
        post = Coin(post_position[0], post_position[1], 0)
        post.r = 12
        obstacles.append(post)
        ax.add_artist(plt.Circle(post.origin, 3, color='k', fill=False))
        if i == 1:
            post_position = rotate(post_position, 3*np.pi/4)
        else:
            post_position = rotate(post_position, np.pi/4)


    # initialize cue coin position
    cue_coin = Coin(-156, -186, 3)
    starting_positions = generate_start_pos(STARTING_ARC_R, 50)

    ax.add_artist(plt.Circle(np.array([0, 0]), cue_coin.r, color='k', fill=False))


    possible_paths = []
    failsafe_path = None

    # find possible path for each starting position
    for start_pos in starting_positions:
        cue_coin.origin = start_pos

        for i in range(len(opponent_coins)):

            # find direct shot to each opponent coin
            u = opponent_coins[i].origin - cue_coin.origin
            cue_coin.direction = u/np.linalg.norm(u)

            # vary the shooting angle
            is_viable, error_code = is_viable_path(cue_coin, opponent_coins[i], obstacles)
            while error_code != 1:
            # while is_viable_path(cue_coin, opponent_coins[i], obstacles):
                
                if error_code != 2 and failsafe_path is None: # actually hits target with no obs in between
                    failsafe_angle = np.arctan2(cue_coin.direction[1], cue_coin.direction[0])
                    failsafe_path = (start_pos, failsafe_angle)

                if is_viable:
                    angle = np.arctan2(cue_coin.direction[1], cue_coin.direction[0])
                    if angle >= MIN_PSI and angle <= (np.pi - MIN_PSI):
                        possible_paths.append((start_pos, angle))

                        # ax.add_artist(plt.Circle(cue_coin.origin, cue_coin.r, color=cue_coin.color, fill=False))
                        # plt.quiver(cue_coin.origin[0], cue_coin.origin[1], cue_coin.direction[0], cue_coin.direction[1], width=0.005, zorder=10)

                cue_coin.direction = rotate(cue_coin.direction, ANGLE_EPSILON)
                is_viable, error_code = is_viable_path(cue_coin, opponent_coins[i], obstacles)

            cue_coin.direction = rotate(u/np.linalg.norm(u), -ANGLE_EPSILON)
            is_viable, error_code = is_viable_path(cue_coin, opponent_coins[i], obstacles)
            while error_code != 1:

                if error_code != 2 and failsafe_path is None: # actually hits target with no obs in between
                    failsafe_angle = np.arctan2(cue_coin.direction[1], cue_coin.direction[0])
                    failsafe_path = (start_pos, failsafe_angle)

                if is_viable:
                    angle = np.arctan2(cue_coin.direction[1], cue_coin.direction[0])
                    if angle >= MIN_PSI and angle <= (np.pi - MIN_PSI):
                        possible_paths.append((start_pos, angle))
                        
                        # ax.add_artist(plt.Circle(cue_coin.origin, cue_coin.r, color=cue_coin.color, fill=False))
                        # plt.quiver(cue_coin.origin[0], cue_coin.origin[1], cue_coin.direction[0], cue_coin.direction[1], width=0.005, zorder=10)

                cue_coin.direction = rotate(cue_coin.direction, -ANGLE_EPSILON)
                is_viable, error_code = is_viable_path(cue_coin, opponent_coins[i], obstacles,)
                
    print "number of possible paths:", len(possible_paths)
    # print possible_start_pos


    ''' TODO: choose angle based on some cost function'''
    if not possible_paths:
        if failsafe_path is None:
            print "no failsafe path, aiming for center"
            path = default_path
        else:
            print "sending failsafe path"
            path = failsafe_path
        ax.add_artist(plt.Circle(path[0], cue_coin.r, color = cue_coin.color))
        direction = rotate(np.array([150, 0]), path[1])
        plt.quiver(path[0][0], path[0][1], direction[0], direction[1], units='xy', scale=1, width=3, zorder=10)
    else:
        print "sending random viable path"
        path = possible_paths[randint(0, len(possible_paths)-1)]
        cue_coin.origin = path[0]
        ax.add_artist(plt.Circle(cue_coin.origin, cue_coin.r, color = cue_coin.color))
        direction = rotate(np.array([150, 0]), path[1])
        cue_coin.direction = direction/np.linalg.norm(direction)
        plt.quiver(cue_coin.origin[0], cue_coin.origin[1], direction[0], direction[1], units='xy', scale=1, width=3, zorder=10)
        for target in opponent_coins:
            is_viable_path(cue_coin, target, obstacles, plot=True)


    # plot circles
    ax.set_xlim(-BOARD_R-50, BOARD_R+50)
    ax.set_ylim(-BOARD_R-50, BOARD_R+50)
    ax.set_aspect('equal')
    # ax.set_axis_bgcolor
    plt.show()

    return path


if __name__ == "__main__":
    coins = [Coin(80, 0, 2), Coin(0, -80, 1), Coin(75, -55, 1), Coin(-100, -100, 2)]
    path = plan_shot(coins)
    print(path)
