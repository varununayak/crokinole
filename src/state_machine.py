'''
State Machine (Master Code) for Crokinole

Author: Varun Nayak

TODO: Combine the whole trajectory into one, so that we need only two states.
'''

#----IMPORT DEPENDENCIES HERE----#
import redis
import ast
import time

from coins_updater import *
from coin import *
from shot_planner import *
from enum import Enum

MODE_CHANGE_KEY = "modechange"
SHOT_ANGLE_KEY = "shotangle"
SHOT_POS_KEY = "shotpos"

# states
class State(Enum):
	WAIT4KEY = 1
	EXECUTING = 2

#-----ADD REDIS KEYS HERE------#
myserver = redis.Redis(decode_responses=True)

#-----STATE TRANSITION FUNCTIONS------#

# Transition from WAIT4KEY to EXECUTING
def transition1():
    # get the board state (coins and identities)
    coins = updateBoardCoins()
    for coin in coins:
        print(coin.origin[0], coin.origin[1], coin.identity)
    # plan the shot and get the shot parameters
    shot = plan_shot(coins)
    # #pass the shot parameters over redis
    x_pos = str(shot[0][0])
    y_pos = str(shot[0][1])
    myserver.set(SHOT_POS_KEY, x_pos + "," + y_pos)
    myserver.set(SHOT_ANGLE_KEY, str(shot[1]))
    # pass key over redis to make controller change state from waiting to executing
    myserver.set(MODE_CHANGE_KEY, "execute")
    pass

# Transition from EXECUTING TO WAIT4KEY
def transition2():
    pass

def main():
    # initialize the state
    state = State.WAIT4KEY
    myserver.set(MODE_CHANGE_KEY, "wait")
	# run state machine
    while (True):
        if(state == State.WAIT4KEY):
            user_input = raw_input(
                "Place the cue coin in the home position and enter 'y' to proceed with robot turn: ")
            if(user_input.lower() == 'y'):
                transition1()  # transition actions
                state = State.EXECUTING  # change the state
                print("Entering EXECUTING state...")
            else:
                print("Okay, will ask once again...")
        elif(state == State.EXECUTING):
            redis_input = myserver.get(MODE_CHANGE_KEY)
            if(redis_input == "wait"):
                transition2()  # transitions actions
                state = State.WAIT4KEY
                print("Going back to WAIT4KEY...")
            else:
                pass
        else:
            pass


if __name__ == '__main__':
    main()