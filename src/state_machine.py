'''
State Machine (Master Code) for Crokinole

Author: Varun Nayak
'''

#----IMPORT DEPENDENCIES HERE----#
import redis
import ast
import time

STATE_CHANGE_KEY = "statechange"


#states
WAIT4KEY = 1
MOVECUE = 2
FLICKCUE = 3


#-----ADD REDIS KEYS HERE------#
myserver = redis.Redis(decode_responses=True)



#-----STATE TRANSITION FUNCTIONS------#


#transition from WAIT4KEY to MOVECUE
def transition1():
	#get the board state (coins and identities)
	#plan the shot and get the shot parameters
	#generate the trajectory for moving the cue (may need redis)
	#execute the trajectory (may need redis)
	pass

#transition from MOVECUE to FLICKCUE
def transition2():
	#get the position of the cue coin (optional)
	#generate the flick trajectory (may need redis)
	#execute the trajectroy (may need redis)
	pass

#transition from FLICKCUE TO WAIT4KEY
def transition3():
	#clean up latched redis values
	#do something else
	pass



def main():

	#initialize the state
	state = WAIT4KEY

	while(True):

		if(state == WAIT4KEY):
			user_input = raw_input("Place the cue coin in the home position and enter 'y' to proceed with robot turn: ")			
			if(user_input.lower() == 'y'):
				transition1()	#transition actions
				state = MOVECUE	#change the state
				print("Entering MOVECUE state...")
			else:
				print("Okay, will ask once again...")

		elif(state == MOVECUE):			
			redis_input = myserver.get(STATE_CHANGE_KEY)
			if(redis_input == "done_cue_move"):
				transition2()	#transitions actions
				state = FLICKCUE				
				print("Entering FLICKCUE state...")
			else:
				pass

		elif(state == FLICKCUE):
			redis_input = myserver.get(STATE_CHANGE_KEY)
			if(redis_input == "done_cue_flick"):
				transition3()	#transition actions
				state = WAIT4KEY
				print("Entering WAIT4KEY state...")
			else:
				pass
				
		else:
			pass

	




if __name__ == '__main__':
	main()


