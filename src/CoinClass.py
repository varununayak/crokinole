'''
Coin Class

Author: Varun Nayak
'''



class Coin:

	#identifiers
	ROBOT_COIN = 1 #black
	HUMAN_COIN = 2 #white
	CUE_COIN = 3 #white and in a certain position (predetermined)

	def __init__(self):
		self.x = 0.0	#x position on board
		self.y = 0.0	#y position on board
		self.identity = 2	#identifier for the coin, human coin by default

	#set the position of the center of the coin in the board frame
	def setPosition(self,x,y):
		self.x = x
		self.y = y

	def setIdentity(self,identity):
		if(identity not in [1,2,3]):
			raise Exception('identifier not recognized, use 1 for ROBOT_COIN , 2 for HUMAN_COIN, 3 for CUE_COIN  ')
		else:
			self.identity = identity




####-------TEST HARNESS--------###

#'''

def main():

	coins = []

	coin1 = Coin()
	coin1.setPosition(1,2)
	coin1.setIdentity(1)

	coins.append(coin1)

	coins.append(coin1)

	print(coins[0].x,coins[0].y,coins[0].identity)





if __name__ == "__main__":
	main()

#'''

####---------------------------####