'''

'''

from CoinClass import Coin
from CoinsUpdater import updateBoardCoins

coins = updateBoardCoins()


for coin in coins:
	print(coin.origin[0],coin.origin[1],coin.identity)