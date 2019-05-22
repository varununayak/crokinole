'''

'''

from CoinClass import Coin
from CoinsUpdater import updateBoardCoins

coins = updateBoardCoins()


for coin in coins:
	print(coin.x,coin.y,coin.identity)