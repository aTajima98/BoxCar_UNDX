import Box2D
from Box2D.b2 import *
from Box2D import *

import os
import datetime

curdir = os.getcwd()
print("現在のディレクトリ", curdir)

now = datetime.datetime.now()
filename = 'log_' + now.strftime('%Y%m%d_%H%M%S' + '.csv')
print("ファイル名", filename)


groundPieceWidth = 1.5
groundPieceHeight = 0.15

chassisMaxAxis = 1.1
chassisMinAxis = 0.1
chassisMinDensity = 50
chassisMaxDensity = 100

wheelMaxRadius = 0.5
wheelMinRadius = 0.2
wheelMaxDensity = 30
wheelMinDensity = 10
motorSpeed = 25
gravity = b2Vec2(0.0, -9.81)
doSleep = True
start_position = b2Vec2(1,2)

max_health = 100

MAX_NUM_WHEEL_VERTEX = 8


# 車輪の最大数
MAX_NUM_WHEEL = 2

# 遺伝子の長さ
LEN_GENOME = 20

# 個体数
population_size = 20

#世代更新回数
MAX_GEN = 50

