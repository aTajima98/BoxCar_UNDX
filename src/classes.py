import Box2D
from Box2D.b2 import *
from Box2D import *

import pygame
from pygame.locals import *

import setting
import math
import random
import numpy as np
import csv
import datetime
import os
import copy

pygame.init()

class car_data:
	def __init__(self,chassis,wheels,car_def,xy_pos=[0,0],linear_vel=0):
		self.xy_pos = xy_pos #[x,y]
		self.linear_vel = linear_vel
		self.health = setting.max_health
		self.isDead = False
		self.chassis = chassis
		self.wheels = wheels
		self.max_dist = 0
		self.car_def = car_def
	def kill_it(self):
		self.health = 0
		self.isDead = True
	def getHealth(self):
		return self.health
	def isDead(self):
		return self.isDead
	def dcr_health(self):
		self.health -= 2
	def get_vel(self):
		return self.linear_vel
	def get_pos_x(self):
		return self.xy_pos[0]
	def get_pos(self):
		return self.xy_pos
	def set_pos_and_vel(self,pos,vel):
		if not self.isDead:
			self.xy_pos = pos
			self.linear_vel = vel
			self.update_health()
			self.update_max_dist()
	def update_health(self):
		# if self.linear_vel < 0.0001:
		if self.linear_vel < 0.01:
			self.dcr_health()
			if self.health <= 0:
				self.kill_it()
	def print_info(self):
		if not self.isDead:
			print("Velocity:",self.linear_vel," Position:",self.xy_pos," Health:",self.health)
		else:
			pass
			# print( "Dead")
	def update_max_dist(self):
		self.max_dist = self.xy_pos[0] # -setting.start_position.x
		# print("{} - {} = {}".format(self.xy_pos[0], setting.start_position.x, self.max_dist))


class car:
	def __init__(self, world, random = True, gene = None):

		self.world = world
		if random:
			gene = self.init_gene()
			self.car_def = self.make_car_from_gene(gene)
			# self.car_def = self.make_random_car()
		else:
			# self.car_def = car_def
			self.car_def = self.make_car_from_gene(gene)

		self.alive = True;
		self.velocityIndex = 0;
		self.chassis = self.create_chassis(self.car_def.vertex_list, self.car_def.chassis_density)
		self.wheels = []
		for i in range(self.car_def.wheel_count):
			self.wheels.append(self.create_wheel(self.car_def.wheel_radius[i], self.car_def.wheel_density[i]))
		#carmass = 2+1 #(2 for wheels and 1 for body)
		carmass = self.chassis.mass
		for i in range(self.car_def.wheel_count):
			carmass += self.wheels[i].mass

		carmass = 2+1
		#better: theres a getMassData method for b2Body - check that!
		self.torque = []
		for i in range(self.car_def.wheel_count):
			self.torque.append(carmass * -setting.gravity.y / self.car_def.wheel_radius[i])

		self.joint_def = b2RevoluteJointDef()


		for i in range(self.car_def.wheel_count):
			randvertex = self.chassis.vertex_list[self.car_def.wheel_vertex[i]]
			self.joint_def.localAnchorA.Set(randvertex.x, randvertex.y)
			self.joint_def.localAnchorB.Set(0, 0)
			self.joint_def.maxMotorTorque = self.torque[i]
			self.joint_def.motorSpeed = -setting.motorSpeed
			self.joint_def.enableMotor = True
			self.joint_def.collideConnected = False
			self.joint_def.bodyA = self.chassis
			self.joint_def.bodyB = self.wheels[i]
			joint = self.world.CreateJoint(self.joint_def)

		#print "->",self.chassis.fixtures[0].type

	# 初期個体の生成（ランダム）
	def init_gene(self):
		gene = [0] * setting.LEN_GENOME
		for gene_i in range(len(gene)):
			gene[gene_i] = random.random()
		return gene

	# 遺伝子情報から車の設計変数値に変換
	def make_car_from_gene(self, gene):
		# gen[19]=タイヤの数(1~8個の間で) *** 確率50%
		check_num = round(gene[19], 3) * 100
		chek_range = 50
		if 0<=check_num<chek_range:
			wheel_num = 1
		else:
			wheel_num = 2

		make_car = car_info(wheel_num)
		wheel_radius_values = []
		wheel_density_values = []
		vertex_list = []
		wheel_vertex_values = []

		# gen[19]=タイヤの数をセット
		make_car.set_wheel_count(wheel_num)

		#gene[0,1]=タイヤの半径, gene[4,5]=タイヤの密度
		for i in range(setting.MAX_NUM_WHEEL):
			wheel_radius_values.append(gene[i]*(setting.wheelMaxRadius - setting.wheelMinRadius) + setting.wheelMinRadius)
			wheel_density_values.append(gene[i+4]*(setting.wheelMaxDensity - setting.wheelMinDensity) + setting.wheelMinDensity)

		#gene[2,3]=wheel_vertex_values=タイヤがつく頂点の場所

		#タイヤが被らないように　＆　タイヤの位置を決めてる
		index_left = [i for i in range(setting.MAX_NUM_WHEEL)]
		for wheel_i in range(make_car.get_wheel_count()):   # タイヤを付ける場所を決めてる***
			index_of_next = int(gene[wheel_i + setting.MAX_NUM_WHEEL] * (len(index_left)-1))
			wheel_vertex_values.append(int(index_left[index_of_next]))
			index_left = index_left[:index_of_next] + index_left[index_of_next+1:]

		#gene[6:17]=vertex_list[0:7]=車体の各頂点座標
		vertex_list.append(b2Vec2(gene[7]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis,0))
		vertex_list.append(b2Vec2(gene[8]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, gene[9]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(0,gene[10]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(-gene[11]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, gene[12]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(-gene[13]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, 0))
		vertex_list.append(b2Vec2(-gene[14]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, -gene[15]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(0,-gene[16]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(gene[17]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, -gene[18]*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))

		#now, setting all values (these are all the attibutes required to completely describe a car)
		make_car.set_vertex_list(vertex_list)
		make_car.set_wheel_radius(wheel_radius_values)
		make_car.set_wheel_density(wheel_density_values)
		make_car.set_wheel_vertex(wheel_vertex_values)

		#gene[18]=chassis_Density
		make_car.set_chassis_density(gene[18]*(setting.chassisMaxDensity - setting.chassisMinDensity) + setting.chassisMinDensity)

		for gene_i in range(len(gene)):
			make_car.car_genome[gene_i] = gene[gene_i]

		return make_car


	def make_random_car(self):
		# タイヤの数を1～2個で決める
		wheel_num = random.randint(1,2)
		random_car = car_info(wheel_num)
		wheel_radius_values = []
		wheel_density_values = []
		vertex_list = []
		wheel_vertex_values = []

		for i in range(random_car.get_wheel_count()):
			wheel_radius_values.append(random.random()*(setting.wheelMaxRadius - setting.wheelMinRadius) + setting.wheelMinRadius) #設計変数0,1を決めてる
			wheel_density_values.append(random.random()*(setting.wheelMaxDensity - setting.wheelMinDensity) + setting.wheelMinDensity) #設計変数4,5を決めてる

		#車体の形を決める ランダムに決めてる
		vertex_list.append(b2Vec2(random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, 0))
		vertex_list.append(b2Vec2(random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(0, random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(-random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis, random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(-random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis,0))
		vertex_list.append(b2Vec2(-random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis,-random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(0,-random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))
		vertex_list.append(b2Vec2(random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis,-random.random()*(setting.chassisMaxAxis - setting.chassisMinAxis) + setting.chassisMinAxis))

		#index_leftを定義 8はどういう意味？　タイヤが最大8個つく？　MAX_NUM_WHEEL
		MAX_NUM_WHEEL = 8
		index_left = [i for i in range(MAX_NUM_WHEEL)]

		# print index_left
		for i in range(random_car.get_wheel_count()):
			index_of_next = int(random.random() * (len(index_left)-1))
			#print index_of_next
			wheel_vertex_values.append(index_left[index_of_next])
			#remove the last used index from index_left
			index_left = index_left[:index_of_next] + index_left[index_of_next+1:]

		#now, setting all values (these are all the attibutes required to completely describe a car)
		random_car.set_vertex_list(vertex_list)
		random_car.set_wheel_radius(wheel_radius_values)
		random_car.set_wheel_density(wheel_density_values)
		random_car.set_wheel_vertex(wheel_vertex_values)
		random_car.set_chassis_density(random.random()*(setting.chassisMaxDensity - setting.chassisMinDensity) + setting.chassisMinDensity)

		#geneに要素入れる（0~1の範囲に直す）
		random_car.car_genome[0] = (random_car.wheel_radius[0] - setting.wheelMinRadius) / (setting.wheelMaxRadius - setting.wheelMinRadius) #タイヤ1の半径
		random_car.car_genome[1] = (random_car.wheel_radius[1] - setting.wheelMinRadius) / (setting.wheelMaxRadius - setting.wheelMinRadius) #タイヤ2の半径
		random_car.car_genome[2] = float(random_car.wheel_vertex[0] / 8) #タイヤ1がつく頂点箇所
		random_car.car_genome[3] = float(random_car.wheel_vertex[1]/ 8) #タイヤ2がつく頂点箇所
		random_car.car_genome[4] = (random_car.wheel_density[0] - setting.wheelMinDensity) / (setting.wheelMaxDensity - setting.wheelMinDensity) #タイヤ1の密度
		random_car.car_genome[5] = (random_car.wheel_density[1] - setting.wheelMinDensity) / (setting.wheelMaxDensity - setting.wheelMinDensity) #タイヤ2の密度
		random_car.car_genome[6] = (random_car.vertex_list[0][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点1のx
		random_car.car_genome[7] = (random_car.vertex_list[1][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)  #頂点2のx
		random_car.car_genome[8] = (random_car.vertex_list[1][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)  #頂点2のy
		random_car.car_genome[9] = (random_car.vertex_list[2][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点3のy
		random_car.car_genome[10] = -(random_car.vertex_list[3][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点4のｘ
		random_car.car_genome[11] = (random_car.vertex_list[3][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点4のy
		random_car.car_genome[12] = -(random_car.vertex_list[4][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点5のx
		random_car.car_genome[13] = -(random_car.vertex_list[5][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点6のx
		random_car.car_genome[14] = -(random_car.vertex_list[5][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点6のy
		random_car.car_genome[15] = -(random_car.vertex_list[6][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点7のy
		random_car.car_genome[16] = (random_car.vertex_list[7][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点8のx
		random_car.car_genome[17] = -(random_car.vertex_list[7][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis) #頂点8のy
		random_car.car_genome[18] = (random_car.chassis_density - setting.chassisMinDensity) / (setting.chassisMaxDensity - setting.chassisMinDensity) #車体の密度

		#check_range = 125->10000で割る
		check_range = 50
		if random_car.wheel_count == 1:
			check_num = random.randrange(0,check_range-1,1)
		else:
			check_num = random.randrange(check_range,check_range*2,1)
		random_car.car_genome[19] = check_num / 100

		return random_car


	# box2D形式のデータから，GAの形式へ変換
	def to_indivisual(self):

		gene = []

		gene.append((self.wheel_radius[0] - setting.wheelMinRadius) / (setting.wheelMaxRadius - setting.wheelMinRadius)) #タイヤ1の半径
		gene.append((self.wheel_radius[1] - setting.wheelMinRadius) / (setting.wheelMaxRadius - setting.wheelMinRadius)) #タイヤ2の半径
		gene.append(float(self.wheel_vertex[0] / 8)) #タイヤ1がつく頂点箇所
		gene.append((self.wheel_vertex[1]/ 8)) #タイヤ2がつく頂点箇所
		gene.append((self.wheel_density[0] - setting.wheelMinDensity) / (setting.wheelMaxDensity - setting.wheelMinDensity)) #タイヤ1の密度
		gene.append((self.wheel_density[1] - setting.wheelMinDensity) / (setting.wheelMaxDensity - setting.wheelMinDensity)) #タイヤ2の密度
		gene.append((self.vertex_list[0][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点1のx
		gene.append((self.vertex_list[1][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis))  #頂点2のx
		gene.append((self.vertex_list[1][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis))  #頂点2のy
		gene.append((self.vertex_list[2][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点3のy
		gene.append(-(self.vertex_list[3][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点4のｘ
		gene.append((self.vertex_list[3][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点4のy
		gene.append(-(self.vertex_list[4][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点5のx
		gene.append(-(self.vertex_list[5][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点6のx
		gene.append(-(self.vertex_list[5][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点6のy
		gene.append((self.vertex_list[6][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点7のy
		gene.append((self.vertex_list[7][0] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点8のx
		gene.append(-(self.vertex_list[7][1] - setting.chassisMinAxis) / (setting.chassisMaxAxis - setting.chassisMinAxis)) #頂点8のy
		gene.append((self.chassis_density - setting.chassisMinDensity) / (setting.chassisMaxDensity - setting.chassisMinDensity)) #車体の密度

		#check_range = 125
		check_range = 50
		if self.wheel_count == 1:
			check_num = random.randrange(0,check_range-1,1)
		else:
			check_num = random.randrange(check_range,check_range*2,1)
		gene.append(check_num / 100)
		return gene

	def create_wheel(self,radius,density):
		body_def = b2BodyDef()
		body_def.type = b2_dynamicBody
		body_def.position.Set(setting.start_position.x,setting.start_position.y)
		body = self.world.CreateBody(body_def)
		fix_def = b2FixtureDef()
		fix_def.shape = b2CircleShape(radius = radius)
		fix_def.density = density
		fix_def.friction = 1
		fix_def.restitution = 0.2
		fix_def.filter.groupIndex = -1
		body.CreateFixture(fix_def)

		#body_def.type = b2_dynamicBody
		#body_def.position.Set(body.worldCenter.x,body.worldCenter.y)
		#line = self.world.CreateBody(body_def,angle=15)
		#box = body.CreatePolygonFixture(box=(0.01,radius/2),density = 0.00001,friction = 0)

		return body


	def create_chassis_part(self,body,vertex1,vertex2,density):
		vertex_list = []
		vertex_list.append(vertex1)
		vertex_list.append(vertex2)
		vertex_list.append(b2Vec2(0,0))
		fix_def = b2FixtureDef()
		fix_def.shape = b2PolygonShape()
		fix_def.density = density
		fix_def.friction = 10
		fix_def.restitution = 0.0
		fix_def.filter.groupIndex = -1
		#fix_def.shape.SetAsArray(vertex_list,3)
		#print "length of vertex in chassis:",len(vertex_list)
		#print vertex_list
		fix_def.shape = b2PolygonShape(vertices=vertex_list)
		body.CreateFixture(fix_def)

	def create_chassis(self,vertex_list,density):
		body_def = b2BodyDef()
		body_def.type = b2_dynamicBody
		body_def.position.Set(setting.start_position.x,setting.start_position.y)	#start position of the car
		body = self.world.CreateBody(body_def)
		for i in range(len(vertex_list)):
			self.create_chassis_part(body, vertex_list[i],vertex_list[(i+1)%8], density)
		body.vertex_list = vertex_list
		return body

	def get_car_chassis(self):
		return self.chassis
	def get_car_wheels(self):
		return self.wheels


	def draw_stuff(self):
		PPM=30.0 # pixels per meter
		TARGET_FPS=60
		TIME_STEP=1.0/TARGET_FPS
		SCREEN_WIDTH, SCREEN_HEIGHT=640,480
		running=True

		screen=pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 32)
		pygame.display.set_caption('Simple pygame example')
		clock=pygame.time.Clock()
		def my_draw_circle(circle, body, fixture):
			colors = {staticBody  : (255,255,255,255),dynamicBody : (127,127,127,255)}
			position=body.transform*circle.pos*PPM
			position=(position[0], SCREEN_HEIGHT-position[1])
			pygame.draw.circle(screen, colors[body.type], [int(x) for x in position], int(circle.radius*PPM))
		b2CircleShape.draw=my_draw_circle

		while running:
			# Check the event queue
			for event in pygame.event.get():
				if event.type==QUIT or (event.type==KEYDOWN and event.key==K_ESCAPE):
					# The user closed the window or pressed escape
					running=False

			screen.fill((0,0,0,0))
			# Draw the world
			for body in self.wheels: # or: world.bodies
				#print body.type
				# The body gives us the position and angle of its shapes
				for fixture in body.fixtures:
					fixture.shape.draw(body, fixture)
			world.Step(TIME_STEP, 10, 10)

			# Flip the screen and try to keep at the target FPS
			pygame.display.flip()
			clock.tick(TARGET_FPS)


class car_info:
	def __init__(self,num_Wheel):
		self.wheel_count = num_Wheel
		self.wheel_radius = [0]*self.wheel_count
		self.wheel_density = [0]*self.wheel_count
		self.wheel_vertex = [0]*self.wheel_count
		self.chassis_density = 1 #default
		self.vertex_list = [0]*8

		#追加項目：染色体
		self.car_genome = [0]*setting.LEN_GENOME

	def get_wheel_count(self):
		return self.wheel_count
	def get_wheel_radius(self):
		return self.wheel_radius
	def get_wheel_density(self):
		return self.wheel_density
	def get_wheel_vertex(self):
		return self.wheel_vertex
	def get_chassis_density(self):
		return self.chassis_density
	def get_vertex_list(self):
		return self.vertex_list

	def set_wheel_count(self,val):
		self.wheel_count = val
	def set_wheel_radius(self,val_list):
		self.wheel_radius = val_list
	def set_wheel_density(self,val_list):
		self.wheel_density = val_list
	# def set_wheel_vertex(self,val_list):
	# 	self.wheel_vertex = val_list
	def set_chassis_density(self,val):
		self.chassis_density = val
	def set_vertex_list(self,val_list):
		self.vertex_list = val_list
	def set_wheel_vertex(self,val_list):
		# index_left = [x for x in range(setting.MAX_NUM_WHEEL)]
		# # for i in range(random_car.get_wheel_count()):
		# for i in range(self.wheel_count):
		# 	index_of_next = int(random.random() * (len(index_left)-1))
		# 	#self.wheel_vertex_values.append(index_left[index_of_next])
		# 	#remove the last used index from index_left
		# 	index_left = index_left[:index_of_next] + index_left[index_of_next+1:]
		self.wheel_vertex = val_list


class terrain:
	def __init__(self,world):
		self.world = world

	def create_floor(self):
		maxFloorTiles = 200
		last_tile = None
		tile_position = b2Vec2(-1,0)
		floor_tiles = []
		random.seed(random.randint(1,39478534))
		for k in range(maxFloorTiles):
			last_tile = self.create_floor_tile(tile_position, (random.random()*3 - 1.5) * 1.2*k/maxFloorTiles)
			floor_tiles.append(last_tile)
			last_fixture = last_tile.fixtures
			#below is the fix for jagged edges: the vertex order was messed up, so sometimes the left bottom corner
			#would be connected to the top right corner of the previous tile
			if last_fixture[0].shape.vertices[3]==b2Vec2(0,0):
				last_world_coords = last_tile.GetWorldPoint(last_fixture[0].shape.vertices[0])
			else:
				last_world_coords = last_tile.GetWorldPoint(last_fixture[0].shape.vertices[3])
			tile_position = last_world_coords
			#print "lasttile position:",last_world_coords
		#print len(floor_tiles)
		#floor_tiles = []
		#floor_tiles.append(create_floor_tile(b2Vec2(50,50), (random.random()*3 - 1.5) * 1.2*3/maxFloorTiles))
		return floor_tiles

	def create_floor_tile(self,position, angle):
		#print "creating next tile at position: ",position
		body_def = b2BodyDef()
		#body_def.position.Set(position.x, position.y)
		body_def.position = position
		body = self.world.CreateBody(body_def)
		fix_def = b2FixtureDef()
		fix_def.shape = b2PolygonShape()
		fix_def.friction = 0.5
		coords = []
		coords.append(b2Vec2(0,0))
		coords.append(b2Vec2(0,setting.groundPieceHeight))
		coords.append(b2Vec2(setting.groundPieceWidth,setting.groundPieceHeight))
		coords.append(b2Vec2(setting.groundPieceWidth,0))
		newcoords = self.rotate_floor_tile(coords, angle)
		#print "length of polygon in tile:",len(newcoords)
		#newcoords[3].y +=0.15
		fix_def.shape = b2PolygonShape(vertices=newcoords) #setAsArray alt

		#print newcoords
		body.CreateFixture(fix_def)
		#print "newtile position: ",body.GetWorldPoint(body.fixtures[0].shape.vertices[0])
		#print "length of a single body's fixure list:",len(body.fixtures) # 1
		#print "all fixures:",body.fixtures #datas in fictures i.e ficture properties
		#print "shape of fixure:",body.fixtures[0].shape
		return body

	def rotate_floor_tile(self,coords, angle):
		newcoords = []
		for k in range(len(coords)):
			nc = b2Vec2(0,0)
			nc.x = math.cos(angle)*(coords[k].x) - math.sin(angle)*(coords[k].y)
			nc.y = math.sin(angle)*(coords[k].x) + math.cos(angle)*(coords[k].y)
			newcoords.append(nc)
		return newcoords


class do_stuff():

	def __init__(self):
		self.world = b2World(gravity=setting.gravity, doSleep=setting.doSleep)

		self.population_size = setting.population_size

		#停止した車の個体数
		self.killed = 0

		t = terrain(self.world)
		self.terrain = t.create_floor()
		#世代数をカウント
		self.gen = 0
		self.population = [] #array of list of [chassis,wheels]
		self.population_data = [] #array of car_data objetcs
	#	self.population.append([self.wheels,self.chassis])
		self.create_generation_1()
		self.leader_coors = [0,0]
		self.leader = self.population[0][0] #chassis of 1st car



		self.draw_any()

		#self.update_car_data()
	def draw_any(self):
		#body aray contains the list of body objects to be draw
		#type = 1 means polygon, type = 2 means circle

		x_offset = 0
		y_offset = 0
		prev_y = 0
		offset_value = 5

		PPM=30.0 # pixels per meter
		TARGET_FPS=60
		TIME_STEP=1.0/TARGET_FPS
		SCREEN_WIDTH, SCREEN_HEIGHT=640,480
		running=True
		screen=pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 32)
		pygame.display.set_caption('実験4（最適化）：BoxCar2D')
		clock=pygame.time.Clock()

		colors = {staticBody  : (136,150,200,255),dynamicBody : (127,127,127,255)}
		leader_coors = self.leader_coors

		def my_draw_circle(circle, body, fixture):
			#print "drawing circle"
			#help(body)
			position=body.transform*circle.pos*PPM

			y_offset = ((self.leader.worldCenter.y)*70)
			if y_offset < -300:
				y_offset = -300
			if y_offset > 300:
				y_offset = 300

			position=(position[0]-self.leader.worldCenter.x*30+350, SCREEN_HEIGHT-position[1]+y_offset*0.5-200)

			#higher the density darker the wheel
			#c = round(255 - (255 * (body.fixtures[0].density - setting.wheelMinDensity)) / setting.wheelMaxDensity)
			#pygame.draw.circle(screen, (c,c,c), [int(x) for x in position], int(circle.radius*PPM),1)

			center = [int(x) for x in position]   #just (x,y) coor
												  #uncomment above if you just want to draw on the screen and without using blit (cant use alpha color value)

			center_s = [int(circle.radius*PPM),int(circle.radius*PPM)] #this is for drawing on the new surface we create below
																	  #0,0 is top left corner, so to draw a circle on the top
																	  #left corner, we set center as radius,radius

			s = pygame.Surface((50,50))
			s.set_alpha(100)
			s.fill((255,255,255))
			s.set_colorkey((255,255,255))	#comment this to see how screen blit works

			pygame.draw.circle(s, (38, 192, 90), center_s, int(circle.radius*PPM),0)	#draw a circle on the new screen we created

			#pygame.draw.circle(screen, (150,150,150), center, int(circle.radius*PPM),0)	#uncomment to draw on normal screen (no alpha values)

			t = body.transform
			axis = b2Mul(t.q, b2Vec2(10.0, 25.0))

			pygame.draw.aaline(s, (255,0,0), center_s, (center_s[0] -circle.radius*axis[0], center_s[1] +circle.radius*axis[1]) )

			screen.blit(s, (position[0]-int(circle.radius*PPM),position[1]-int(circle.radius*PPM)))

			#myfont = pygame.font.SysFont("impact", 10)
			#screen.blit(myfont.render("P", True, (255,255,0)), (position[0],SCREEN_HEIGHT-position[1]))

		b2CircleShape.draw=my_draw_circle


		def my_draw_polygon(polygon, body, fixture):
			y_offset = ((self.leader.worldCenter.y)*70)
			if y_offset < -300:
				y_offset = -300
			if y_offset > 300:
				y_offset = 300
			vertices=[(body.transform*v)*PPM for v in polygon.vertices]
			vertices=[(v[0]-self.leader.worldCenter.x*30+350, SCREEN_HEIGHT-v[1]+y_offset*0.5-200) for v in vertices]
			pygame.draw.polygon(screen, colors[body.type], vertices)


		polygonShape.draw=my_draw_polygon


		# while running:
		while running and self.gen <= setting.MAX_GEN:

			self.update_car_data()
			self.update_leader()
			if self.killed == self.population_size:
    
    			#適応度の出力
				self.sort_by_dist()
				bestscore = []
				bestscore.append(self.population_data[0].max_dist)

				if(self.gen != setting.MAX_GEN):
					print("{} GENERATION_MAXDIST = {} , {} GENERATION START ".format(self.gen+1, bestscore[0], self.gen+2))
				else:
					print("{} GENERATION_MAXDIST = {} , OPTIMIZATION FINISIED ".format(self.gen+1, bestscore[0]))

				# 染色体をcsvに書き込む場合
				# for i in range(setting.LEN_GENOME):
    			# 		bestscore.append(self.population_data[0].car_def.car_genome[i])

				with open('{}{}'.format(setting.curdir + '/', setting.filename),'a') as f:
					writer = csv.writer(f)#, delimiter = ',')
					writer.writerow(bestscore)

				#次世代集団の生成
				self.gen += 1
				self.next_generation()

			# Check the event queue
			for event in pygame.event.get():
				if event.type==QUIT or (event.type==KEYDOWN and event.key==K_ESCAPE):
					# The user closed the window or pressed escape
					running=False

			#screen.fill((100,25,10,100))
			screen.fill((90,23,100,100))
			#229,153,153,255


			# Draw the world
			for body in self.world.bodies:
				for fixture in body.fixtures:
					#print "drawing.."
					fixture.shape.draw(body, fixture)

			# Make Box2D simulate the physics of our world for one step.
			self.world.Step(TIME_STEP, 10, 10)

			# Flip the screen and try to keep at the target FPS

			pygame.display.flip()
			clock.tick(TARGET_FPS)

	def update_leader(self):
		sorted_data = sorted(self.population_data,key = lambda x:x.max_dist)
		for data in sorted_data:
			if not data.isDead:
				self.leader = data.chassis
	def Step(self, settings):
		super(do_stuff, self).Step(settings)

		self.update_car_data()

		if self.killed == self.population_size:
			self.next_generation()

	def start(self):
		while True:
			self.update_car_data()
			if self.killed == self.population_size:
				self.next_generation()

	def update_car_data(self):
		for index,cars in enumerate(self.population_data):
			if not cars.isDead:
				cars.set_pos_and_vel([self.population[index][0].position.x,self.population[index][0].position.y],self.population[index][0].linearVelocity.x)

				if cars.isDead:
					#id you want to keep all the cars on the screen, (only for testing) commend the bottom 5 lines
					#print("car_num:{} distance:{}".format(index, self.population_data[index].max_dist))
					car_data.print_info(cars)
					for wheel in self.population[index][1]:
						if wheel:
							self.world.DestroyBody(wheel) #remove wheels
					self.world.DestroyBody(self.population[index][0]) #remove chassis
					# self.population[index] = None
					self.killed+=1  #turn this on only after all the mate,mutate methods work
					#print("killed so far;",self.killed)


	def sort_by_dist(self):
		self.population_data = sorted(self.population_data,key = lambda x:x.max_dist, reverse=True)
		self.leader_coors = [self.population_data[0].chassis.worldCenter.x,self.population_data[0].chassis.worldCenter.y]
		self.leader = self.population_data[0].chassis

	def check_dup(self,parent_pair,mates_list):
		dup = False
		if parent_pair in mates_list or parent_pair[::-1] in mates_list:
			dup = True
		return dup

	def get_parent_index(self,mates_list):
		parent1_index = random.randint(0,self.population_size-1)
		parent2_index = random.randint(0,self.population_size-1)
		while parent2_index == parent1_index and not self.check_dup([parent1_index,parent2_index],mates_list):
			parent1_index = random.randint(0,self.population_size-1)
			parent2_index = random.randint(0,self.population_size-1)
		return [parent1_index,parent2_index]


	def which_parent(self,index,last_parent,swp1,swp2):
		if index == swp1 or index == swp2:
			#print "changed parent:",abs(last_parent-1)
			#return abs(last_parent-1) #if 0, 1 is returned|| if 1, 0 is returned
			if last_parent == 0:
				return 1
			else:
				return 0
		else:
			return last_parent

	#def mutate(self,child):

	def mutation(self,pop):
		mut_rate = 0.03
		for m_b in range(len(pop)):
			flag = random.random()
			if flag > mut_rate:
				pop[m_b] = random.random()
		return pop

	def mutation_1(self,pop):
		# 突然変異_1:195
		list=pop
		for j in list:
			n1 = random.randint(0, 19)
			n2 = random.randint(0, 19)
			x = j[n1]
			y = j[n2]
			j[n1] = y
			j[n2] = x

		return list

	def mutation_2(self,pop):
		# 突然変異_2
		new_pop=pop
		count = 0
		for j in range(setting.LEN_GENOME * self.population_size):
			if count == 0 and random.random() >0.97:
				r1, r2 = divmod(j, 20)
				count = 1
			if count == 1 and random.random()>0.97:
				s1, s2 = divmod(j, 20)
				count = 0
				x = new_pop[r1][r2]
				y = new_pop[s1][s2]
				new_pop[r1][r2] = y
				new_pop[s1][s2] = x

		return new_pop

	def mutation_3(self,pop):
		# 突然変異_3:ランダム
		new_pop=pop
		for j in range(setting.LEN_GENOME * self.population_size):
			if random.random()>0.97:
				r1, r2 = divmod(j, 20)
				new_pop[r1][r2]=random.random()
		return new_pop

	def Random_search(self,num_c):
		pop = []
		for rs in range(num_c):
			pop.append(random.random())
		return pop

	def selTournament(self, individuals, k, tournsize):
		# 選択された個体
		selected = []

		for _ in range(k):
			# トーナメントの作成
			tournament = random.sample(individuals, tournsize)
			individuals = set(individuals) - set(tournament)

			champion = tournament[0]
			champion_score = tournament[0].max_dist
			for ind_i in range(tournsize):
				if champion_score <= tournament[ind_i].max_dist:
					champion = tournament[ind_i]
					champion_score = tournament[ind_i].max_dist

			selected.append(champion)

		return selected

	#一様交叉
	def mask(self,x1,x2):
		# 親個体x1,x2をベクトルにする
		x1 = np.array(x1)
		x2 = np.array(x2)
		#マスクを生成する
		mask=[]
		for j in range(setting.LEN_GENOME):
			mask.append(random.randint(0,1))
		#マスクに従って交叉
		for j in range(setting.LEN_GENOME):
			if mask[j]==1:
				y1=x1[j]
				y2=x2[j]
				x1[j]=y2
				x2[j]=y1

		# ベクトルからリストに直して渡す
		return x1.tolist(), x2.tolist()

	#UNDX実装
	def undx(self,x1,x2,x3):
		#親個体x1,x2,x3をベクトルにする
		x1=np.array(x1)
		x2=np.array(x2)
		x3=np.array(x3)
		#長さDを求める
		a=np.dot((x3-x1).T, (x2-x1))
		b=np.linalg.norm(x3-x1) * np.linalg.norm(x2-x1)
		D=np.sqrt(pow(np.linalg.norm(x3-x1),2) * (1-pow(a/b,2)))
		#genera:t生成（ベクトル）
		t=np.array(np.random.normal(loc=0,scale=D*0.35/np.sqrt(setting.LEN_GENOME),size=setting.LEN_GENOME))

		#引き算
		e0 = (x2-x1)/np.linalg.norm(x2-x1)
		t = t - (np.dot(t,e0) * e0)

		#足し算
		xi = np.random.normal(loc=0,scale=0.5,size=1)
		t = t + (xi[0] * (x2-x1))

		#2つの子個体生成する
		n1 = (x1 + x2)/2 + t
		n2 = (x1 + x2)/2 - t

		for j in range(len(n1)):
			if n1[j]>1:
				n1[j]=1
			if n2[j]>1:
				n2[j]=1
			if n1[j]<0:
				n1[j]=0
			if n2[j]<0:
				n2[j]=0
		# ベクトルからリストに直して渡す
		return n1.tolist(),n2.tolist()

	def next_generation(self):

		#個体集団の評価値を出力（確認用）
		# pop_fit = []
		# for i in range(self.population_size):
		# 	pop_fit.append(self.population_data[i].max_dist)
		# print(pop_fit)

		# 一般的なGAの処理
		num_elite = 2		# エリート個体の数
		num_pop = self.population_size
		num_chrom = setting.LEN_GENOME
		new_pop = []

		# エリート保存
		elite1 = self.population_data[0].car_def.car_genome
		#print("elite", self.population_data[0].max_dist)

		elite2 = self.population_data[1].car_def.car_genome

		new_pop.append(elite1)
		new_pop.append(elite2)

		# 子個体の作成
		# トーナメントまたはルーレット選択(エリートも選択肢に入る)　復元抽出、非復元抽出　MGG(家族の中で最良を１つ残す)
		# 複製選択（トーナメント選択）
		selected = self.selTournament(self.population_data, k=3, tournsize=3)
		parent1 = selected[0].car_def.car_genome
		parent2 = selected[1].car_def.car_genome
		parent3 = selected[2].car_def.car_genome

		# 子個体の作成
		#UNDX呼び出す
		for j in range(int((num_pop - num_elite)/2)):
			#child = self.Random_search(num_chrom)
			#child1,child2 = self.undx(parent1,parent2,parent3)
			child1, child2 = self.undx(self.population_data[j].car_def.car_genome, self.population_data[j+1].car_def.car_genome, self.population_data[j+2].car_def.car_genome)
			#child1,child2 =self.mask(self.population_data[j].car_def.car_genome,self.population_data[j+1].car_def.car_genome)
			new_pop.append(child1)
			new_pop.append(child2)
		print("child:",len(new_pop))
		new_population = []
		new_population_data = []

		#突然変異
		new_pop=self.mutation_2(new_pop)

		# GAの一般系からboxcar2D形式のデータへ変換
		for indi in range(num_pop):
			child_car = car(self.world,random = False,gene = new_pop[indi])
			new_population.append([child_car.get_car_chassis(),child_car.get_car_wheels()])
			new_population_data.append(car_data(child_car.get_car_chassis(), child_car.get_car_wheels(),child_car.car_def))

		self.killed = 0
		for index,elem in enumerate(new_population_data):
			self.population_data[index] = elem

		for index,elem in enumerate(new_population):
			self.population[index] = elem


	def create_generation_1(self):
		for i in range(self.population_size):
			temp  = car(self.world)
			self.population.append([temp.get_car_chassis(),temp.get_car_wheels()])
			self.population_data.append(car_data(temp.get_car_chassis(),temp.get_car_wheels(),temp.car_def))
