#!/usr/bin/env python

import socket
import sys
import math
import rospy
import csv
import drone_pb2 as drone

# Get inspired by : https://github.com/germain-hug/Autonomous-RC-Car/blob/master/scripts/manual_driver.py


class Controller(object):
	""" Controller Node
	Generates motor control and sends them to the hovercraft
	"""

	def __init__(self):

		# Set up ground socket
		self.ground = 	ground = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.ground_address = ground_address = ('0.0.0.0', 3333)
		print >> sys.stderr, 'starting up on %s port %s' % ground_address
		self.ground.bind(ground_address)

		# Try to connect to hovercraft
		print("Connecting to hovercraft ...")
		_ , self.remote_address = ground.recvfrom(1024)
		
		if (self.remote_address != 0):
			print("Connected successfully")
		else:
			print("Could not connect to hovercraft")

		# Start up rospy node
		rospy.init_node('Controller')
		rospy.on_shutdown(self.stop)	# Stop motors when shutdown

		self.rate = rospy.Rate(10)	# 10 Hz

		self.motors = drone.Motors()

		self.motors.motor_DL = 2000
		self.motors.motor_DR = 2000
		self.motors.motor_L = 1000
		self.motors.motor_R = 1000

		self._armed = False
		self._thrust = 0.6
		self._lift = 1.0

		self._yawrate_ref = 0

		self.ud_prev = 0
		self.y_prev = 0


	def sendToMotors(self):
		self.ground.sendto(self.motors.SerializeToString(), self.remote_address)


	def controlLoop(self):
		#rospy.Timer(rospy.Duration(2), controll_callback)
		# Open and read prbs file
		prbs = []
		with open('/home/prabhat/hovercraft_ws/src/ground/src/prbs','r') as f:
			read = csv.reader(f)
			for row in read:
				prbs.append(row)\

		prbs = iter(prbs)

		# Currently input.csv is written to hovercraft_ws
		with open('input.csv', 'w') as f:
			writer = csv.writer(f)		# Ugly way of logging data -> better use separate node

			while not rospy.is_shutdown():
				self.motors.motor_R = int(next(prbs) [0])
				self.motors.motor_L = int(next(prbs) [1])

				self.sendToMotors()
				data = [rospy.get_time(), self.motors.motor_R, self.motors.motor_L]
				#print(rospy.get_time())
				writer.writerow(data)
				self.rate.sleep()
		

	def stop(self):
		print("\nShut down and stop motors")
		self.motors.motor_DL = 1000
		self.motors.motor_DR = 1000
		self.motors.motor_L = 1000
		self.motors.motor_R = 1000
		self.sendToMotors()


if __name__ == "__main__":
	controller = Controller()
	controller.controlLoop()
