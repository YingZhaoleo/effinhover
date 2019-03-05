from aioudp import * 
import asyncio
from datetime import datetime
import threading
import click
import csv
import time

import drone_pb2 as drone

import pygame


class Periodic_Delay:
	def __init__(self, period, show_period=None, name=None):
		self.period = period
		self.prevtime = datetime.now()
		self.prevshow = datetime.now()
		self.show_period = show_period
		self.name = name
		self.count = 0

	async def sleep(self):
		now = datetime.now()
		delta = now - self.prevtime
		self.prevtime = now
		await asyncio.sleep((self.period - delta.total_seconds())/1000.0)

		self.count = self.count + 1
		if self.show_period is not None:
			delta = now - self.prevshow
			if delta.total_seconds() > self.show_period:
				frequency = (float)(self.count) / (float)(delta.total_seconds())
				print(f"Frequency of {self.name} is {frequency:.1f} (target {1/self.period*1000.0:.1f})\r")
				self.count = 0
				self.prevshow = now


async def setup():
	# Create a linkState UDP enpoint
	linkState = await open_local_endpoint('0.0.0.0', 3333)

	# The linkState endpoint receives the datagram, along with the address
	print("=> Waiting for first message from hover")
	data, address = await linkState.receive()

	# Create a UDP enpoint from the sending address
	print("=> Received message - beginning")
	linkCommand = await open_remote_endpoint(address[0], address[1])

	return linkState, linkCommand



class Controller(object):
	def __init__(self, linkCommand):
		self.linkCommand = linkCommand
		self.motors = drone.Motors()

		self._armed = False
		self._thrust = 0.6
		self._lift = 1.0

		self._yawrate_ref = 0

		self.ud_prev = 0
		self.y_prev = 0

	def stop(self):
		self.armed = False

	@property
	def yawrate_ref(self):
		return self._yawrate_ref/5

	@yawrate_ref.setter
	def yawrate_ref(self, val):
		# Incoming signal is between -1 and 1
		val = min(max(val,-1),1)
		# Scale to a good yawrate target
		self._yawrate_ref = 5*val

	@property
	def thrust(self):
		return self._thrust

	@thrust.setter
	def thrust(self, val):
		self._thrust = min(max(val, 0), 1)

	@property
	def lift(self):
		return self._lift

	@lift.setter
	def lift(self, val):
		self._lift = min(max(val, 0), 1)

	@property
	def armed(self):
		return self._armed
	
	@armed.setter
	def armed(self, val):
		if val == True:
			self._armed = True
		else:
			self._armed = False
			self._off()
			self._send_motors()

	async def run(self, estimator, period):
		periodic = Periodic_Delay(period)

		f = open('prbs.csv','r')
		read = csv.reader(f)
		prbs = []
		for row in read:
			print(row)	# Just to test what is stored in row
			prbs.append(row[0])
		f.close()
		prbs = iter(prbs)

		print(prbs)
		print(next(prbs))
		print(next(prbs))
		print(next(prbs))
		print(next(prbs))

		while True:
			# Sets the motor values
			#self.control_loop(estimator.state, int(next(prbs)))	# This generated errors so I replaced it with line below
			self.control_loop(estimator.state, [1000,1000,1000,1000])	# Looks like the prbs signal is never applied in method control_loop

			if self.armed is False:
				self._off()
			self._send_motors()

			await periodic.sleep()



	def control_loop(self, state, prbs):
		self.motors.motor_DL = (int)(1000 + 1000 * self.lift)
		self.motors.motor_DR = self.motors.motor_DL

		# Compute PD controller
		y = state.gyro_z / 1000
		ud = 0.606531 * self.ud_prev + 11.000000 * y + -10.606531 * self.y_prev
		self.ud_prev = ud
		self.y_prev = y

		Kp = 1/5
		u = Kp * (self._yawrate_ref - ud)

		# print('u = %f\r' % u)
		u = max(min(u, 1), -1)

		thrust = self.thrust

		self.motors.motor_L = (int)(1000 + thrust * 500)
		self.motors.motor_R = (int)(1000 + thrust * 500)

		if u > 0:
			self.motors.motor_L = self.motors.motor_L + (int)(500 * u)
		else:
			self.motors.motor_R = self.motors.motor_L - (int)(500 * u)

	def _send_motors(self):
		self.linkCommand.send(self.motors.SerializeToString())

	def _off(self):
		self.motors.motor_DL = 1000
		self.motors.motor_DR = 1000
		self.motors.motor_L  = 1000
		self.motors.motor_R  = 1000

	def __str__(self):
		s = '[x] ' if self.armed else '[ ] '
		s = s + "Thrust %.1f Lift %.1f " % (self.thrust, self.lift)
		s = s + "YawRef %.1f " % self.yawrate_ref
		return s






class Estimator(object):
	def __init__(self):
		self.state = drone.DroneState()
		self._start_time = datetime.now()

	async def run(self, linkState):
		prevtime = datetime.now()
		count = 0
		while True:
			count = count + 1
			if count > 1000:
				delta = datetime.now() - prevtime
				frequency = 1.0 / delta.total_seconds() * 1000.0
				print ("Estimator loop frequency = %.2f\r" % frequency)
				count = 0
				prevtime = datetime.now()

			data, address = await linkState.receive()
			self.state.ParseFromString(data)




class Logger(object):
	def __init__(self, filename='hoverlog.csv'):

		self._f = open('hoverlog.csv', 'w')
		self._logger = csv.writer(self._f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

		self._logger.writerow([
			'Time', 
			'roll', 'pitch', 'yaw', 
			'gyro_x', 'gyro_y', 'gyro_z', 
			'acc_x', 'acc_y', 'acc_z', 
			'mag_x', 'mag_y', 'mag_z',
			'armed', 'thrust', 'lift', 'yawrate_ref',
			'motor_DL', 'motor_DR', 'motor_L', 'motor_R'])

		self._start_time = datetime.now()

	async def run(self, period, estimator, controller):
		periodic = Periodic_Delay(period)
		while True:
			time = datetime.now() - self._start_time

			state = estimator.state
			motors = controller.motors

			if controller.armed:
				self._logger.writerow([
					time.total_seconds(), 
					state.roll,   state.pitch,  state.yaw, 
					state.gyro_x, state.gyro_y, state.gyro_z, 
					state.acc_x,  state.acc_y,  state.acc_z, 
					state.mag_x,  state.mag_y,  state.mag_z,
					controller.armed, controller.thrust, controller.lift, controller.yawrate_ref,
					motors.motor_DL, motors.motor_DR, motors.motor_L, motors.motor_R])

			await periodic.sleep()


	async def display(self, period, estimator, controller):
		periodic = Periodic_Delay(50)
		while True:
			state = estimator.state
			print(f"[{'x' if controller.armed else ' '}] gyro/ref {state.gyro_z/1000:6.1f}/{controller.yawrate_ref:6.1f} thrust/lift {controller.thrust:.1f}/{controller.lift:.1f}", end='')
			print(f" Motors D {controller.motors.motor_DL}|{controller.motors.motor_DR} L|R {controller.motors.motor_L}|{controller.motors.motor_R}", end='\r')
			await periodic.sleep()

	def __del__(self):
		close(self._f)


class SystemThread (threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

		loop = asyncio.new_event_loop()
		asyncio.set_event_loop(loop)

		linkState, linkCommand = loop.run_until_complete(setup())

		controller = Controller(linkCommand)
		estimator = Estimator()
		logger = Logger()

		control_period = 10 # ms
		logging_period = 10 # ms
		display_period = 5000 # ms

		tasks = []
		tasks.append(loop.create_task(controller.run(estimator, control_period)))
		tasks.append(loop.create_task(estimator.run(linkState)))
		tasks.append(loop.create_task(logger.run(logging_period, estimator, controller)))
		tasks.append(loop.create_task(logger.display(display_period, estimator, controller)))

		self.loop = loop
		self.controller = controller
		self.estimator = estimator
		self.logger = logger
		self.tasks = tasks

		# userInput = UserInput(controller)
		# userInput.start()

	def run(self):
		# self.loop = asyncio.get_event_loop()
		self.loop.run_until_complete(asyncio.gather(*self.tasks))


# Thread for managing the hovercraft
systemThread = SystemThread()
controller = systemThread.controller
systemThread.start()

while True:
	key = click.getchar(False)
	if key == 'a':
		controller.armed = False if controller.armed else True

	if key == 'x':
		controller.lift = controller.lift + 0.1
	if key == 'z':
		controller.lift = controller.lift - 0.1

	if key == u'\x1b[A': # Up 
		controller.thrust = controller.thrust + 0.1
	if key == u'\x1b[B': # Down
		controller.thrust = controller.thrust - 0.1

	if key == u'\x1b[C': # Right			
		controller.yawrate_ref = controller.yawrate_ref + 0.5
	if key == u'\x1b[D': # Left
		controller.yawrate_ref = controller.yawrate_ref - 0.5
	if key == ' ': # Space
		controller.yawrate_ref = 0

	# print (repr(key))



