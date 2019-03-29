import socket
import sys
import rospy
import drone_pb2 as drone

# Get inspired by : https://github.com/germain-hug/Autonomous-RC-Car/blob/master/scripts/manual_driver.py

def setup():
	ground = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

	ground_address = ('0.0.0.0', 3333)
	print >> sys.stderr, 'starting up on %s port %s' % ground_address
	ground.bind(ground_address)

	while True:
		print >>sys.stderr, '\nwaiting to receive message'
		_ , hover_address = ground.recvfrom(4096)
		
		if (hover_address != 0):
			print >>sys.stderr, 'received answer from %s' % hover_address
			return ground, hover_address
    
  

thrust = 1
lift = 0

motors = drone.Motors()

motors.motor_DL = (int)(1000 + 1000 * lift)
motors.motor_DR = motors.motor_DL

motors.motor_L = (int)(1000 + thrust * 500)
motors.motor_R = (int)(1000 + thrust * 500)


def controll_callback(event):
    print 'Timer called at ' + str(event.current_real)
	data = motors.SerializeToString()
	sent = ground.sendto(data, hover_address)

def controller():
	rospy.Timer(rospy.Duration(2), controll_callback)


if __name__ == "__main__":
	setup()
	contoller()




























'''

def control_loop(self, state, prbs):
		self.motors.motor_DL = (int)(1000 + 1000 * self.lift)
		self.motors.motor_DR = self.motors.motor_DL

		# Compute PD controller
		y = state.gyro_z / 1000
		ud = 0.606531 * self.ud_prev + 11.000000 * y + -10.606531 * self.y_prev
		self.ud_prev = ud
		self.y_prev = y

		Kp = 1/2
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

'''

