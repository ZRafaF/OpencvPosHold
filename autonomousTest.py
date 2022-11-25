from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import socket
import math

def connectMyCopter():
	baud_rate = 57600
	connection_string = "/dev/ttyAMA0"
	print(connection_string)

	vehicle = connect(connection_string, baud=baud_rate,wait_ready=True)
	return vehicle

def arm():
	while vehicle.is_armable==True:
		print("Aguardando veiculo poder ser armado")
		time.sleep(1)
	print("Armando...")
	vehicle.armed=True
	while vehicle.armed==False:
		print("Aguardando o veiculo ser armado")
		time.sleep(1)

	print("VEICULO ARMADO!!")
	return None

def send_distance_message( dist):
	msg = vehicle.message_factory.distance_sensor_encode(
		0,  # time since system boot, not used
		1,  # min distance cm
		10000,  # max distance cm
		dist,   # current distance, must be int
		0,  # type = laser?
		0,  # onboard id, not used
		mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
		0   # covariance, not used
	)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def sendPos():
	msg = vehicle.message_factory.landing_target_encode(
		time_usec,  # time target data was processed, as close to sensor capture as possible
		target_num,	# target num, not used
		mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
		x_rad,  	# X-axis angular offset, in radians
		y_rad,  	# Y-axis angular offset, in radians
		dist_m,  	# distance, in meters
		0,			# Target x-axis size, in radians
		0,  		# Target y-axis size, in radians
		x_m,  		# x	float	X Position of the landing target on MAV_FRAME
		y_m,  		# y	float	Y Position of the landing target on MAV_FRAME
		z_m,  		# z	float	Z Position of the landing target on MAV_FRAME
		(1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
		2,  		# type of landing target: 2 = Fiducial marker
		1,  		# position_valid boolean
	)

def set_velocity_body(vehicle, vx, vy, vz):
	""" Remember: vz is positive downward!!!
	http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

	Bitmask to indicate which dimensions should be ignored by the vehicle 
	(a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
	none of the setpoint dimensions should be ignored). Mapping: 
	bit 1: x,  bit 2: y,  bit 3: z, 
	bit 4: vx, bit 5: vy, bit 6: vz, 
	bit 7: ax, bit 8: ay, bit 9:
	"""
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_NED,
		0b0000111111000111, #-- BITMASK -> Consider only the velocities
		0, 0, 0,		#-- POSITION
		vx, vy, vz, 	#-- VELOCITY
		0, 0, 0,		#-- ACCELERATIONS
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()


vehicle = connectMyCopter()

#arm()

while True:
	if not vehicle.armed:
		time.sleep(1)
	print(vehicle.channels['7'])
	if(vehicle.channels['7'] > 1500):
		set_velocity_body (vehicle, 5, 0, 0) #2 m/s para frente 
		print("Autopilot is running")




print("Fim do programa")
