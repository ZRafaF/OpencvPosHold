## Credito
#https://www.youtube.com/watch?v=kB9YyG2V-nA&t=1880s&ab_channel=TheDroneDojo

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse

def connectMyCopter():
	parser = argparse.ArgumentParser(description='comands')
	parser.add_argument("--connect")
	args = parser.parse_args()
	connection_string = args.connect
	baud_rate = 57600

	print(connection_string)

	vehicle = connect(connection_string, baud=baud_rate,wait_ready=True)
	return vehicle

vehicle = connectMyCopter()


while True:
	for ch in vehicle.channels:
		print("ch %s" % ch, end = ' ')
		print("ch  %s" % vehicle.channels[str(ch)], end = ' ')
	print(".")

print("Fim do programa")
