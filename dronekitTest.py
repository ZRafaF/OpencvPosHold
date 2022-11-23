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

def arm():
	while vehicle.is_armable==True:
		print("Aguardando veiculo poder ser armado")
		time.sleep(1)
	print("Armando...")
	vehicle.armed = True
	while vehicle.armed==False:
		print("Aguardando o veiculo ser armado")
		time.sleep(1)

	print("VEICULO ARMADO!!")
	
	return None

vehicle = connectMyCopter()
arm()

time.sleep(5)

vehicle.armed = False

print("Fim do programa")
