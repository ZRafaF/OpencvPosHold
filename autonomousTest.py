from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
vehicle = connectMyCopter()

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
    vehicle.armed = True
    while vehicle.armed==False:
		print("Aguardando o veiculo ser armado")
		time.sleep(1)
    
	print("VEICULO ARMADO!!")

	
	return None

arm()

while(vehicle.channels['7'] > 1750):
    print("Autopilot is running")



print("Fim do programa")
