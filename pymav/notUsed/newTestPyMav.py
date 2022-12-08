from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException



import sys
import time
from argparse import ArgumentParser    

parser = ArgumentParser()

parser.add_argument("-s", type=bool,
                    help="Executar como simulador?", metavar="bool", default=False)


ehSimulacao = parser.parse_args().s

print("é simulação: ", ehSimulacao)
baud_rate = 57600

def conectarV():
    if(ehSimulacao):
        return connect('udpin:localhost:14551')
    else:
        return connect("/dev/ttyAMA0", baud=baud_rate,wait_ready=True)
        


vehicle = conectarV()
the_connection = vehicle._master


""" MASKS

    Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)

    Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)

    Use Acceleration : 0b110000111000 / 0x0C38 / 3128 (decimal)

    Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)

    Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)

    Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)

    Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)

----------------------------------------------------------------------
        DESCRIÇÃO DA MENSAGEM SET_POSITION_TARGET_LOCAL_NED

time_boot_ms 	        Sender's system time in milliseconds since boot
target_system 	        System ID of vehicle
target_component 	    Component ID of flight controller or just 0
coordinate_frame 	    Valid options are listed below
type_mask 	            Bitmask to indicate which fields should be ignored by the vehicle (see POSITION_TARGET_TYPEMASK enum)
x 	                    X Position in meters (positive is forward or North)
y 	                    Y Position in meters (positive is right or East)
z 	                    Z Position in meters (positive is down)
vx 	                    X velocity in m/s (positive is forward or North)
vy 	                    Y velocity in m/s (positive is right or East)
vz 	                    Z velocity in m/s (positive is down)
afx 	                X acceleration in m/s/s (positive is forward or North)
afy 	                Y acceleration in m/s/s (positive is right or East)
afz 	                Z acceleration in m/s/s (positive is down)
yaw 	                yaw or heading in radians (0 is forward or North)
yaw_rate 	            yaw rate in rad/s



COMO FRAME DE REFERENCIA TEM:
    MAV_FRAME_LOCAL_NED
    MAV_FRAME_LOCAL_OFFSET_NED
    MAV_FRAME_BODY_NED
    MAV_FRAME_BODY_OFFSET_NED


POR FAVOR REFERENCIAR https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-command-details

"""
type_mask = int(0b110111000111) #mascara para usar apenas a velocidade


while 1:
    print(vehicle.channels['7'])
    print(vehicle.mode.name)

    if(vehicle.mode.name != 'GUIDED'):
        continue

    print("Acionado")
    # Descrição dessa mensagem em https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-command-details

    
    
    
    # Mover para trás à 5 m/s
    the_connection.mav.send(
		mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, the_connection.target_system,
            the_connection.target_component, 
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
            type_mask, 
            0,  #X Position in meters (positive is forward or North)
            0,  #Y Position in meters (positive is right or East)
            0,  #Z Position in meters (positive is down)
            4, #X velocity in m/s (positive is forward or North)
            0,  #Y velocity in m/s (positive is right or East)
            0,  #Z velocity in m/s (positive is down)
            0,  #X acceleration in m/s/s (positive is forward or North)
            0,  #Y acceleration in m/s/s (positive is right or East)
            0,  #Z acceleration in m/s/s (positive is down)
            0,  #yaw or heading in radians (0 is forward or North)
            0)) #yaw rate in rad/s

    """
    # Comando para manter o drone no heading 0
    # No real colocar o heading na direção do marker
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,          #confirmation
        90,          # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        0,          # param 3, direction -1 ccw, 1 cw
        0,          # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used

    """    


    """
    msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
    print(msg)



    """

    # Mensagem que contem a posição atual + heading
    #msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    # Imrimindo apenas o heading
    #print(msg.hdg)
