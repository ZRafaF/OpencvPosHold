from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException


import sys
import time
from argparse import ArgumentParser
import math

parser = ArgumentParser()

parser.add_argument(
    "-s", type=bool, help="Executar como simulador?", metavar="bool", default=False
)


ehSimulacao = parser.parse_args().s
baud_rate = 57600


def conectarV():
    if ehSimulacao:
        return connect("/dev/ttyAMA0", baud=baud_rate, wait_ready=True)
    else:
        return connect("udpin:localhost:14551")


vehicle = conectarV()


the_connection = vehicle._master


print("Aguardando conexao")

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
# the_connection.wait_heartbeat()
# print("Heartbeat from system (system %u component %u)" %
#    (the_connection.target_system, the_connection.target_component))

# Armando
# the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
#                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
# print(msg)
###


"""
# Decolando
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
###

"""

"""
# Setando o mode

# Choose a mode
mode = "GUIDED"

# Check if mode is available
if mode not in the_connection.mode_mapping():
    print("Unknown mode : {}".format(mode))
    print("Try:", list(the_connection.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = the_connection.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id,
)

"""

# the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
#                                     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 5, 0, 0, 0, 0, 0)


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
type_mask = int(0b110111000111)  # mascara para usar apenas a velocidade


wasArmed = False


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


while 1:
    # print(vehicle.mode.name)

    if not vehicle.armed:
        print("Nao armado")
        print(vehicle.mode.name)
        if wasArmed:
            print("desligando")
            break
        time.sleep(1)
        continue
    wasArmed = True
    """
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.PLAY_TUNE_V2,
        3,"d")
    """
    if vehicle.mode.name != "GUIDED_NOGPS":
        continue

    # print("Acionado")

    the_connection.mav.set_attitude_target_send(
        0,
        the_connection.target_system,
        the_connection.target_component,
        0b00000000,
        to_quaternion(0, -5, 0),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(0),  # Body yaw rate in radian/second
        0.5,  # Thrust
    )

    """
def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    
    #use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
    #              When one is used, the other is ignored by Ardupilot.
    #thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
    #        Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
    #        the code for maintaining current altitude.
    
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)
    
    https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-attitude-target
    """

    # Descrição dessa mensagem em https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-command-details

    """   msg = the_connection.recv_match(type="ATTITUDE_TARGET", blocking=False)
    print(msg)
    
    
    
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
    # msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    # Imrimindo apenas o heading   msg = the_connection.recv_match(type="ATTITUDE_TARGET", blocking=False)
    print(msg)
    # print(msg.hdg)

vehicle.close()


print("Fim do programa")
os.system("sudo shutdown -h now")
exit()
