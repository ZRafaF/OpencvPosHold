from pymavlink import mavutil

import sys
import time

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# Armando
#the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
#                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

#msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
#print(msg)
###



# Decolando
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
###


# Setando o mode

# Choose a mode
mode = 'GUIDED'

# Check if mode is available
if mode not in the_connection.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(the_connection.mode_mapping().keys()))
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
    mode_id)



#the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
#                                     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 5, 0, 0, 0, 0, 0)

time.sleep(3)


type_mask = int(0b110111111000) #mascara para usar apenas a posição
while 1:
    # Descrição dessa mensagem em https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-command-details
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                       the_connection.target_component, 
                       mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, type_mask, 10 , 0, -10, 1, 0, 0, 0, 0, 0, 0, 0))



    msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
    print(msg)



    msg = the_connection.recv_match(
        type='LOCAL_POSITION_NED', blocking=True)
    print(msg)