from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException


import cv2 as cv
from cv2 import aruco
import numpy as np
import os
import math

import sys
import time
from argparse import ArgumentParser
import math

calib_data_path = "calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

# Tamanho da lateral do marker
MARKER_SIZE = 15  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)

param_markers = aruco.DetectorParameters_create()

cap = cv.VideoCapture(0)
have_display = bool(os.environ.get("DISPLAY", None))

if os.name == 'nt':
    have_display = True

## comprimindo a captura
cap.set(3, 400)
cap.set(4, 300)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc("M", "J", "P", "G"))



# Qual o ID do marker objetivo
targetId = 72




####    CONECTANDO VIA DRONEKIT E PYMAVLINK



parser = ArgumentParser()

parser.add_argument(
    "-s", type=bool, help="Executar como simulador?", metavar="bool", default=False
)
parser.add_argument(
    "-r", type=bool, help="Gravar camera?", metavar="bool", default=True
)


ehSimulacao = parser.parse_args().s
recordCamera = parser.parse_args().r
baud_rate = 57600

print(f"ehSimulacao: {ehSimulacao}, recordCamera: {recordCamera}")


def conectarV():
    if ehSimulacao:
        return connect("udpin:localhost:14551")
    else:
        return connect("/dev/ttyAMA0", baud=baud_rate, wait_ready=True)
        


print("Aguardando conexao")
vehicle = conectarV()
the_connection = vehicle._master
print("Conectado!")


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

def endProgramAndShutDown():
    cap.release()
    cv.destroyAllWindows()
    vehicle.close()
    print("Fim do programa")
    exit()
    os.system("sudo shutdown -h now")
    exit()


# Ganho nos eixos (menor = mais abrupto)
ganhoX = 10
ganhoY = 10

maxPitchAngle = 12
maxRollAngle = 12

def stayStill():
    if vehicle.mode.name != "GUIDED_NOGPS":
        return
    the_connection.mav.set_attitude_target_send(
        0,
        the_connection.target_system,
        the_connection.target_component,
        0b00000000,
        to_quaternion(0, 0, 0),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(0),  # Body yaw rate in radian/second
        0.5,  # Thrust
    )


wasArmed = False

def processAutoFlight(deltaX, deltaY, rotation, altitude):
    if not vehicle.armed:
        print("Nao armado")
        print(vehicle.mode.name)
        if False: #wasArmed:
            print("desligando")
            endProgramAndShutDown()
        return
    wasArmed = True

    if vehicle.mode.name != "GUIDED_NOGPS":
        return

    pitchAngle = deltaX / ganhoX
    rollAngle = deltaY / ganhoY
    

    print(f'roll = {rollAngle} pitch = {pitchAngle}')

    if pitchAngle > maxPitchAngle:
        pitchAngle = maxPitchAngle
    if pitchAngle < -maxPitchAngle:
        pitchAngle = -maxPitchAngle

    if rollAngle < -maxRollAngle:
        rollAngle = -maxRollAngle

    if rollAngle > maxRollAngle:
        rollAngle = maxRollAngle

    the_connection.mav.set_attitude_target_send(
        0,
        the_connection.target_system,
        the_connection.target_component,
        0b00000000,
        to_quaternion(-rollAngle, pitchAngle, 0),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(0),  # Body yaw rate in radian/second
        0.5,  # Thrust
    )

fourcc = cv.VideoWriter_fourcc('X','V','I','D')
videoWriter = cv.VideoWriter('video.avi', fourcc, 30.0, (400,300))

while True:
    print(vehicle.mode.name)
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, mark = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )

        # Calculando a rotação

        # resolvendo PnP muito pesado
        # _, rVecs, tVecs = cv.solvePnP(mark, marker_corners[0], cam_mat, dist_coef)

        # -X o drone deve se mover para direita
        # +X o drone deve se mover para frente
        # -Y o drone deve se mover para frente
        # +Y o drone deve se mover para trás

        # A camera está rotacionada 90° no sentido horário

        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            if ids != targetId:
                continue

            # Calculating the distance
            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )
            deltaX = tVec[i][0][0]
            deltaY = tVec[i][0][1]
            rotation = math.degrees(rVec[i][0][1]) + 180

            #if have_display:
            if have_display or recordCamera:
                cv.polylines(
                    frame,
                    [corners.astype(np.int32)],
                    True,
                    (0, 255, 255),
                    4,
                    cv.LINE_AA,
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
                # Draw the pose of the marker
                point = cv.drawFrameAxes(
                    frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4
                )
                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} r:{round(rotation)}",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )

            print(f"x = {deltaX}, y = {deltaY}, r = {rotation}")
            processAutoFlight(deltaX, deltaY, rotation, distance)
    else:
        stayStill()
    if have_display:
        cv.imshow("frame", frame)

    if recordCamera:
        videoWriter.write(frame)
    key = cv.waitKey(1)

    if key == ord("q"):
        break


endProgramAndShutDown()

