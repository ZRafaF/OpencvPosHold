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
#have_display = bool(os.environ.get("DISPLAY", None))

CAP_WIDTH = 640
CAP_HEIGHT = 480

## comprimindo a captura
cap.set(3, CAP_WIDTH)
cap.set(4, CAP_HEIGHT)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc("M", "J", "P", "G"))



# Qual o ID do marker objetivo
targetId = 72




####    CONECTANDO VIA DRONEKIT E PYMAVLINK



parser = ArgumentParser()

parser.add_argument(
    "-s", "--simulation", help="Executar como simulador", default=False, action='store_true'
)
parser.add_argument(
    "-r", "--record",  help="Gravar camera?", default=False, action='store_true'
)
parser.add_argument(
    "-d","--display",  help="Tem display", default=False, action='store_true'
)


ehSimulacao = parser.parse_args().simulation
recordCamera = parser.parse_args().record
have_display = parser.parse_args().display

baud_rate = 57600
print(f"ehSimulacao: {ehSimulacao}, recordCamera: {recordCamera}, have_display: {have_display}")


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

lastTime = 0

def rotate_marker_corners(rvec, markersize, tvec = None):

    mhalf = markersize / 2.0
    # convert rot vector to rot matrix both do: markerworld -> cam-world
    mrv, jacobian = cv.Rodrigues(rvec)

    #in markerworld the corners are all in the xy-plane so z is zero at first
    X = mhalf * mrv[:,0] #rotate the x = mhalf
    Y = mhalf * mrv[:,1] #rotate the y = mhalf
    minusX = X * (-1)
    minusY = Y * (-1)

    # calculate 4 corners of the marker in camworld. corners are enumerated clockwise
    markercorners = []
    markercorners.append(np.add(minusX, Y)) #was upper left in markerworld
    markercorners.append(np.add(X, Y)) #was upper right in markerworld
    markercorners.append(np.add( X, minusY)) #was lower right in markerworld
    markercorners.append(np.add(minusX, minusY)) #was lower left in markerworld
    # if tvec given, move all by tvec
    if tvec is not None:
        C = tvec #center of marker in camworld
        for i, mc in enumerate(markercorners):
            markercorners[i] = np.add(C,mc) #add tvec to each corner
    print('Vec X, Y, C, dot(X,Y)', X,Y,C, np.dot(X,Y)) # just for debug
    markercorners = np.array(markercorners,dtype=np.float32) # type needed when used as input to cv2
    return markercorners, mrv

def get_xy_from_corner(corner, capture_width, capture_height):
    x = 0
    y = 0
    
    x = (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4
    y = (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]) / 4
    sideSizeInPixels = math.sqrt(((corner[1][0] - corner[0][0])**2) + ((corner[1][1] - corner[0][1])**2))
    x = x - (capture_width/2)
    y = y -(capture_height/2)
    return x,y,sideSizeInPixels



while True:
    currentTime= time.time()
    fps = 1.0 / (currentTime - lastTime)
    lastTime = currentTime

    print(f"{vehicle.mode.name}, fps: {fps}")
    ret, frame = cap.read()
    if not ret:
        break
    if have_display or recordCamera:
        cv.putText(
            frame,
            f"fps: {fps}",
            (0,20),
            cv.FONT_HERSHEY_PLAIN,
            0.8,
            (0, 255, 0),
            1,
            cv.LINE_AA,
        )
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, mark = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )

        #print(rotate_marker_corners(rVec, MARKER_SIZE, tVec))

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
            xPixelPos, yPixelPos, sideSizePixel = get_xy_from_corner(corners[i], CAP_WIDTH, CAP_HEIGHT)
            
            pixelCmProportion = MARKER_SIZE / sideSizePixel

            

            # Calculating the distance
            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )

            print(f"old x {pixelCmProportion * xPixelPos} y {pixelCmProportion * yPixelPos}")
            deltaX = tVec[i][0][0]
            #deltaX = pixelCmProportion * xPixelPos
            deltaY = tVec[i][0][1]
            #deltaY = pixelCmProportion * yPixelPos
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
                cv.putText(
                    frame,
                    f"fps: {fps}",
                    (0,20),
                    cv.FONT_HERSHEY_PLAIN,
                    0.8,
                    (0, 255, 0),
                    1,
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

