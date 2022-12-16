from __future__ import print_function

import apriltag
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException


import numpy as np
from argparse import ArgumentParser

from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2



#cap = cv2.VideoCapture(0)
CAP_WIDTH = 640
CAP_HEIGHT = 480

MARKER_SIZE = 15  # centimeters

## comprimindo a captura
#cap.set(3, CAP_WIDTH)
#cap.set(4, CAP_HEIGHT)


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
parser.add_argument(
    "-c","--cap",  help="Usar funcao cap.read do opencv ao inves da pycam", default=False, action='store_true'
)

ehSimulacao = parser.parse_args().simulation
recordCamera = parser.parse_args().record
have_display = parser.parse_args().display
useCap = parser.parse_args().cap

print(f"ehSimulacao: {ehSimulacao}, recordCamera: {recordCamera}, have_display: {have_display}, useCap: {useCap}")

def conectarV():
    if ehSimulacao:
        return connect("udpin:localhost:14551")
    else:
        return connect("/dev/ttyAMA0", baud=baud_rate, wait_ready=True)
        
print("Aguardando conexao")
vehicle = conectarV()
the_connection = vehicle._master
print("Conectado!")

# Parametros gerados do script aprilCalibrateCam.py
cam_params = (630.8669379442165, 630.3123204518172, 335.75042566981904, 227.83332282734318)



options = apriltag.DetectorOptions(
                families="tag16h5",
                refine_edges=True,
                refine_decode=True,
                refine_pose=True,
                quad_contours=True)


"""
options = apriltag.Detectoroptions(families='tag36h11',
                                 border=1,
                                 nthreads=4,
                                 quad_decimate=1.0,
                                 quad_blur=0.0,
                                 refine_edges=True,
                                 refine_decode=False,
                                 refine_pose=False,
                                 debug=False,
                                 quad_contours=True)
"""


detector = apriltag.Detector(options)

baud_rate = 57600


lastTime = 0

def _draw_pose(overlay, camera_params, tag_size, pose, z_sign=1):

    opoints = np.array([
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,
         1, -1, -2*z_sign,
         1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
    ]).reshape(-1, 1, 3) * 0.5*tag_size

    edges = np.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2)
        
    fx, fy, cx, cy = camera_params

    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    tvec = pose[:3, 3]

    dcoeffs = np.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = np.round(ipoints).astype(int)
    
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


if not useCap:
    camera = PiCamera()
    camera.resolution = (CAP_WIDTH, CAP_HEIGHT)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(CAP_WIDTH, CAP_HEIGHT))
    stream = camera.capture_continuous(
        rawCapture, 
        format="bgr",
        use_video_port=True)

def getVs():
    if useCap:
        cap = cv2.VideoCapture(0)
        CAP_WIDTH = 640
        CAP_HEIGHT = 480

        ## comprimindo a captura
        cap.set(3, CAP_WIDTH)
        cap.set(4, CAP_HEIGHT)
        return cap
    ret = PiVideoStream().start()
    time.sleep(1.0)
    return ret

vs = getVs()


while True:
    
    frame = vs.read()
    frame = imutils.resize(frame, width=CAP_WIDTH)

    
    #ret, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    currentTime= time.time()
    fps = 1.0 / (currentTime - lastTime)
    lastTime = currentTime

    print(f"fps: {fps}")

    cv2.putText(
            frame,
            f"fps: {fps}",
            (0,20),
            cv2.FONT_HERSHEY_PLAIN,
            0.8,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    
    results = detector.detect(gray_frame)
    


    print("[INFO] {} total AprilTags detected".format(len(results)))
    
    """
    tag.id,
    tag.hamming,
    tag.goodness,
    tag.decision_margin,
    """


    
    for r in results:
        id = r.tag_id
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(
            frame, 
            tagFamily, 
            (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_PLAIN, 
            1.5, 
            (0, 255, 0), 
            2)
        
        cv2.putText(
            frame,                  # frame
            f"id: {id}",            # Texto
            ptD,                    # Posição
            cv2.FONT_HERSHEY_PLAIN, # Fonte
            1.2,                      # Tamanho
            (0, 0, 255),            # Cor
            2,                      # Grossura
            cv2.LINE_AA
        )
        pose, e0, e1 = detector.detection_pose(
            r,
            cam_params,
            MARKER_SIZE)
        if have_display:
            _draw_pose(
                frame,
                cam_params,
                MARKER_SIZE,
                pose)

    if have_display:
        # show the output image after AprilTag detection
        cv2.imshow("Image", frame)    
        
        key = cv2.waitKey(1)
        if key == ord("q"):
            break