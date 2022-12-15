import cv2
import apriltag

import numpy as np
from argparse import ArgumentParser

import time

cap = cv2.VideoCapture(0)
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

ehSimulacao = parser.parse_args().simulation
recordCamera = parser.parse_args().record
have_display = parser.parse_args().display

print(f"ehSimulacao: {ehSimulacao}, recordCamera: {recordCamera}, have_display: {have_display}")


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


while True:
    ret, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    currentTime= time.time()
    fps = 1.0 / (currentTime - lastTime)
    lastTime = currentTime


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
    


    print(f"tags: {format(len(results))}  fps: {fps}")
    
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
