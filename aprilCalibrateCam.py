from __future__ import print_function
import numpy as np
import cv2
import argparse
import glob
import os

CHESS_BOARD_DIM = (9, 6)
SQUARE_SIZE = 26 #- mm
show_detections = True

calib_data_path = "./calib_data"
CHECK_DIR_CALIB = os.path.isdir(calib_data_path)

image_dir_path = "images"
imageType       = "png"

filename    = image_dir_path + "/*." + imageType
images      = glob.glob(filename)

def main():
    if not CHECK_DIR_CALIB:
        os.makedirs(calib_data_path)
        print(f'"{calib_data_path}" Directory is created')
    else:
        print(f'"{calib_data_path}" Directory already Exists.')
    cols = CHESS_BOARD_DIM[0]
    rows = CHESS_BOARD_DIM[1]
    

    if rows < cols:
        patternsize = (cols, rows)
    else:
        patternsize = (rows, cols)

    sz = SQUARE_SIZE

    x = np.arange(patternsize[0])*sz
    y = np.arange(patternsize[1])*sz

    xgrid, ygrid = np.meshgrid(x, y)
    zgrid = np.zeros_like(xgrid)
    opoints = np.dstack((xgrid, ygrid, zgrid)).reshape((-1, 1, 3)).astype(np.float32)

    imagesize = None

    win = 'Calibrate'
    cv2.namedWindow(win)

    ipoints = []

    for filename in images:

        rgb = cv2.imread(filename)
        
        if rgb is None:
            print('warning: error opening {}, skipping'.format(filename))
            continue

        cursize = (rgb.shape[1], rgb.shape[0])
        
        if imagesize is None:
            imagesize = cursize
        else:
            assert imagesize == cursize

        print('loaded ' + filename + ' of size {}x{}'.format(*imagesize))

        if len(rgb.shape) == 3:
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        else:
            gray = rgb
        
        retval, corners = cv2.findChessboardCorners(gray, patternsize)

        if show_detections:
            display = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
            cv2.drawChessboardCorners(display, patternsize, corners, retval)
            cv2.imshow(win, display)
            while cv2.waitKey(5) not in range(128): pass

        if not retval:
            print('warning: no chessboard found in {}, skipping'.format(filename))
        else:
            ipoints.append( corners )

    flags = (cv2.CALIB_ZERO_TANGENT_DIST |
             cv2.CALIB_FIX_K1 |
             cv2.CALIB_FIX_K2 |
             cv2.CALIB_FIX_K3 |
             cv2.CALIB_FIX_K4 |
             cv2.CALIB_FIX_K5 |
             cv2.CALIB_FIX_K6)

    opoints = [opoints] * len(ipoints)

    retval, K, dcoeffs, rvecs, tvecs = cv2.calibrateCamera(opoints, ipoints, imagesize,
                                                           cameraMatrix=None,
                                                           distCoeffs=np.zeros(5),
                                                           flags=flags)

    assert( np.all(dcoeffs == 0) )
    
    fx = K[0,0]
    fy = K[1,1]
    cx = K[0,2]
    cy = K[1,2]

    params = (fx, fy, cx, cy)

    print()
    print('all units below measured in pixels:')
    print('  fx = {}'.format(K[0,0]))
    print('  fy = {}'.format(K[1,1]))
    print('  cx = {}'.format(K[0,2]))
    print('  cy = {}'.format(K[1,2]))
    print()
    print('pastable into Python:')
    print('  fx, fy, cx, cy = {}'.format(repr(params)))
    print()
    np.savez(f"{calib_data_path}/CamParam", cameraParams = params)

    print("Carregando data")

    data = np.load(f"{calib_data_path}/CamParam.npz")

    params = data["cameraParams"]

    print(params)


if __name__ == '__main__':
    main()