## Derivado de https://github.com/Asadullah-Dal17/Basic-Augmented-reality-course-opencv
## e https://github.com/tizianofiorenzani/how_do_drones_work
## por favor visite os criadores originais

import cv2
import os
import glob
import numpy as np

print("Opencv version: ",cv2.__version__)

# Dimensão do tabuleiro
CHESS_BOARD_DIM = (9, 6)
SQUARE_SIZE = 26 #- mm



image_dir_path = "images"
imageType       = "png"


# Checando se o diretorio "images" já existe
CHECK_DIR = os.path.isdir(image_dir_path)

if not CHECK_DIR:
    print(f'"{image_dir_path}" Diretorio não encontrado')
    print("Finalizando programa...")
    quit()

# Checando se o diretorio "calib_data" já existe
calib_data_path = "./calib_data"
CHECK_DIR_CALIB = os.path.isdir(calib_data_path)

if not CHECK_DIR_CALIB:
    os.makedirs(calib_data_path)
    print(f'"{calib_data_path}" Directory is created')

else:
    print(f'"{calib_data_path}" Directory already Exists.')


filename    = image_dir_path + "/*." + imageType
images      = glob.glob(filename)

# Número de imagens
n = len(images)

# Criteria do opencv
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, SQUARE_SIZE, 0.001)


# Criando o vetor do numpy
obj_3D = np.zeros((CHESS_BOARD_DIM[0] * CHESS_BOARD_DIM[1], 3), np.float32)
obj_3D[:, :2] = np.mgrid[0 : CHESS_BOARD_DIM[0], 0 : CHESS_BOARD_DIM[1]].T.reshape(
    -1, 2
)
obj_3D *= SQUARE_SIZE
print(obj_3D)

# Arrays to store object points and image points from all the images.
obj_points_3D = []  # 3d point in real world space
img_points_2D = []  # 2d points in image plane.


def detect_checker_board(image, grayImage, criteria, boardDimension):
    ret, corners = cv2.findChessboardCorners(grayImage, boardDimension)
    if ret == True:
        corners1 = cv2.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv2.drawChessboardCorners(image, boardDimension, corners1, ret)

    return image, ret, corners1


for imageIt in images:
    cv2.destroyAllWindows()
    frame = cv2.imread(imageIt)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image, board_detected, corners1 = detect_checker_board(frame, gray, criteria, CHESS_BOARD_DIM)

    cv2.imshow(f"{imageIt}",frame)
    key = cv2.waitKey(0)
    if(key == ord("s") and board_detected):
        obj_points_3D.append(obj_3D)
        img_points_2D.append(corners1)
        print("Imagen salva")
    else:
        print("Imagen descartada")
    




cv2.destroyAllWindows()

# calibrar
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points_3D, img_points_2D, gray.shape[::-1], None, None
)

print("calibrated")

print("duming the data into one files using numpy ")
np.savez(
    f"{calib_data_path}/MultiMatrix",
    camMatrix=mtx,
    distCoef=dist,
    rVector=rvecs,
    tVector=tvecs,
)

print("-------------------------------------------")

print("loading data stored using numpy savez function\n \n \n")

data = np.load(f"{calib_data_path}/MultiMatrix.npz")

camMatrix = data["camMatrix"]
distCof = data["distCoef"]
rVector = data["rVector"]
tVector = data["tVector"]

print("loaded calibration data successfully")
print(camMatrix)
print(distCof)
print(rVector)
print(tVector)


print("Fim do programa")