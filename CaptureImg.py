## Derivado de https://github.com/Asadullah-Dal17/Basic-Augmented-reality-course-opencv
## por favor visite o criador original

import cv2
import os

print("Opencv version: ",cv2.__version__)

# Dimensão do tabuleiro
CHESS_BOARD_DIM = (9, 6)

# Número de imagens
n = 0

# Checando se o diretorio "images" já existe
image_dir_path = "images"

CHECK_DIR = os.path.isdir(image_dir_path)

if not CHECK_DIR:
    os.makedirs(image_dir_path)
    print(f'"{image_dir_path}" Directory is created')
else:
    print(f'"{image_dir_path}" Directory already Exists.')


cap = cv2.VideoCapture(0)
## comprimindo a captura
cap.set(3, 400)
cap.set(4, 300)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))


while True:
    _, frame = cap.read()

    cv2.imshow("frame", frame)

    key = cv2.waitKey(1)

    if key == ord("q"):
        break
    if key == ord("s"):
        # storing the checker board image
        cv2.imwrite(f"{image_dir_path}/image{n}.png", frame)

        print(f"imagem de numer {n} salva")
        n += 1  # incrementing the image counter
cap.release()
cv2.destroyAllWindows()

print("Total de imagens:", n)
