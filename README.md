# CREDITOS

* Códigos em python: [Basic-Augmented-reality-course-opencv](https://github.com/Asadullah-Dal17/Basic-Augmented-reality-course-opencv) muito obrigado!


# Como instalar opencv na raspberry pi3

1. Seguir o tutorial de instalação (cerca de 8 horas) [aqui](https://linuxize.com/post/how-to-install-opencv-on-raspberry-pi/)



2. Linkar o modulo [aqui](https://pyimagesearch.com/2019/09/16/install-opencv-4-on-raspberry-pi-4-and-raspbian-buster/), vá na categoria *Sym-link your OpenCV 4 on the Raspberry Pi*


> ```bash
> cd /usr/local/lib/python3.9/site-packages/cv2/python-3.9
> ```

> ```bash
> sudo mv cv2.cpython-39-arm-linux-gnueabihf.so cv2.so
> ```

copiando o link ```cv2.so``` para o diretorio que você estiver

>```bash
>ln -s /usr/local/lib/python3.9/site-packages/cv2/python-3.9/cv2.so cv2.so
> ```


# Simulando com o [dronekit-sitl]

Abrindo uma simulação com o sitl
```bash
dronekit-sitl copter
```

Abrindo o visualizador (Está usando o [mavproxy](https://github.com/ArduPilot/MAVProxy))
```bash
sim_vehicle.py -v ArduCopter --mcast --map --osd --console
```

Após isso você pode executar o teu código

## **Conectar no SITL**
```python
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
```
>Colocar o valor da porta que foi aberta no lugar do "5760"

