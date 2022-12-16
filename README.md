# CREDITOS

- Códigos em python: [Basic-Augmented-reality-course-opencv](https://github.com/Asadullah-Dal17/Basic-Augmented-reality-course-opencv) muito obrigado!

# Como instalar opencv na raspberry pi3

1. Seguir o tutorial de instalação (cerca de 8 horas) [aqui](https://linuxize.com/post/how-to-install-opencv-on-raspberry-pi/)

2. Linkar o modulo [aqui](https://pyimagesearch.com/2019/09/16/install-opencv-4-on-raspberry-pi-4-and-raspbian-buster/), vá na categoria _Sym-link your OpenCV 4 on the Raspberry Pi_

```bash
cd /usr/local/lib/python3.9/site-packages/cv2/python-3.9
```

```bash
sudo mv cv2.cpython-39-arm-linux-gnueabihf.so cv2.so
```

copiando o link `cv2.so` para o diretorio que você estiver

```bash
ln -s /usr/local/lib/python3.9/site-packages/cv2/python-3.9/cv2.so cv2.so
```

# Simulando com o [dronekit-sitl]

Abrindo uma simulação com o sitl

```bash
dronekit-sitl copter
```

Abrindo o visualizador Está usando o [mavproxy](https://github.com/ArduPilot/MAVProxy)

```bash
sim_vehicle.py -v ArduCopter --mcast --map --osd --console
```

ou

```bash
sim_vehicle.py -v ArduCopter --udp --map --osd --console
```

Após isso você pode executar o teu código

Executando [autoPositionHold.py](https://github.com/ZRafaF/OpencvPosHold/blob/master/autoPositionHold.py)

```bash
python autoPositionHold.py -s -r -d
```

```-s``` 
significa que conectará para **simulação**.

```-r``` 
deve **gravar captura de video**.

```-d``` 
mostrar o **display**.

Para ajuda 
```bash
python autoPositionHold.py --help
```

<details>
<summary>Obsoleto</summary>
<br>
Conectar no SITL

```python
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
```

> Colocar o valor da porta que foi aberta no lugar do "5760"
</details>


___

# Referências

<details>
<summary><a href="https://github.com/ArduPilot/pymavlink">PyMavlink</a></summary>

Esta é a bibliotéca em python para se comunicar com a controladora através do protocolo mavlink.

Links uteis:

- [Documentação](https://github.com/ArduPilot/pymavlink)
- [Guia mavlink](https://dronekit-python.readthedocs.io/en/latest/guide/index.html)
- [Comandos categorizados](https://ardupilot.org/dev/docs/mavlink-commands.html)
- [Lista completa de mensagens](https://mavlink.io/en/messages/common.html)
- Exemplos:
  - [Playlist com alguns exemplos](https://www.youtube.com/playlist?list=PLy9nLDKxDN68cwdt5EznyAul6R8mUSNou)
  - [Exemplos bem documentados de funções básicas](https://www.ardusub.com/developers/pymavlink.html)
- [Descrição mensagem de controle do drone](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#movement-command-details)
</details>
    
<details>
<summary>April tag</summary>

* Original
  * [Github do projeto antigo (ultima atualização em 2018)](https://github.com/swatbotics/apriltag)
    * [Bindings em python](https://github.com/swatbotics/apriltag/blob/master/python/apriltag.py)
  * [Referencia sobre as familias](https://www.ssontech.com/docs/SynthEyesUM_files/Choosing_an_AprilTag.html)
  * [Exemplo de implementação em python](https://pyimagesearch.com/2020/11/02/apriltag-with-python/)
  ___
* Pupil Labs
  * [Github do projeto](https://github.com/pupil-labs/apriltags)
    * [Bindings](https://github.com/pupil-labs/apriltags/blob/main/src/pupil_apriltags/bindings.py)
  * [Documentação da API](https://pupil-apriltags.readthedocs.io/en/stable/index.html)
  * [Exemplo do stack overflow de estimação de pose](https://stackoverflow.com/questions/59044973/how-do-i-draw-a-line-indicating-the-orientation-of-an-apriltag)

</details>

[Multithreading na raspicam](https://pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/)
