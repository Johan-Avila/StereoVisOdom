# StereoVisOdom

Este repositorio consiste en el intento de desarrollo de un paquete de ROS que permita reconocer, configurar y estimar la Odometría Visual Estereoscópica de un sistema de cámaras estéreo de forma "Offline" en Python y de forma "Online" en C++, para la base de datos de KITTI y de forma "Online" para un sistema de cámaras estéreo propio.

##### MENU

- [Instalación](#instalacion)
- [Configuración](#configuracion)
- [Implementación](#implementacion)

<a name="instalacion"/>

## 1. Instalación

- opencv:

```console
$ pip install opencv-python
$ pip install opencv-contrib-python
```

- numpy:

```console
$ pip install numpy
```

- yaml:

```console
$ pip install pyyaml
```

- progressbar:

```console
$ pip install progressbar2
```

Clone este repositorio en ~/catkin_ws/src

```console
$ git clone https://github.com/JohanAvilaU17/StereoVisOdom.git
$ cd ..
$ catkin_make
```

<a name="configuracion"/>

## 2. Configuración

- KITTI

  - "Offline" : Modifique en el archivo "pwd.yaml", la ubicación de las imágenes en
    blanco y negro de las secuencias de KITTI respectando la siguiente estructura YAML:

    ```console
    pwd:
        dataset: "/home/johanp/Dataset/KITTI/sequences/"
    ```

    Esta ubicación debe contener la siguiente estructura

    ```console
    - sequences
        - 0
            - image_0
            - image_1
        - 1
            - image_0
            - image_1
        - 2
            - image_0
            - image_1
    ```

- Sistema de cámaras estéreo

  - "Online" : Modifique en el archivo "params.yaml", los tópicos de las imágenes
    derecha e izquierda o img0 e img1, los cuales deben contener el mensaje estándar de ROS para imágenes, como sus dimensiones y matrices de proyección mp0 y mp1 de cada una de las cámaras del sistema respectando la siguiente estructura YAML:

    ```console
    stereo_vis_odom:
        topics:
            img0: "stereo_dfm27uro135ml/cam0/image_raw"
            img1: "stereo_dfm27uro135ml/cam1/image_raw"

        orb:
            nfeatures: 1500
            scaleFactor: 1.2
            nlevels: 1
            edgeThreshold: 31
            firstLevel: 0
            WTA_K: 4
            scoreType: 0
            patchSize: 31
            fastThreshold: 7

        bf:
            normType: 7
            crossCheck: True

        filters:
            StereoGeometry:
            max_u: 100.0
            min_u: 3.0
            max_v: 3.0

        solvePnPRansac:
            useExtrinsicGuess: False
            iterationsCount: 100
            reprojectionError: 2.0
            confidence: 0.99
            flags: 2

        stereoCalibration:
            size:
                Width: 1024
                Height: 768

            matrixProjection:
                mp0:
                    [504.59056, 0.,505.99393,0.,
                    0.,504.59056,440.19245,0.,
                    0.,0.,1.,0.,]
                mp1:
                    [504.59056, 0.,505.99393,-100.02933,
                    0.,504.59056,440.19245,0.,
                    0.,0.,1.,0.,]
    ```

<a name="implementacion"/>

## 3. Implementación

- KITTI

  - "Offline" : La estimación de las rutas calculadas son guardadas en un archivo txt, a
    través de una matriz de transformación de 4x4 de forma lineal para cada una de sus poses, según la secuencia suministrada por consola.

    ```console
    $ rosrun stereo_vis_odom KittiOffLine.py
    ```

  - "Online" : Publicación de la Odometría Visual Estereoscópica en ROS con el tópico:

    - '/stereo_vis_odom/odom'

    ```console
    $ roslaunch stereo_dfm27uro135ml KITTI.launch
    ```

- Sistema de cámaras estéreo

  - "Online" : Publicación de la Odometría Visual Estereoscópica en ROS con el tópico:

    - '/stereo_vis_odom/odom'

    ```console
    $ roslaunch stereo_dfm27uro135ml StereoSystem.launch
    ```
