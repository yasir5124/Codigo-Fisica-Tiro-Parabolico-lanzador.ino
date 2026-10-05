# Catapulta Robotica de Lanzamiento Controlado

Codigo para Arduino Uno de una catapulta robotica que lanza un proyectil hacia un objetivo, utilizando el modelo fisico del movimiento parabolico. 

Este proyecto forma parte del Taller de Investigacion: Modelacion y validacion experimental del movimiento parabolico.

---

## Como funciona?

El mecanismo utiliza 4 servomotores y un sensor ultrasonico para automatizar el apuntado y disparo:

| Componente | Funcion |
| :--- | :--- |
| **Servo liberador** | Suelta el brazo para ejecutar el disparo. |
| **Servo tensor** | Tensa o comprime el resorte a la posicion requerida. |
| **Servo de angulo** | Eleva el cañon para ajustar el angulo de elevacion (θ). |
| **Servo de base** | Rota la plataforma para ajustar el azimut o angulo de base (φ). |
| **HC-SR04** | Mide la distancia en tiempo real hacia el objetivo. |

---

## Modos de operacion

Los modos se seleccionan directamente desde el Monitor Serie configurado a 115200 baudios:

* **Modo 1 – Barrido automatico:** La base gira de 0° a 180° hasta que el sensor detecta un objeto dentro del rango. En ese momento, calcula el angulo de elevacion necesario, posiciona los servos y dispara.
* **Modo 2 – Coordenadas:** El usuario ingresa de forma manual las coordenadas X, Y en metros (por ejemplo: `1.20,0.80`). El programa calcula automaticamente el angulo de la base y de elevación, se posiciona y dispara.

---

## Modelo fisico

El proyectil no sale directamente del eje de rotacion, sino de la boca del cañon, situada a una distancia L_CANON del pivote. Por lo tanto, el punto de salida cambia dinamicamente segun el angulo:

\[x_{lanz} = L \cdot \cos(\theta)\]
\[y_{lanz} = L \cdot \sin(\theta)\]

* **Calculo de trayectoria:** El alcance total se calcula resolviendo la ecuacion cuadratica completa del tiempo de vuelo (sin suponer salida al ras del suelo).
* **Busqueda del angulo:** El angulo de elevacion se obtiene mediante una busqueda numerica entre `ANG_ELEVACION_MIN` y `ANG_ELEVACION_MAX` (con un paso de 0.1°), eligiendo el angulo cuyo alcance estimado se acerque mas al objetivo con una tolerancia de 3 cm.
* **Velocidad inicial (v_i):** Se obtiene a partir de la energia elastica del resorte:
  \[v_i = \sqrt{\frac{k \cdot x_{max}^2}{m}}\]
  Donde k se mide por equilibrio estatico, x_max es la compresion maxima del resorte y m es la masa del proyectil.

---

## Materiales

* Arduino Uno
* 4 Servomotores
* Sensor ultrasonico HC-SR04
* Fuente de alimentacion externa de 5 V (para los servos)
* Resorte, estructura mecanica y cañon (tubo)
* Cables jumper

---

## Conexiones

| Elemento | Pin Arduino |
| :--- | :---: |
| Servo liberador | **9** |
| Servo tensor | **8** |
| Servo de angulo | **7** |
| Servo de base | **6** |
| HC-SR04 TRIG | **4** |
| HC-SR04 ECHO | **3** |

> [!IMPORTANT]
> El pin de 5 V del Arduino Uno no puede alimentar 4 servos de forma confiable, especialmente durante el pico de corriente al disparar. Usa una fuente externa de 5 V para los servos y recuerda unir su tierra (GND) con el GND del Arduino. El HC-SR04 trabaja a 5 V y se conecta directo, sin divisor de voltaje.

---

## Instalacion y uso

1. Descarga e instala el Arduino IDE.
2. Clona este repositorio o descarga el archivo `lanzador.ino`.
3. Asegurate de que el archivo este dentro de una carpeta con su mismo nombre (`lanzador/lanzador.ino`).
4. Abre el archivo en el IDE, selecciona la placa Arduino Uno y el puerto COM asignado.
5. Sube el codigo a la placa. (No requiere librerias externas; utiliza `Servo.h` integrada en el IDE).
6. Abre el Monitor Serie a 115200 baudios.
7. Escribe `1` o `2` en la linea de comandos para elegir el modo de operacion.

---

## Calibracion (Antes de usar)

Los siguientes parametros del codigo dependen del montaje mecanico real y deben ajustarse mediante pruebas fisicas en el laboratorio:

* **`VI`**: Velocidad inicial real del proyectil (m/s). El valor por defecto (`3.20`) es ilustrativo; reemplazalo por el medido experimentalmente.
* **`L_CANON`**: Largo del tubo desde el pivote de elevacion hasta la boca de salida (m).
* **`R_SENSOR`**: Distancia fisica desde el sensor ultrasonico hasta el eje de rotacion (m).
* **`ANG_*`**: Limites de posicion de cada servo (liberador, tensor, elevacion y base) segun el acoplamiento mecanico.
* **`phiServo` (offset de +90°)**: Ajuste de orientacion del grado cero de la base respecto al eje de coordenadas del laboratorio.
* **`DIST_MIN_DISPARO` / `DIST_MAX_DISPARO`**: Rango operativo de distancias de disparo. Aunque el limite maximo este configurado en `2.5 m`, el alcance real final esta acotado por la velocidad inicial (`VI`). Si el objetivo es inalcanzable, el programa lo advertira en el monitor serie y abortara el disparo.

---

## Estructura del repositorio

```text
.
├── lanzador/
│   └── lanzador.ino
└── README.md
```

---

## Seguridad

* Mantenga a todas las personas alejadas de la zona de trayectoria y del vector de lanzamiento.
* Utilice exclusivamente proyectiles ligeros y de materiales blandos (ej. espuma o goma eva).
* Desconecte siempre la alimentacion electrica antes de manipular manualmente el resorte o el mecanismo del tensor.

---

## Autor

* **yasir5124, giselle2713, junkun123**
