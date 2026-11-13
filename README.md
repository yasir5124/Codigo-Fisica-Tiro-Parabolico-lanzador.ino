# Cañón Robótico de Lanzamiento Controlado

Código para Arduino Uno de un cañón de resorte con elevación y base giratoria controladas por servomotores. Un sensor ultrasónico mide la distancia a un objeto y el programa calcula, con el modelo físico del movimiento parabólico, el ángulo de elevación necesario para que el proyectil caiga sobre él.

Este proyecto forma parte del Taller de Investigación: *Modelación y validación experimental del movimiento parabólico mediante un mecanismo robótico de lanzamiento controlado* (Universidad Evangélica de El Salvador).

> **Nota:** el tensado del resorte y la liberación de la varilla se hacen **a mano**. El programa posiciona la base y la elevación, pero **no dispara**.

---

## ¿Cómo funciona?

El mecanismo usa 2 servomotores y un sensor ultrasónico:

| Componente | Función |
| :--- | :--- |
| **Servo de elevación** | Inclina el cañón mediante engranes 2:1 (20 y 40 dientes): el tubo gira la mitad que el servo. |
| **Servo de base** | Gira la plataforma de 0° a 180° (azimut). |
| **HC-SR04** | Mide la distancia hacia el objetivo. |

---

## Comandos

Se envían desde el Monitor Serie, a **9600 baudios**, con el formato `comando,valor`:

| Comando | Qué hace |
| :--- | :--- |
| `1,ang` | Fija la elevación del tubo, de 0° a 80° (el programa envía al servo el doble del ángulo). |
| `2,ang` | Gira la base a `ang` grados (0° a 180°). |
| `3,0` | **Barrido automático:** la base gira 0° → 180° → 0° en pasos de 1° midiendo con el sensor. Se queda con el primer objeto detectado (entre 15 y 80 cm), mide su ancho, apunta al centro, calcula el ángulo y posiciona la elevación. Si el objetivo no es alcanzable, avisa y no mueve el lanzador. Para buscar de nuevo se envía `3,0`. |
| `4,0` | Mide la distancia (mediana de 5 lecturas) y muestra el ángulo de tiro correspondiente, sin mover servos. |
| `5,cm` | **Calibración:** después de fijar una elevación con `1,ang` y disparar, se envía la distancia del primer impacto (en cm, medida desde el eje de giro). El programa recalcula `K_VEL`, el alcance máximo teórico y la velocidad equivalente. El cambio dura hasta reiniciar la placa. |
| `0,0` | Cancela el barrido en curso. |

---

## Modelo físico

El proyectil no sale del eje de giro sino de la boca del cañón, a una distancia `LONG_BRAZO` (10 cm) del pivote, así que la posición de salida cambia con el ángulo:

```text
x_boca = L · cos(θ)
h_boca = h_eje + L · sin(θ)
```

* **Constante de velocidad:** `K_VEL = v₀² / g` (cm, con v₀ en cm/s y g = 981 cm/s²). Con `V0 = 3.82 m/s` vale ≈ 148.8 cm. No debe confundirse con `K_RESORTE`, la constante del resorte.
* **Cálculo del ángulo:** se despeja directamente de la ecuación de la trayectoria para que el proyectil caiga al piso a la distancia medida. Hay dos soluciones (tiro bajo y tiro alto) y el programa usa el **tiro alto**. Como la boca cambia de posición con el ángulo, el cálculo se repite 4 veces corrigiendo la distancia y la altura de salida.
* **Fuera de alcance:** el objetivo se descarta si está a menos de 15 cm, a más del alcance máximo teórico (≈ 172 cm), o si el ángulo supera `ANG_MAX` (80°).
* **Velocidad inicial:** por conservación de la energía del resorte, `v₀ = sqrt(k · x² / m)`. En el programa `V0` es una constante medida en las pruebas, y `K_RESORTE` y `MASA_KG` solo se usan para mostrar la compresión equivalente (≈ 3.19 cm).

---

## Materiales

* Arduino Uno
* 2 servomotores (elevación y base; el de la base es un MG995 de 180°)
* Sensor ultrasónico HC-SR04
* Fuente de alimentación externa para los servos
* Resorte de compresión (k = 588.2 N/m), varilla, estructura mecánica y cañón (tubo de PVC)
* Cables jumper

---

## Conexiones

| Elemento | Pin Arduino |
| :--- | :---: |
| Servo de elevación | **9** |
| Servo de base | **10** |
| HC-SR04 TRIG | **7** |
| HC-SR04 ECHO | **6** |

> [!IMPORTANT]
> Los servos deben alimentarse con una fuente externa, no con el pin de 5 V del Arduino. Une el GND de esa fuente con el GND del Arduino. El HC-SR04 trabaja a 5 V y se conecta directo, sin divisor de voltaje.

---

## Instalación y uso

1. Descarga e instala el Arduino IDE.
2. Clona este repositorio o descarga el archivo `lanzador.ino`.
3. Asegúrate de que el archivo esté dentro de una carpeta con su mismo nombre (`lanzador/lanzador.ino`).
4. Ábrelo en el IDE, selecciona la placa Arduino Uno y el puerto COM asignado.
5. Sube el código a la placa. No requiere librerías externas: usa `Servo.h`, incluida en el IDE.
6. Abre el Monitor Serie a **9600 baudios**.
7. Envía un comando, por ejemplo `3,0` para iniciar el barrido.
8. Cuando el programa muestre `OBJETIVO FIJADO`, tensa el resorte y suelta la varilla a mano.

---

## Calibración (antes de usar)

Estos parámetros, al inicio del código, dependen del montaje real:

| Parámetro | Valor | Descripción |
| :--- | :---: | :--- |
| `V0` | 3.82 m/s | Velocidad inicial, medida en Tracker. Reemplázala si cambias el resorte o la compresión. |
| `ALTURA_EJE` | 24.7 cm | Altura del eje de giro sobre el piso. |
| `LONG_BRAZO` | 10 cm | Distancia del eje de giro a la boca del cañón. |
| `K_RESORTE` | 588.2 N/m | Constante del resorte (medida por equilibrio estático). |
| `MASA_KG` | 0.041 kg | Masa que acelera el resorte (varilla y canica). |
| `ANG_MAX` | 80° | Ángulo máximo de tiro; el límite mecánico sería 90°. |
| `DETECT_MIN` / `DETECT_MAX` | 15 / 80 cm | Rango de detección. En las pruebas el sensor no detectó objetos más lejos. |
| `OFFSET_SENSOR` | 0 cm | Se suma a la distancia medida. |
| `OFFSET_BASE` | 0° | Corrección si el eje del lanzador no es paralelo al del sensor. |
| `PASO_MS`, `PASO_BUSQ` | 70 ms, 1° | Velocidad y paso del barrido. |
| `VERBOSE` | `true` | `false` reduce el texto por serial y acelera el barrido. |

Si el proyectil no cae donde se espera, usa el comando `5,cm` para recalcular `K_VEL` a partir de un disparo real y luego actualiza `V0` en el código.

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

* Mantén a todas las personas alejadas de la zona de trayectoria y del vector de lanzamiento.
* Usa proyectiles ligeros (el proyecto usó una canica de 5 g) y protección para los ojos.
* Desconecta la alimentación eléctrica antes de manipular el resorte o la varilla.

---

## Autores

* **yasir5124, giselle2713, junkun123**
