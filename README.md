Catapulta robótica de lanzamiento controlado
Código para Arduino Uno de una catapulta robótica que lanza un proyectil hacia un objetivo, usando el modelo del movimiento parabólico.

Proyecto del Taller de Investigación: Modelación y validación experimental del movimiento parabólico.

¿Cómo funciona?
El mecanismo usa 4 servos y un sensor ultrasónico:

Componente	Función
Servo liberador	Suelta el brazo para disparar
Servo tensor	Tensa/comprime el resorte
Servo de ángulo	Eleva el cañón (ángulo θ)
Servo de base	Rota la plataforma (ángulo φ, azimut)
HC-SR04	Mide la distancia al objetivo
Modos de operación
Se eligen desde el Monitor Serie (115200 baudios):

Modo 1 – Barrido automático: la base gira de 0° a 180° hasta detectar un objeto dentro del rango. Calcula el ángulo de elevación necesario, se posiciona y dispara.
Modo 2 – Coordenadas: el usuario ingresa X,Y en metros (por ejemplo 1.20,0.80). El programa calcula el ángulo de base y de elevación, se posiciona y dispara.
Modelo físico
El proyectil no sale del eje de rotación, sino de la boca del cañón, a una distancia L_CANON del pivote. Por eso el punto de salida cambia con el ángulo:

x_lanz = L · cos(θ)
y_lanz = L · sin(θ)
El alcance total se calcula resolviendo la cuadrática completa del tiempo de vuelo (sin suponer salida al ras del piso), y el ángulo de elevación se obtiene con una búsqueda numérica entre ANG_ELEVACION_MIN y ANG_ELEVACION_MAX (paso de 0.1°), eligiendo el ángulo cuyo alcance se acerque más al objetivo (tolerancia de 3 cm).

La velocidad inicial se obtiene de la energía del resorte:

vi = sqrt(k · xmax² / m)
donde k se mide por equilibrio estático, xmax es la compresión máxima del resorte y m la masa del proyectil.

Materiales
Arduino Uno
4 servomotores
Sensor ultrasónico HC-SR04
Fuente externa de 5 V para los servos
Resorte, estructura y cañón (tubo)
Cables jumper
Conexiones
Elemento	Pin Arduino
Servo liberador	9
Servo tensor	8
Servo de ángulo	7
Servo de base	6
HC-SR04 TRIG	4
HC-SR04 ECHO	3
Importante: el pin de 5 V del Arduino Uno no alimenta 4 servos de forma confiable, sobre todo al disparar. Usa una fuente externa de 5 V para los servos y conecta su GND al GND del Arduino.

El HC-SR04 trabaja a 5 V igual que el Uno, así que se conecta directo, sin divisor de voltaje.

Instalación y uso
Instala el IDE de Arduino.
Clona el repositorio o descarga lanzador.ino. Recuerda que el archivo debe estar dentro de una carpeta con el mismo nombre (lanzador/lanzador.ino).
Abre el archivo, selecciona la placa Arduino Uno y el puerto correspondiente.
Sube el código. No hace falta instalar librerías: usa Servo.h, que ya viene con el IDE.
Abre el Monitor Serie a 115200 baudios.
Escribe 1 o 2 para elegir el modo.
Calibración (hacer antes de usar)
Estos valores dependen del montaje real y deben ajustarse con pruebas físicas:

Parámetro	Descripción
VI	Velocidad inicial real del proyectil (m/s). El valor por defecto (3.20) es solo un ejemplo: reemplázalo por el medido en el laboratorio
L_CANON	Largo del tubo desde el pivote hasta la boca de salida (m)
R_SENSOR	Distancia del sensor al eje de rotación (m)
ANG_*	Posiciones de cada servo (liberador, tensor, elevación y base) según el montaje mecánico
phiServo (offset de +90°)	Depende de cómo quede orientado el cero del servo de base respecto al sistema de coordenadas del laboratorio
DIST_MIN_DISPARO / DIST_MAX_DISPARO	Rango de distancias en el que el mecanismo dispara
Alcance real: aunque DIST_MAX_DISPARO está en 2.5 m, el alcance verdadero lo limita VI. Si el objetivo está fuera de lo que el mecanismo puede alcanzar, el programa lo indica y no dispara.

Estructura del repositorio
.
├── lanzador/
│   └── lanzador.ino
└── README.md
Seguridad
Mantén a las personas fuera de la zona de lanzamiento.
Usa proyectiles ligeros y blandos.
Desconecta la alimentación antes de manipular el resorte o el tensor.
Autor
