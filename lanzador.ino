/*
  ============================================================
  Catapulta robótica de lanzamiento controlado - Arduino Uno
  Taller de Investigación: Modelación y validación experimental
  del movimiento parabólico
  ============================================================

  Servos:
    1) Liberador  -> suelta el brazo para disparar
    2) Tensor     -> tensa/comprime el resorte
    3) Ángulo     -> eleva el cañón (theta)
    4) Base       -> rota la plataforma (phi, azimut)

  Sensor:
    HC-SR04 -> detecta distancia a un objeto/objetivo

  Modos:
    1 -> Barrido automático: gira la base hasta detectar un
         objeto dentro de rango y dispara hacia él.
    2 -> Coordenadas: el usuario indica X,Y (metros) y el
         mecanismo calcula el ángulo de base y de elevación
         necesarios, se posiciona y dispara.

  IMPORTANTE - Calibrar antes de usar:
    - VI: velocidad inicial real del proyectil (ver informe:
      vi = sqrt(k * xmax^2 / m), con k medido por equilibrio
      estático y xmax la compresión máxima del resorte).
    - L_CANON: largo real del tubo desde el eje de rotación
      (el pivote) hasta la boca de salida. El punto de salida
      del proyectil se mueve con el ángulo, así que el cálculo
      del ángulo de elevación ahora hace una búsqueda numérica
      en vez de una fórmula cerrada (ver calcularAnguloElevacion).
    - Los ángulos ANG_* de cada servo dependen del montaje
      mecánico real; ajústense con pruebas físicas.
    - El offset de phiServo depende de cómo quede orientado
      el cero del servo de base respecto al sistema de
      coordenadas que usen para medir X,Y en el laboratorio.

  Esta versión es para ARDUINO UNO (no ESP32). Usa la librería
  "Servo.h" estándar, que ya viene incluida con el IDE de
  Arduino — no hay que instalar nada aparte.

  Nota de hardware: el Arduino Uno trabaja a 5V (a diferencia
  del ESP32, que es de 3.3V), así que el HC-SR04 queda
  conectado directo sin necesidad de divisor de voltaje en el
  pin ECHO. Lo que sí hay que vigilar es la corriente: el pin
  de 5V del Uno no alcanza para alimentar 4 servos a la vez de
  forma confiable, sobre todo en el momento del disparo. Se
  recomienda una fuente externa de 5V para los servos, con el
  GND de esa fuente conectado también al GND del Arduino.
*/

#include <Servo.h>

// ---------------- Pines (Arduino Uno) ----------------
// Se evitan los pines 0 y 1 porque el Uno los usa para el
// Serial (comunicación con la PC); cualquier otro pin digital
// sirve para los servos, la librería Servo no necesita que
// sean pines "PWM marcados con ~".
#define PIN_SERVO_LIBERADOR    9
#define PIN_SERVO_TENSOR       8
#define PIN_SERVO_ANGULO       7
#define PIN_SERVO_BASE         6

#define PIN_TRIG                4
#define PIN_ECHO                3

// ---------------- Posiciones de servo (grados) — CALIBRAR ----------------
const int ANG_LIBERADOR_REPOSO   = 0;
const int ANG_LIBERADOR_DISPARO  = 90;

const int ANG_TENSOR_REPOSO      = 0;
const int ANG_TENSOR_TENSADO     = 120;

const int ANG_ELEVACION_MIN      = 10;
const int ANG_ELEVACION_MAX      = 80;

const int ANG_BASE_MIN           = 0;
const int ANG_BASE_MAX           = 180;

// ---------------- Constantes físicas (de la calibración) ----------------
const float G  = 9.81;     // m/s^2
float VI = 3.20;           // m/s  <-- REEMPLAZAR por el valor medido en el laboratorio
const float L_CANON = 0.30; // m, largo del tubo desde el eje de rotación hasta la boca
const float R_SENSOR = 0.15; // m, distancia del sensor ultrasónico al eje de rotación (va en la base)

// ---------------- Parámetros de detección / barrido ----------------
const float DIST_MIN_DISPARO = 0.30;   // m
const float DIST_MAX_DISPARO = 2.50;   // m, alcance máximo útil del mecanismo
const int   PASO_BARRIDO     = 2;      // grados por paso
const int   RETARDO_PASO_MS  = 60;     // ms entre pasos del barrido

Servo servoLiberador, servoTensor, servoAngulo, servoBase;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  servoLiberador.attach(PIN_SERVO_LIBERADOR);
  servoTensor.attach(PIN_SERVO_TENSOR);
  servoAngulo.attach(PIN_SERVO_ANGULO);
  servoBase.attach(PIN_SERVO_BASE);

  servoLiberador.write(ANG_LIBERADOR_REPOSO);
  servoTensor.write(ANG_TENSOR_REPOSO);
  servoAngulo.write(ANG_ELEVACION_MIN);
  servoBase.write(90);

  Serial.println(F("=== Catapulta robotica - Arduino Uno ==="));
  imprimirMenu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') modoAutoDeteccion();
    else if (c == '2') modoCoordenadas();
    else if (c == '\n' || c == '\r') { /* ignorar */ }
    else imprimirMenu();
  }
}

void imprimirMenu() {
  Serial.println();
  Serial.println(F("Selecciona un modo:"));
  Serial.println(F("  1 -> Barrido automatico + disparo al detectar objeto"));
  Serial.println(F("  2 -> Ingresar coordenadas (X,Y en metros) para apuntar"));
}

// ================= Utilidades =================

float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duracion = pulseIn(PIN_ECHO, HIGH, 30000UL); // timeout ~5 m
  if (duracion == 0) return -1;                     // sin eco

  return duracion * 0.0343f / 2.0f; // cm
}

// Alcance horizontal TOTAL medido desde el eje de rotación (el origen real),
// para un ángulo de elevación dado. Toma en cuenta que el proyectil no sale
// del origen, sino del extremo del tubo (L_CANON de distancia), cuya
// posición cambia con theta: x_lanz = L*cos(theta), y_lanz = L*sin(theta).
float alcanceTotal(float thetaGrados) {
  float thetaRad = thetaGrados * PI / 180.0f;
  float x_lanz = L_CANON * cos(thetaRad);
  float y_lanz = L_CANON * sin(thetaRad);

  float vxi = VI * cos(thetaRad);
  float vyi = VI * sin(thetaRad);

  // Tiempo de vuelo hasta regresar a la altura del eje de rotación (y=0),
  // resolviendo la cuadrática completa en vez de asumir salida al ras del piso.
  float discriminante = vyi * vyi + 2.0f * G * y_lanz;
  if (discriminante < 0) return -1.0f; // no debería ocurrir físicamente
  float t = (vyi + sqrt(discriminante)) / G;

  return x_lanz + vxi * t;
}

// Ángulo de elevación (grados) para alcanzar un rango R (m), dado VI y L_CANON.
// Ya no existe una fórmula cerrada simple (el punto de salida se mueve con
// theta), así que se prueba un barrido fino de ángulos entre
// ANG_ELEVACION_MIN y ANG_ELEVACION_MAX y se usa el que más se acerque a R.
bool calcularAnguloElevacion(float R_metros, float &thetaGrados) {
  const float PASO = 0.1f; // resolución de la búsqueda, en grados
  const float TOLERANCIA = 0.03f; // 3 cm — si no se acerca más que esto, se considera fuera de alcance

  float mejorTheta = -1.0f;
  float mejorError = 1e9f;

  for (float theta = ANG_ELEVACION_MIN; theta <= ANG_ELEVACION_MAX; theta += PASO) {
    float R = alcanceTotal(theta);
    if (R < 0) continue;
    float error = fabs(R - R_metros);
    if (error < mejorError) {
      mejorError = error;
      mejorTheta = theta;
    }
  }

  if (mejorTheta < 0 || mejorError > TOLERANCIA) return false; // fuera de alcance real del mecanismo

  thetaGrados = mejorTheta;
  return true;
}

void tensarResorte() {
  Serial.println(F("Tensando resorte..."));
  servoTensor.write(ANG_TENSOR_TENSADO);
  delay(800);
}

void cargarYDisparar() {
  Serial.println(F("Disparando..."));
  servoLiberador.write(ANG_LIBERADOR_DISPARO);
  delay(400);
  servoLiberador.write(ANG_LIBERADOR_REPOSO);
  delay(300);
  servoTensor.write(ANG_TENSOR_REPOSO);
  delay(300);
}

void moverAngulo(float grados) {
  grados = constrain(grados, ANG_ELEVACION_MIN, ANG_ELEVACION_MAX);
  servoAngulo.write(grados);
  delay(400);
}

// Movimiento rápido de la base, usado durante el barrido
void moverBase(float grados) {
  grados = constrain(grados, ANG_BASE_MIN, ANG_BASE_MAX);
  servoBase.write(grados);
}

// Movimiento suave/directo de la base, usado en el modo de coordenadas
void moverBaseDirecto(float grados) {
  grados = constrain(grados, ANG_BASE_MIN, ANG_BASE_MAX);
  int actual = servoBase.read();
  int paso = (grados > actual) ? 1 : -1;
  for (int a = actual; a != (int)grados; a += paso) {
    servoBase.write(a);
    delay(15);
  }
  servoBase.write((int)grados);
}

// ================= MODO 1: barrido + deteccion =================
void modoAutoDeteccion() {
  Serial.println(F("Modo 1: barriendo hasta detectar un objeto..."));
  bool detectado = false;

  for (int angBase = ANG_BASE_MIN; angBase <= ANG_BASE_MAX && !detectado; angBase += PASO_BARRIDO) {
    moverBase(angBase);
    delay(RETARDO_PASO_MS);

    float distCm = medirDistanciaCm();
    if (distCm < 0) continue;

    float distM = distCm / 100.0f;

    if (distM >= DIST_MIN_DISPARO && distM <= DIST_MAX_DISPARO) {
      // El sensor está a R_SENSOR del eje, no en el eje mismo, así que
      // hay que sumar ese offset para obtener la distancia real desde
      // el eje de rotación (el origen que usan todos los cálculos).
      float distDesdeEje = distM + R_SENSOR;

      Serial.print(F("Objeto detectado a "));
      Serial.print(distM, 2);
      Serial.print(F(" m del sensor ("));
      Serial.print(distDesdeEje, 2);
      Serial.print(F(" m desde el eje), base = "));
      Serial.print(angBase);
      Serial.println(F(" grados"));

      float theta;
      if (calcularAnguloElevacion(distDesdeEje, theta)) {
        Serial.print(F("Angulo de elevacion calculado: "));
        Serial.println(theta, 1);

        moverAngulo(theta);
        tensarResorte();
        cargarYDisparar();
        detectado = true;
      } else {
        Serial.println(F("Distancia fuera del alcance fisico del mecanismo, se ignora."));
      }
    }
  }

  if (!detectado) {
    Serial.println(F("No se detecto ningun objeto en el barrido completo."));
  }
  imprimirMenu();
}

// ================= MODO 2: coordenadas indicadas por el usuario =================
void modoCoordenadas() {
  Serial.println(F("Modo 2: ingresa X e Y en metros, separados por coma (ej: 1.20,0.80)"));

  while (!Serial.available()) { delay(10); }
  String entrada = Serial.readStringUntil('\n');
  entrada.trim();

  int idxComa = entrada.indexOf(',');
  if (idxComa == -1) {
    Serial.println(F("Formato invalido. Usa X,Y (ej: 1.20,0.80)"));
    imprimirMenu();
    return;
  }

  float X = entrada.substring(0, idxComa).toFloat();
  float Y = entrada.substring(idxComa + 1).toFloat();

  float R = sqrt(X * X + Y * Y);
  float phiGrados = atan2(Y, X) * 180.0f / PI;

  // AJUSTAR este offset segun la orientacion fisica del cero del
  // servo de base respecto al sistema de coordenadas del laboratorio.
  float phiServo = constrain(phiGrados + 90.0f, (float)ANG_BASE_MIN, (float)ANG_BASE_MAX);

  float theta;
  if (!calcularAnguloElevacion(R, theta)) {
    Serial.println(F("Esas coordenadas estan fuera del alcance fisico del mecanismo."));
    imprimirMenu();
    return;
  }

  Serial.print(F("R = ")); Serial.print(R, 2); Serial.println(F(" m"));
  Serial.print(F("Base -> ")); Serial.print(phiServo, 1); Serial.println(F(" grados"));
  Serial.print(F("Elevacion -> ")); Serial.print(theta, 1); Serial.println(F(" grados"));

  moverBaseDirecto(phiServo);
  moverAngulo(theta);
  tensarResorte();
  cargarYDisparar();

  imprimirMenu();
}
