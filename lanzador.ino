#include <Servo.h>

Servo lanzador, base;
const byte PIN_LANZADOR = 9, PIN_BASE = 10, PIN_TRIG = 7, PIN_ECHO = 6;

// ===== Calibración =====
const float V0 = 3.82;              // m/s, velocidad inicial (medida en Tracker)
const float ALTURA_EJE = 24.7;      // cm, suelo -> eje de giro (objetivo en el suelo)
const float K_RESORTE = 588.2;      // N/m, constante elástica del resorte (no confundir con K_VEL)
const float MASA_KG = 0.041;        // kg, masa lanzada
const float LONG_BRAZO = 10.0;      // cm, eje de giro (abrazadera) -> boca del cañón (donde sale la pelota)
const float OFFSET_SENSOR = 0.0;    // cm, se suma a la distancia medida
const int OFFSET_BASE = 0;          // grados fijos si el eje del lanzador no es paralelo al del sensor
const int ANG_MAX = 80;             // ángulo máximo de tiro: la elevación solo se usó de 0° a 80° (el límite mecánico sería 90°: servo 180° / relación 2:1)
const float DETECT_MIN = 15.0, DETECT_MAX = 80.0;      // el sensor solo toma de 15 a 80 cm (en las pruebas no detectó objetos más lejos)
const int BARRIDO_MIN = 0, BARRIDO_MAX = 180;          // barrido completo: 0 -> 180 -> 0 en bucle

// K_VEL = v²/g (cm), con v en cm/s y g = 981 cm/s²  ->  ≈ 148.8 cm. Es distinta de K_RESORTE (la del resorte).
float K_VEL = (V0 * 100.0) * (V0 * 100.0) / 981.0;
float alcMax, anguloLanz;

// ===== Sensor =====
const unsigned long ECO_MAX_US = 5000UL;             // ≈ 85 cm: los fallos se descartan rápido
const byte PAUSA_MS = 25;                            // pausa entre disparos (evita ecos de la lectura anterior)

// ===== Barrido =====
const unsigned long PASO_MS = 70;                    // ms por paso (más alto = barrido más lento)
const byte PASO_BUSQ = 1;                            // ° por paso mientras busca (1 = seguro, 2 = más rápido)
const byte HITS = 4, FALLOS = 4;                     // HITS: válida suma 2, fallida resta 1 -> basta con 2 lecturas válidas
const byte RETROCESO = 3;
const bool VERBOSE = true;                           // false = más rápido (menos texto por serial)

// ===== Estado =====
enum Estado { REPOSO, BUSCANDO, SIGUIENDO } estado = REPOSO;
int baseAng = 90, dir = 1, ini, fin, primerHit, hits, fallos;
unsigned long tPaso;
float dUlt;

// Mediana de varias lecturas (solo de 15 a 80 cm); exige que al menos la mitad sean válidas. -1 si falla.
float medir(byte n) {
  float l[9];
  byte v = 0;
  n = min(n, 9);
  for (byte i = 0; i < n; i++) {
    digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    unsigned long t = pulseIn(PIN_ECHO, HIGH, ECO_MAX_US);
    if (t) {
      float c = t * 0.01715 + OFFSET_SENSOR;
      if (c >= DETECT_MIN && c <= DETECT_MAX) l[v++] = c;   // descarta lo que esté fuera de 15 a 80 cm
    }
    if (i < n - 1) delay(PAUSA_MS);
  }
  if (v < (n + 1) / 2) return -1;
  for (byte i = 1; i < v; i++) {                       // ordenar (inserción)
    float x = l[i];
    int j = i - 1;
    while (j >= 0 && l[j] > x) { l[j + 1] = l[j]; j--; }
    l[j + 1] = x;
  }
  return l[v / 2];
}

// Lectura rápida para el barrido: un disparo, y si no vuelve eco, un segundo intento (objetos pequeños fallan a ratos)
float leer() {
  float d = medir(1);
  if (d < 0) { delay(PAUSA_MS); d = medir(1); }
  return d;
}

float tol(float d) { return max(10.0, d * 0.12); }
bool enRango(float d) { return d >= DETECT_MIN && d <= DETECT_MAX; }
bool mismo(float d, float f) { return d > 0 && fabs(d - dUlt) <= tol(dUlt) * f; }
float velocidad() { return sqrt(K_VEL * 981.0) / 100.0; }                 // m/s
float compresion() { return sqrt(K_VEL * 981.0 * MASA_KG / K_RESORTE); }  // cm
void calcAlcance() { alcMax = K_VEL * sqrt(1.0 + 2.0 * ALTURA_EJE / K_VEL); } // alcance teórico máximo (el límite real es ANG_MAX)

void imprimirVelocidad() {
  Serial.print(F("Velocidad inicial: ")); Serial.print(velocidad(), 2);
  Serial.print(F(" m/s | Compresion: ")); Serial.print(compresion(), 2); Serial.println(F(" cm"));
}

// ===== Física del tiro (tiro alto). -1 si el objetivo está fuera de 15-alcMax o el ángulo supera ANG_MAX =====
float calcAngulo(float d) {
  if (d < DETECT_MIN || d > alcMax) return -1;
  float ang = 0, x = d, h = ALTURA_EJE;
  for (byte i = 0; i < 4 && x >= 1; i++) {
    float A = x * x / (2.0 * K_VEL), disc = x * x - 4.0 * A * (A - h);
    if (disc < 0) disc = 0;
    ang = atan((x + sqrt(disc)) / (2.0 * A)) * RAD_TO_DEG;
    x = d - LONG_BRAZO * cos(ang * DEG_TO_RAD);
    h = ALTURA_EJE + LONG_BRAZO * sin(ang * DEG_TO_RAD);
  }
  return (x < 1 || ang > ANG_MAX) ? -1 : ang;
}

void moverLanzador(float a) {
  anguloLanz = a;
  int s = (int)round(a * 2);                           // relación 2:1
  lanzador.write(s);
  Serial.print(F("[OK] Lanzador: ")); Serial.print(a, 1);
  Serial.print(F("° (servo ")); Serial.print(s); Serial.println(F("°)"));
}

// Mueve la base llegando siempre desde el mismo lado (evita el juego del servo)
void apuntar(int a) {
  a = constrain(a, 0, 180);
  base.write(max(0, a - RETROCESO)); delay(150);
  base.write(a); delay(250);
  baseAng = a;
}

// Se queda con el PRIMER objeto detectado: va al centro de su ancho (ini..fin) y mide ahí una vez.
void confirmarObjeto() {
  int centro = (ini + fin) / 2;
  Serial.print(F(">> Objeto entre ")); Serial.print(min(ini, fin)); Serial.print(F("° y ")); Serial.print(max(ini, fin));
  Serial.print(F("°. Centro: ")); Serial.println(centro);
  apuntar(centro);
  float d = medir(5);
  if (!mismo(d, 1.5)) d = dUlt;                        // si no hay lectura válida, usa la distancia seguida en el barrido
  float ang = calcAngulo(d);
  estado = REPOSO;                                     // REPOSO para no re-detectarlo en bucle

  if (ang < 0) {
    Serial.print(F(">> Objeto a ")); Serial.print(d, 1);
    Serial.print(F(" cm (base ")); Serial.print(baseAng);
    Serial.print(F("°) requiere angulo mayor a ")); Serial.print(ANG_MAX);
    Serial.println(F("°. NO se dispara. Envia 3,0 para buscar de nuevo."));
    return;
  }

  if (OFFSET_BASE) apuntar(baseAng + OFFSET_BASE);
  Serial.println(F("====================================="));
  Serial.print(F("OBJETIVO FIJADO | Base: ")); Serial.print(baseAng);
  Serial.print(F("° | Dist: ")); Serial.print(d, 1);
  Serial.print(F(" cm | Angulo: ")); Serial.print(ang, 1); Serial.println(F("°"));
  moverLanzador(constrain(ang, 0, ANG_MAX));
  imprimirVelocidad();
  Serial.println(F("Sensor detenido. Envia 3,0 para buscar de nuevo."));
  Serial.println(F("====================================="));
}

void pasoBarrido() {
  float d = leer();
  bool ok = enRango(d);
  if (VERBOSE) {
    Serial.print(F("Base: ")); Serial.print(baseAng); Serial.print(F("° | Dist: "));
    if (d >= 0) Serial.print(d, 1); else Serial.print(F("--"));
    Serial.println(ok ? F(" cm  <- objeto") : F(" cm"));
  }
  if (estado == BUSCANDO) {
    if (ok && !hits) primerHit = baseAng;
    hits = ok ? hits + 2 : max(0, hits - 1);           // tolera lecturas perdidas
    if (ok) dUlt = d;
    if (hits >= HITS) {
      estado = SIGUIENDO; ini = primerHit; fin = baseAng; fallos = 0;
      Serial.println(F(">> Objeto detectado. Midiendo ancho..."));
    }
  }
  else if (mismo(d, 1.5)) { fin = baseAng; dUlt = d; fallos = 0; }   // sigue siendo el mismo objeto
  else if (++fallos >= FALLOS + (int)(dUlt / 25)) { confirmarObjeto(); return; }

  baseAng += dir * (estado == BUSCANDO ? PASO_BUSQ : 1);   // buscando: PASO_BUSQ grados; siguiendo: 1°
  bool lim = baseAng <= BARRIDO_MIN || baseAng >= BARRIDO_MAX;
  if (lim) { baseAng = constrain(baseAng, BARRIDO_MIN, BARRIDO_MAX); dir = -dir; }
  base.write(baseAng);
  if (lim && estado == SIGUIENDO) confirmarObjeto();
}

// ===== Comandos por serial =====
void comando(int cmd, int v) {
  bool manual = v >= 0 && v <= (cmd == 1 ? ANG_MAX : 180);   // rango válido (solo cmd 1 y 2)
  if (cmd == 0 || ((cmd == 1 || cmd == 2) && manual && estado != REPOSO)) {
    estado = REPOSO;
    Serial.println(F("[OK] Busqueda cancelada."));
  }
  if (cmd == 1 || cmd == 2) {
    if (!manual) { Serial.print(F("[ERROR] Rango: 0 a ")); Serial.println(cmd == 1 ? ANG_MAX : 180); }
    else if (cmd == 1) moverLanzador(v);
    else { baseAng = v; base.write(v); Serial.print(F("[OK] Base en: ")); Serial.print(v); Serial.println(F("°")); }
  }
  else if (cmd == 3) {
    estado = BUSCANDO; hits = fallos = 0; tPaso = 0;
    Serial.println(F("[OK] Barrido iniciado. Envia 0,0 para cancelar."));
  }
  else if (cmd == 4) {
    float d = medir(5);
    if (d < 0) { Serial.println(F("[ERROR] Sin objeto entre 15 y 80 cm.")); return; }
    float a = calcAngulo(d);
    Serial.print(F("Distancia: ")); Serial.print(d, 1); Serial.print(F(" cm | "));
    if (a >= 0) { Serial.print(F("Angulo de tiro: ")); Serial.print(a, 1); Serial.println(F("°")); }
    else Serial.println(F("Fuera de alcance de tiro."));
  }
  else if (cmd == 5) {                                 // calibrar: 1,ang -> disparar -> 5,cm (distancia del primer impacto medida desde el EJE DE GIRO)
    float a = anguloLanz * DEG_TO_RAD, R = v - LONG_BRAZO * cos(a), h = ALTURA_EJE + LONG_BRAZO * sin(a);
    if (anguloLanz < 5 || R < 10) { Serial.println(F("[ERROR] Antes usa 1,angulo (mayor a 5) y dispara.")); return; }
    K_VEL = R * R / (2.0 * cos(a) * (R * sin(a) + h * cos(a)));
    calcAlcance();
    Serial.print(F("[OK] K_VEL = ")); Serial.print(K_VEL, 1);
    Serial.print(F(" cm | Alcance teorico: ")); Serial.print(alcMax, 1); Serial.println(F(" cm"));
    imprimirVelocidad();
  }
  else Serial.println(F("[ERROR] Comando no valido. Use 0 a 5."));
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  lanzador.attach(PIN_LANZADOR);
  base.attach(PIN_BASE);
  lanzador.write(0);
  base.write(baseAng);

  calcAlcance();
  Serial.println(F("=== SISTEMA DE CONTROL DE SERVOS ==="));
  imprimirVelocidad();
  Serial.print(F("K_VEL: ")); Serial.print(K_VEL, 1);
  Serial.print(F(" cm | Deteccion: ")); Serial.print(DETECT_MIN, 0);
  Serial.print(F(" a ")); Serial.print(DETECT_MAX, 0);
  Serial.print(F(" cm | Angulo max: ")); Serial.print(ANG_MAX); Serial.println(F("°"));
  Serial.println(F("Comandos: 1,ang lanzador | 2,ang base | 3,0 buscar | 4,0 medir | 5,cm calibrar | 0,0 cancelar"));
}

void loop() {
  unsigned long ahora = millis();
  if (estado != REPOSO && ahora - tPaso >= PASO_MS) { tPaso = ahora; pasoBarrido(); }

  if (Serial.available() > 0) {
    int cmd = Serial.parseInt();
    if (Serial.read() == ',') comando(cmd, Serial.parseInt());
    while (Serial.available() > 0) Serial.read();
  }
}
