// Lector de encoder óptico, disco de 30 ranuras (60 flancos/vuelta)
// ESP32-S3, señal en GPIO 1. Muestra todo por el monitor serie (115200).

#define ENC_PIN         1
#define EDGES_PER_REV   60        // 30 ranuras x 2 flancos (subida y bajada)
#define MIN_EDGE_US     400       // ignora pulsos más juntos que esto (ruido)
#define STOP_TIMEOUT_US 200000    // sin flancos por 200 ms = detenido
#define PRINT_MS        200       // cada cuánto se imprime
#define WHEEL_RADIUS_MM 32.5f     // radio de tu rueda (cámbialo, mídelo)

volatile uint32_t edgeCount = 0;      // flancos totales
volatile uint32_t lastEdgeUs = 0;     // instante del último flanco
volatile uint32_t lastPeriodUs = 0;   // tiempo entre los dos últimos flancos
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onEdge() {
  uint32_t now = micros();
  uint32_t dt = now - lastEdgeUs;
  if (dt < MIN_EDGE_US) return;       // filtro de rebotes/ruido
  portENTER_CRITICAL_ISR(&mux);
  lastPeriodUs = dt;
  lastEdgeUs = now;
  edgeCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);   // espera al USB hasta 3 s
  pinMode(ENC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_PIN), onEdge, CHANGE);
  Serial.println("Encoder listo (GPIO 1, 60 flancos/vuelta)");
}

void loop() {
  static uint32_t prevCount = 0;
  static uint32_t prevMs = millis();

  delay(PRINT_MS);

  uint32_t count, period, last;
  portENTER_CRITICAL(&mux);
  count  = edgeCount;
  period = lastPeriodUs;
  last   = lastEdgeUs;
  portEXIT_CRITICAL(&mux);

  uint32_t nowMs = millis();
  float dtS = (nowMs - prevMs) / 1000.0f;
  prevMs = nowMs;

  // RPM promedio en la ventana (suave)
  float rpmAvg = ((count - prevCount) / (float)EDGES_PER_REV) / dtS * 60.0f;
  prevCount = count;

  // RPM instantánea por periodo entre flancos (rápida)
  float rpmInst = 0;
  if (micros() - last < STOP_TIMEOUT_US && period > 0)
    rpmInst = 1e6f / ((float)period * EDGES_PER_REV) * 60.0f;

  float revs   = count / (float)EDGES_PER_REV;
  float angle  = (count % EDGES_PER_REV) * (360.0f / EDGES_PER_REV);
  float distMm = revs * 2.0f * PI * WHEEL_RADIUS_MM;
  float velMs  = rpmInst / 60.0f * 2.0f * PI * WHEEL_RADIUS_MM / 1000.0f;

  Serial.printf("flancos=%lu  vueltas=%.3f  angulo=%.0f deg  rpm(prom)=%.1f  rpm(inst)=%.1f  dist=%.1f mm  vel=%.3f m/s\n",
                (unsigned long)count, revs, angle, rpmAvg, rpmInst, distMm, velMs);
}