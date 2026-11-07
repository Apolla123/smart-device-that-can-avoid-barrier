#include <Arduino.h>
#include <Servo.h>
#define CALIBRATION_MODE 0
#define SENSOR_TEST_MODE 0

// ---------- Пины ----------
const uint8_t TRIG_PIN  = 12;
const uint8_t ECHO_PIN  = 13;
const uint8_t SERVO_PIN = 9;

const uint8_t PWM1_PIN = 5;
const uint8_t PWM2_PIN = 6;
const uint8_t SHCP_PIN = 2;
const uint8_t EN_PIN   = 7;
const uint8_t DATA_PIN = 8;
const uint8_t STCP_PIN = 4;

// ---------- Коды направлений (74HCT595) ----------
const uint8_t DIR_STOP       = 0;
const uint8_t DIR_FORWARD    = 92;
const uint8_t DIR_BACKWARD   = 163;
// Коды взяты из примеров производителя (1_Auto_move.ino, 4_Obstacle_Avoidance.ino).
const uint8_t DIR_TURN_LEFT   = 172;  // поворот на месте против часовой (Contrarotate)
const uint8_t DIR_TURN_RIGHT  = 83;   // поворот на месте по часовой (Clockwise)
const uint8_t DIR_STRAFE_LEFT  = 149; // сдвиг боком влево (меканум-колёса)
const uint8_t DIR_STRAFE_RIGHT = 106; // сдвиг боком вправо

// ---------- Настройки движения ----------
const uint8_t  DRIVE_SPEED = 255;    // 0..255 (максимум; у производителя 180-250)
const uint8_t  TURN_SPEED  = 255;
const float    STOP_CM     = 25.0;   // хотим стоять не ближе этого расстояния
const float    BRAKE_CM    = 0.0;    // запас на инерцию: если проскакивает ближе 25 см — поставьте 3..7
const float    NO_ECHO_CM  = 999.0;  // «ничего не вернулось» = свободно
const float    MIN_FREE_CM = 35.0;   // направление годится для проезда, если там свободно не меньше
const float    CAR_WIDTH_CM     = 25.0;
const float    SAFETY_MARGIN_CM = 3.0;   // запас с каждой стороны
const float    FAR_CAP_CM  = 150.0;  // дальше этого расстояния «одинаково хорошо»
const float    TURN_PENALTY = 0.2;   // штраф за каждый градус поворота (малые повороты выгоднее)
const uint16_t BACK_MS     = 700;    // сколько отъезжать назад из тупика
const uint8_t  MAX_BACK_TRIES = 2;   // сколько раз отъехать назад, прежде чем развернуться на 180°

// ---------- Настройки поворота ----------
const uint8_t  SERVO_FRONT = 90;
const uint8_t  SERVO_LEFT  = 180;    // 90° влево от центра (по совету куратора). Если «лево» и «право»
const uint8_t  SERVO_RIGHT = 0;      // перепутаны — поменяйте местами. Если серво жужжит в упор — 175 и 5
const uint16_t SERVO_SETTLE_MS = 450; // время, чтобы серво доехало в крайнее положение
const uint16_t SERVO_STEP_MS   = 200; // пауза между соседними точками осмотра
const uint8_t  SCAN_STEP_DEG   = 15;  // шаг осмотра 15° = угол луча датчика, значит «дыр» между замерами нет (веер 180°)
const uint8_t  SCAN_POINTS     = 180 / SCAN_STEP_DEG + 1;

Servo sensorServo;

// ---------- Моторы ----------
void motor(uint8_t dir, uint8_t speed) {
  digitalWrite(EN_PIN, LOW);           // выходы шилда включены
  analogWrite(PWM1_PIN, speed);
  analogWrite(PWM2_PIN, speed);

  digitalWrite(STCP_PIN, LOW);
  shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, dir);
  digitalWrite(STCP_PIN, HIGH);
}

bool movingForward = false;
void stopMotors()    { motor(DIR_STOP, 0); movingForward = false; }
void driveForward() {
  if (!movingForward) {      // «пинок» только при старте с места, чтобы не терять время в цикле
    motor(DIR_FORWARD, 255);
    delay(80);
    movingForward = true;
  }
  motor(DIR_FORWARD, DRIVE_SPEED);
}
void driveBackward() { motor(DIR_BACKWARD, DRIVE_SPEED); }

// Поворот на месте на заданный угол (по времени).
void turnLeft(uint16_t deg) {
  motor(DIR_TURN_LEFT, TURN_SPEED);
  delay((uint32_t)deg * TURN_MS_PER_DEG);
  stopMotors();
}

void turnRight(uint16_t deg) {
  motor(DIR_TURN_RIGHT, TURN_SPEED);
  delay((uint32_t)deg * TURN_MS_PER_DEG);
  stopMotors();
}

// ---------- Дальномер ----------
float measureOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long t = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (t == 0) return NO_ECHO_CM;
  return t / 58.0;   // мкс / 58 = см
}

// Медиана из трёх измерений — отсекает случайные выбросы.
float getDistanceCm() {
  float a = measureOnce(); delay(10);
  float b = measureOnce(); delay(10);
  float c = measureOnce();

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// Повернуть датчик на угол и измерить расстояние в этом направлении.
float lookAt(uint8_t angle) {
  sensorServo.write(angle);
  delay(SERVO_SETTLE_MS);
  return getDistanceCm();
}

// ---------- Объезд ----------
uint8_t scanBestAngle(float &bestDist) {
  float dist[SCAN_POINTS];

  sensorServo.write(0);
  delay(SERVO_SETTLE_MS);

  for (uint8_t i = 0; i < SCAN_POINTS; i++) {
    uint8_t a = i * SCAN_STEP_DEG;
    if (i != 0) { sensorServo.write(a); delay(SERVO_STEP_MS); }
    float d = getDistanceCm();
    if (d > FAR_CAP_CM) d = FAR_CAP_CM;          // «нет эха» и очень далеко — одинаково свободно
    dist[i] = d;

    Serial.print(F("  угол "));
    Serial.print(a);
    Serial.print(F(": "));
    if (d >= FAR_CAP_CM) Serial.println(F("свободно"));
    else { Serial.print(d, 1); Serial.println(F(" см")); }
  }

  float halfAngle = atan2(CAR_WIDTH_CM / 2.0 + SAFETY_MARGIN_CM, MIN_FREE_CM) * 180.0 / 3.14159265;
  int8_t n = (int8_t)ceil(halfAngle / SCAN_STEP_DEG);

  uint8_t bestAngle = SERVO_FRONT;
  float bestScore = -10000.0;
  bestDist = 0;

  for (int8_t i = 0; i < (int8_t)SCAN_POINTS; i++) {
    float safe = dist[i];
    for (int8_t j = i - n; j <= i + n; j++) {
      if (j < 0 || j >= (int8_t)SCAN_POINTS) continue;
      if (dist[j] < safe) safe = dist[j];         // любая преграда внутри коридора = коридор уже
    }
    uint8_t a = i * SCAN_STEP_DEG;
    float turnDeg = (a > SERVO_FRONT) ? (a - SERVO_FRONT) : (SERVO_FRONT - a);
    float score = safe - TURN_PENALTY * turnDeg;
    if (score > bestScore) { bestScore = score; bestAngle = a; bestDist = safe; }
  }

  Serial.print(F("  коридор: +-"));
  Serial.print(n * SCAN_STEP_DEG);
  Serial.print(F(" град, лучший угол "));
  Serial.print(bestAngle);
  Serial.print(F(", свободно "));
  Serial.print(bestDist, 1);
  Serial.println(F(" см"));

  sensorServo.write(SERVO_FRONT);
  delay(SERVO_SETTLE_MS);
  return bestAngle;
}

// Повернуть машинку в сторону, куда смотрел датчик при заданном угле серво.
void turnToward(uint8_t servoAngle) {
  int16_t delta = (int16_t)servoAngle - (int16_t)SERVO_FRONT;
  uint16_t deg = (delta < 0) ? -delta : delta;
  if (deg < 10) {
    Serial.println(F("  -> путь прямо свободен, едем"));
    return;
  }
  bool left = (SERVO_LEFT > SERVO_FRONT) ? (delta > 0) : (delta < 0);
  Serial.print(left ? F("  -> ПОВОРОТ ВЛЕВО на ") : F("  -> ПОВОРОТ ВПРАВО на "));
  Serial.println(deg);
  if (left) turnLeft(deg); else turnRight(deg);
}

// Вызывается, когда машинка уже стоит перед препятствием. Всегда находит выход.
void avoidObstacle() {
  stopMotors();

  for (uint8_t attempt = 0; ; attempt++) {
    Serial.println(F("осмотр:"));
    float dist;
    uint8_t angle = scanBestAngle(dist);

    if (dist >= MIN_FREE_CM) {
      turnToward(angle);
      delay(150);               // дать машинке успокоиться
      return;
    }

    // Тупик: везде теснее MIN_FREE_CM.
    if (attempt < MAX_BACK_TRIES) {
      Serial.println(F("  -> ТУПИК, отъезжаю назад"));
      driveBackward();
      delay(BACK_MS);
      stopMotors();
      delay(150);
    } else {
      Serial.println(F("  -> ТУПИК, РАЗВОРОТ на 180"));
      turnRight(180);
      delay(150);
      return;
    }
  }
}

// ---------- Проверка кодов моторов ----------
#if CALIBRATION_MODE
void runCalibration() {
  Serial.println(F("КАЛИБРОВКА: вперёд 1 с"));
  driveForward();  delay(1000); stopMotors(); delay(1000);
  Serial.println(F("КАЛИБРОВКА: назад 1 с"));
  driveBackward(); delay(1000); stopMotors(); delay(1000);
  Serial.println(F("КАЛИБРОВКА: поворот ВЛЕВО на 90 градусов"));
  turnLeft(90);    delay(1500);
  Serial.println(F("КАЛИБРОВКА: поворот ВПРАВО на 90 градусов"));
  turnRight(90);   delay(1500);
  Serial.println(F("КАЛИБРОВКА: сдвиг боком ВЛЕВО 1 с"));
  motor(DIR_STRAFE_LEFT, DRIVE_SPEED);  delay(1000); stopMotors(); delay(1000);
  Serial.println(F("КАЛИБРОВКА: сдвиг боком ВПРАВО 1 с"));
  motor(DIR_STRAFE_RIGHT, DRIVE_SPEED); delay(1000); stopMotors(); delay(1000);
  Serial.println(F("КАЛИБРОВКА: серво влево"));
  sensorServo.write(SERVO_LEFT);  delay(1200);
  Serial.println(F("КАЛИБРОВКА: серво вправо"));
  sensorServo.write(SERVO_RIGHT); delay(1200);
  sensorServo.write(SERVO_FRONT);
  Serial.println(F("КАЛИБРОВКА окончена. Пауза 5 с и повтор."));
  delay(5000);
}
#endif

void setup() {
  // Пока регистр не очищен, выходы отключены и колёса не дёрнутся при старте.
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  pinMode(SHCP_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STCP_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMotors();
  digitalWrite(EN_PIN, LOW);

  sensorServo.attach(SERVO_PIN, 700, 2400);
  sensorServo.write(SERVO_FRONT);

  Serial.begin(115200);
  Serial.println(F("ZY0076: объезд препятствий"));
  delay(2000);   // время поставить машинку на пол / убрать руки
}

void loop() {
#if SENSOR_TEST_MODE
  stopMotors();
  float t = measureOnce();
  Serial.print(F("ТЕСТ дальномера: "));
  if (t >= NO_ECHO_CM) Serial.println(F("нет эха"));
  else { Serial.print(t, 1); Serial.println(F(" см")); }
  delay(200);
#elif CALIBRATION_MODE
  runCalibration();
#else
  float d = measureOnce();

  if (d <= STOP_CM + BRAKE_CM) {
    stopMotors();                                   // сначала тормозим
    Serial.print(F("distance: "));
    Serial.print(d, 1);
    Serial.println(F(" cm  -> СТОП"));
    avoidObstacle();                                // осмотр и поворот (в тупике — назад/разворот)
    return;
  }

  driveForward();
  Serial.print(F("distance: "));
  if (d >= NO_ECHO_CM) Serial.print(F("нет эха"));
  else                 Serial.print(d, 1);
  Serial.println(F(" cm  -> ЕДЕМ"));
#endif
}
