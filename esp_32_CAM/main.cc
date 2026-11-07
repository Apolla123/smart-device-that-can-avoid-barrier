#include <Arduino.h>

// ESP32-CAM: НЕ использовать GPIO 6–11 (флеш), 1 и 3 (UART), 16 (PSRAM)
const int L_IN1 = 14, L_IN2 = 15;  // левая сторона
const int R_IN1 = 13, R_IN2 = 12;  // правая сторона

// скорость от -255 (назад) до 255 (вперёд)
void drive(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  analogWrite(L_IN1, left  > 0 ?  left  : 0);
  analogWrite(L_IN2, left  < 0 ? -left  : 0);
  analogWrite(R_IN1, right > 0 ?  right : 0);
  analogWrite(R_IN2, right < 0 ? -right : 0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  drive(0, 0);

  Serial.println("ESP32-CAM ready");
}

void loop() {
  Serial.println("Moving forward");  drive(255, 255);   delay(2000);
  Serial.println("Turning left");    drive(180, 255);   delay(2000);
  Serial.println("Turning right");   drive(255, 180);   delay(2000);
  Serial.println("Moving backward"); drive(-255, -255); delay(2000);
  Serial.println("Stopped");         drive(0, 0);       delay(2000);
}