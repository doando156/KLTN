#include <math.h>

// Cấu hình chân động cơ
const int LEFT_MOTOR_IN1  = 6;  // PWM bánh trái
const int LEFT_MOTOR_IN2  = 7;  // Chiều bánh trái
const int RIGHT_MOTOR_IN1 = 5;  // PWM bánh phải
const int RIGHT_MOTOR_IN2 = 4;  // Chiều bánh phải

// Tốc độ tối đa
const int MAX_PWM = 255;

// Biến lưu tốc độ
float WL = 0, WR = 0;

void setup() {
  Serial2.begin(115200);  // Nhận từ ESP32 Receiver
  Serial.begin(115200);   // Monitor

  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  Serial.println("✅ Arduino Mega sẵn sàng nhận tốc độ từ ESP32!");
}

void motor(float left, float right) {
  // Động cơ trái
  if (left > 0) {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    analogWrite(LEFT_MOTOR_IN2, constrain(left, 0, MAX_PWM));
  } else {
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_IN1, constrain(-left, 0, MAX_PWM));
  }

  // Động cơ phải
  if (right > 0) {
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    analogWrite(RIGHT_MOTOR_IN2, constrain(right, 0, MAX_PWM));
  } else {
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_IN1, constrain(-right, 0, MAX_PWM));
  }
}

void loop() {
  static String receivedData = "";
  
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == ';') {
      // Xử lý gói tin sau khi đã nhận đủ
      Serial.println("📥 Nhận được gói: " + receivedData + ";");

      int commaPos = receivedData.indexOf(',');
      if (commaPos > 0) {
        String wlStr = receivedData.substring(0, commaPos);
        String wrStr = receivedData.substring(commaPos + 1);

        WL = wlStr.toFloat();
        WR = wrStr.toFloat();

        if (isValidData(WL) && isValidData(WR)) {
          Serial.println("🚀 Điều khiển động cơ:");
          Serial.print("  - Bánh trái: "); Serial.println(WL);
          Serial.print("  - Bánh phải: "); Serial.println(WR);
          motor(WL, WR);
        } else {
          Serial.println("⛔ Giá trị PWM không hợp lệ!");
        }
      } else {
        Serial.println("⛔ Dữ liệu không đúng định dạng!");
      }

      receivedData = "";  // Reset buffer
    }
    else if (c >= 32 && c <= 126) {
      receivedData += c;
    }
  }

  delay(5);  // Nhẹ CPU
}

bool isValidData(float value) {
  return !isnan(value) && !isinf(value);
}