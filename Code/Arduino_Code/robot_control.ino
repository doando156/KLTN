#include <math.h>

// Cấu hình ID cho robot này
const int MY_ID = 9;  // <--- THAY ĐỔI ID TƯƠNG ỨNG VỚI TỪNG ROBOT

// Cấu hình chân động cơ
const int LEFT_MOTOR_IN1 = 6;  // Chân PWM cho động cơ trái
const int LEFT_MOTOR_IN2 = 7;  // Chân điều khiển chiều động cơ trái
const int RIGHT_MOTOR_IN1 = 5; // Chân PWM cho động cơ phải
const int RIGHT_MOTOR_IN2 = 4; // Chân điều khiển chiều động cơ phải

// Cấu hình tốc độ tối đa
const int MAX_PWM = 255;  // Giá trị PWM tối đa (0-255)

// Biến lưu tốc độ bánh xe
float WL = 0, WR = 0;

void setup() {
  Serial2.begin(9600);  // Giao tiếp với HC-05
  Serial.begin(9600);   // Debug qua Serial Monitor

  // Cấu hình chân động cơ
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  Serial.println("🚀 Arduino Mega đã sẵn sàng!");
}

// Hàm điều khiển động cơ
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
  char incomingChar;
  
  if (Serial2.available()) {
    incomingChar = Serial2.read();
    receivedData += incomingChar;

    if (incomingChar == ';') {
      Serial.println("📥 Dữ liệu nhận: " + receivedData);

      int firstComma = receivedData.indexOf(',');
      int secondComma = receivedData.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > firstComma) {
        String idStr = receivedData.substring(0, firstComma);
        String wlStr = receivedData.substring(firstComma + 1, secondComma);
        String wrStr = receivedData.substring(secondComma + 1, receivedData.length() - 1); // Bỏ dấu ';'

        int received_id = idStr.toInt();
        WL = wlStr.toFloat();
        WR = wrStr.toFloat();

        Serial.print("🔍 ID nhận được: ");
        Serial.println(received_id);

        if (received_id == MY_ID) {
          Serial.println("✅ ID khớp - Điều khiển động cơ");
          Serial.print("🚀 Tốc độ bánh trái (WL): ");
          Serial.println(WL);
          Serial.print("🚀 Tốc độ bánh phải (WR): ");
          Serial.println(WR);
          motor(WL, WR);
        } else {
          Serial.println("⛔ ID không khớp - Bỏ qua");
        }
      }

      receivedData = "";
    }
  }
}
