#include <ArduinoBLE.h>

// Đối với dữ liệu manufacturer, 2 byte đầu tiên phải là ID công ty
uint16_t companyID = 0xFFFF;  // ID công ty tiêu chuẩn dùng cho kiểm tra

// Bộ đệm để lưu trữ dữ liệu từ Serial
char inputBuffer[100];
int bufferPos = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Khởi tạo BLE
  if (!BLE.begin()) {
    Serial.println("❌ Khởi tạo BLE thất bại!");
    while (1);
  }

  // Thiết lập thông tin quảng cáo
  BLE.setLocalName("ESP32_Beacon");
  
  // Ban đầu gửi dữ liệu trống
  updateAdvertisement("");
  
  Serial.println("✅ Server BLE đã sẵn sàng để nhận lệnh từ Serial");
  Serial.println("📝 Định dạng: 'ID,PWM1,PWM2;' (VD: '9,100,100;')");
}

void loop() {
  // Đọc dữ liệu từ Serial khi có sẵn
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Nếu gặp ký tự kết thúc hoặc buffer đầy
    if (c == '\n' || bufferPos >= sizeof(inputBuffer) - 1) {
      // Kết thúc chuỗi
      inputBuffer[bufferPos] = '\0';
      
      // Xử lý lệnh
      if (bufferPos > 0) {
        String command = String(inputBuffer);
        Serial.print("📡 Đang phát lệnh: ");
        Serial.println(command);
        
        // Cập nhật quảng cáo BLE - chỉ chứa thông tin một robot
        updateAdvertisement(command);
      }
      
      // Reset buffer
      bufferPos = 0;
    } else {
      // Thêm ký tự vào buffer
      inputBuffer[bufferPos++] = c;
    }
  }
  
  // Cập nhật quảng cáo BLE
  BLE.poll();  // Cho phép BLE cập nhật
}

// Hàm cập nhật dữ liệu quảng cáo
void updateAdvertisement(String payload) {
  // Tạo dữ liệu manufacturer - cấp phát không gian cho ID công ty + payload
  size_t payloadLen = payload.length();
  uint8_t mData[2 + payloadLen];
  
  // Đặt ID công ty (little-endian)
  mData[0] = companyID & 0xFF;        // LSB
  mData[1] = (companyID >> 8) & 0xFF; // MSB
  
  // Sao chép chuỗi payload
  payload.getBytes(&mData[2], payloadLen + 1);

  // Dừng quảng cáo cũ
  BLE.stopAdvertise();
  
  // Cập nhật dữ liệu manufacturer mới
  BLE.setManufacturerData(mData, sizeof(mData));
  
  // Bắt đầu quảng cáo lại
  BLE.advertise();
}