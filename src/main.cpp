#include <Arduino.h>
#include "RobotConfig.h"
#include "BLEManager.h"
#include "RobotNav.h"
#include "CommandHandler.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    // 1. Khởi tạo kết nối BLE Server
    bleManager.begin("MM-Robot-BLE");

    // 2. Khởi tạo động cơ, cảm biến VL53L0X và Gyro MPU6050
    robotNav.init();

    Serial.println("==========================================");
    Serial.println(">> ROBOT MICROMOUSE READY!");
    Serial.println("==========================================");
}

void loop() {
    // 1. Cập nhật di chuyển & vòng lặp PID bám tường
    robotNav.update();

    // 2. Cập nhật xử lý lệnh từ điện thoại qua BLE & phát Telemetry JSON
    commandHandler.update();
}
