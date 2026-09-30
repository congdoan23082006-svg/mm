#ifndef ROBOT_NAV_H
#define ROBOT_NAV_H

#include "BLEManager.h"
#include "PIDController.h"
#include "RobotConfig.h"
#include "encoder.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <Wire.h>

struct WallStatus {
  bool hasFront; // Có tường trước
  bool hasLeft;  // Có tường trái
  bool hasRight; // Có tường phải
};

class RobotNav {
public:
  RobotNav();

  void init();
  void update();

  void stopMotors();
  void brakeMotors();

  void turnLeft(float angle = 90.0f);
  void turnRight(float angle = 90.0f);
  void runTurnTest();
  void updatePIDLoop();
  void startPID();
  void stopPID();
  void stepCell(int numCells = 1);

  // 4 Hàm Chuyển Động Nguyên Tử (Atomic Motion - Bước 3)
  WallStatus moveOneCell();
  WallStatus turnLeftAndStep();
  WallStatus turnRightAndStep();
  WallStatus turnAroundAndStep();
  WallStatus senseCurrentWalls();

  // Chế Độ 3: Bám Tường Tự Động (Autonomous Wall Follower)
  void startAutoWallFollow();
  void stopAutoWallFollow();
  void stepAutoWallFollow();
  bool autoWallFollowActive;
  bool followRightHand; // true: Tay Phải, false: Tay Trái
  int autoCellCount;    // Đếm số ô đã đi
  int autoMaxCells;     // Giới hạn an toàn (mặc định 60 ô)
  String lastAutoDecision;

  // Encoder helper methods
  long getLeftEncoder() const;
  long getRightEncoder() const;
  void resetEnc();

  // Chạy từng ô theo Encoder (Cell Stepping)
  long pulsesPerCell;
  bool stepCellActive;
  long stepStartPulses;
  long stepStartL;
  long stepStartR;
  long stepTargetPulses;
  long stepTraveledPulses;
  unsigned long stepStartTime;

  // Các biến thông số có thể điều chỉnh
  float leftCompensation;
  float rightCompensation;
  uint8_t turnSpeed;
  uint8_t baseForwardSpeed;
  int currentLeftSpeed;
  int currentRightSpeed;
  bool autoTestMode;
  bool pidRunActive;

  // Cấu hình đồng bộ bánh bằng Encoder (Cascaded Inner Loop)
  bool encoderSyncActive;
  float encKp;
  bool invertEncLeft;
  bool invertEncRight;

  // Thông số hiệu chuẩn khoảng cách ô (VL53L0X)
  float targetLeftDist;
  float targetRightDist;
  float centerOffset;
  uint16_t wallThreshold;
  uint16_t frontStopDist;

  // Cảm biến & PID
  VL53L0X sensorLeft;
  VL53L0X sensorFront;
  VL53L0X sensorRight;
  MPU6050 mpu6050;

  PIDController wallPID;
  PIDController gyroPID;

  bool leftReady;
  bool frontReady;
  bool rightReady;
  bool mpuReady;

  // Dữ liệu cảm biến sau lọc & Vùng chết tâm ô
  float smoothDL;
  float smoothDF;
  float smoothDR;
  float currentWallError;
  float wallDeadband; // Vùng chết khử nhiễu tâm ô (mm)

  void updateSensors();

private:
  float _targetYaw;
  unsigned long _lastPIDLoopTime;
  unsigned long _lastSensorReadTime;
  long _lastEncLeft;
  long _lastEncRight;
  long _startEncLeft;
  long _startEncRight;
  float _smoothError;

  bool initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address,
                const char *name);
};

extern RobotNav robotNav;

#endif
