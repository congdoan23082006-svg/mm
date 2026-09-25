#include "CommandHandler.h"

CommandHandler commandHandler;

CommandHandler::CommandHandler() { _lastTelemetryTime = 0; }

void CommandHandler::update() {
  processBLECommands();
  sendTelemetry();
}

void CommandHandler::processBLECommands() {
  if (!bleManager.hasCommand())
    return;

  String cmd = bleManager.readCommand();
  cmd.toUpperCase();

  if (cmd == "TL" || cmd == "TEST_LEFT") {
    robotNav.pidRunActive = false;
    bleManager.println(">> [BLE] LENH: RE TRAI 90 DO");
    robotNav.turnLeft(90.0f);
    robotNav.stopMotors();
  } else if (cmd == "TR" || cmd == "TEST_RIGHT") {
    robotNav.pidRunActive = false;
    bleManager.println(">> [BLE] LENH: RE PHAI 90 DO");
    robotNav.turnRight(90.0f);
    robotNav.stopMotors();
  } else if (cmd == "PID_ON" || cmd == "FORWARD" || cmd == "FWD") {
    robotNav.startPID();
    bleManager.println(
        ">> [BLE] KICH HOAT PID BAM TUONG & GIU HUONG TIEN THANG!");
  } else if (cmd == "PID_OFF" || cmd == "STOP" || cmd == "ST") {
    robotNav.stopPID();
    bleManager.println(">> [BLE] DA DUNG XE (PID OFF)");
  } else if (cmd == "RESET_YAW" || cmd == "RST") {
    robotNav.mpu6050.resetYaw();
    bleManager.println(">> [BLE] DA RESET GOC YAW VE 0.0 DO!");
  } else if (cmd == "CALIB" || cmd == "CALIBRATE") {
    robotNav.stopMotors();
    bleManager.println(
        ">> [BLE] DANG HIEU CHUAN GYRO... VUI LONG DE YEN ROBOT 1.5S!");
    robotNav.mpu6050.calibrate(400);
    bleManager.println(">> [BLE] HIEU CHUAN GYRO HOAN TAT! GOC YAW VE 0.0 DO.");
  } else if (cmd.startsWith("SET_LC=")) {
    float val = cmd.substring(7).toFloat();
    if (val >= 0.0f && val <= 50.0f) {
      robotNav.leftCompensation = val;
      bleManager.println(">> [BLE] CAP NHAT LEFT_COMPENSATION = " +
                         String(robotNav.leftCompensation, 1));
    }
  } else if (cmd.startsWith("SET_RC=")) {
    float val = cmd.substring(7).toFloat();
    if (val >= 0.0f && val <= 50.0f) {
      robotNav.rightCompensation = val;
      bleManager.println(">> [BLE] CAP NHAT RIGHT_COMPENSATION = " +
                         String(robotNav.rightCompensation, 1));
    }
  } else if (cmd.startsWith("SET_SPD=")) {
    int val = cmd.substring(8).toInt();
    if (val >= 40 && val <= 255) {
      robotNav.turnSpeed = (uint8_t)val;
      robotNav.baseForwardSpeed = (uint8_t)val;
      bleManager.println(">> [BLE] CAP NHAT SPEED = " +
                         String(robotNav.turnSpeed));
    }
  } else if (cmd.startsWith("SET_FSPD=")) {
    int val = cmd.substring(9).toInt();
    if (val >= 40 && val <= 255) {
      robotNav.baseForwardSpeed = (uint8_t)val;
      bleManager.println(">> [BLE] CAP NHAT BASE_FORWARD_SPEED = " +
                         String(robotNav.baseForwardSpeed));
    }
  } else if (cmd.startsWith("SET_KP=")) {
    float val = cmd.substring(7).toFloat();
    robotNav.wallPID.setGains(val, robotNav.wallPID.getKi(),
                              robotNav.wallPID.getKd());
    robotNav.gyroPID.setGains(val * 1.5f, robotNav.gyroPID.getKi(),
                              robotNav.gyroPID.getKd());
    bleManager.println(">> [BLE] CAP NHAT PID Kp = " + String(val, 2));
  } else if (cmd.startsWith("SET_KI=")) {
    float val = cmd.substring(7).toFloat();
    robotNav.wallPID.setGains(robotNav.wallPID.getKp(), val,
                              robotNav.wallPID.getKd());
    robotNav.gyroPID.setGains(robotNav.gyroPID.getKp(), val,
                              robotNav.gyroPID.getKd());
    bleManager.println(">> [BLE] CAP NHAT PID Ki = " + String(val, 3));
  } else if (cmd.startsWith("SET_KD=")) {
    float val = cmd.substring(7).toFloat();
    robotNav.wallPID.setGains(robotNav.wallPID.getKp(),
                              robotNav.wallPID.getKi(), val);
    robotNav.gyroPID.setGains(robotNav.gyroPID.getKp(),
                              robotNav.gyroPID.getKi(), val * 1.5f);
    bleManager.println(">> [BLE] CAP NHAT PID Kd = " + String(val, 2));
  } else if (cmd == "AUTO_ON") {
    robotNav.pidRunActive = false;
    robotNav.autoTestMode = true;
    bleManager.println(">> [BLE] KICH HOAT CHEDO AUTO TEST RE TRAI/PHAI");
  } else if (cmd == "AUTO_OFF") {
    robotNav.autoTestMode = false;
    robotNav.pidRunActive = false;
    robotNav.stopMotors();
    bleManager.println(">> [BLE] TAT CHEDO AUTO TEST");
  } else if (cmd.startsWith("SET_TLD=")) {
    float val = cmd.substring(8).toFloat();
    if (val >= 50.0f && val <= 300.0f) {
      robotNav.targetLeftDist = val;
      robotNav.centerOffset =
          robotNav.targetLeftDist - robotNav.targetRightDist;
      bleManager.println(">> [BLE] CAP NHAT TARGET_LEFT_DIST = " +
                         String(val, 1));
    }
  } else if (cmd.startsWith("SET_TRD=")) {
    float val = cmd.substring(8).toFloat();
    if (val >= 50.0f && val <= 300.0f) {
      robotNav.targetRightDist = val;
      robotNav.centerOffset =
          robotNav.targetLeftDist - robotNav.targetRightDist;
      bleManager.println(">> [BLE] CAP NHAT TARGET_RIGHT_DIST = " +
                         String(val, 1));
    }
  } else if (cmd.startsWith("SET_WTH=")) {
    int val = cmd.substring(8).toInt();
    if (val >= 100 && val <= 400) {
      robotNav.wallThreshold = (uint16_t)val;
      bleManager.println(">> [BLE] CAP NHAT WALL_THRESHOLD = " + String(val));
    }
  } else if (cmd.startsWith("SET_FSTOP=")) {
    int val = cmd.substring(10).toInt();
    if (val >= 30 && val <= 150) {
      robotNav.frontStopDist = (uint16_t)val;
      bleManager.println(">> [BLE] CAP NHAT FRONT_STOP_DIST = " + String(val));
    }
  } else if (cmd == "RESET_ENC" || cmd == "RST_ENC") {
    robotNav.resetEnc();
    bleManager.println(">> [BLE] DA RESET XUNG ENCODER VE 0");
  } else if (cmd == "INV_ENCL") {
    robotNav.invertEncLeft = !robotNav.invertEncLeft;
    bleManager.println(">> [BLE] DAO CHIEU ENCODER TRAI: " +
                       String(robotNav.invertEncLeft ? "INVERTED (-)" : "NORMAL (+)"));
  } else if (cmd == "INV_ENCR") {
    robotNav.invertEncRight = !robotNav.invertEncRight;
    bleManager.println(">> [BLE] DAO CHIEU ENCODER PHAI: " +
                       String(robotNav.invertEncRight ? "INVERTED (-)" : "NORMAL (+)"));
  } else if (cmd == "STATUS" || cmd == "GET") {
    printStatus();
  }
}

void CommandHandler::sendTelemetry() {
  if (bleManager.isConnected() && millis() - _lastTelemetryTime >= 200) {
    _lastTelemetryTime = millis();
    float yaw = robotNav.mpu6050.getYaw();
    uint16_t dLeft = robotNav.leftReady
                         ? robotNav.sensorLeft.readRangeContinuousMillimeters()
                         : 0;
    uint16_t dFront =
        robotNav.frontReady
            ? robotNav.sensorFront.readRangeContinuousMillimeters()
            : 0;
    uint16_t dRight =
        robotNav.rightReady
            ? robotNav.sensorRight.readRangeContinuousMillimeters()
            : 0;
    long encL = robotNav.getLeftEncoder();
    long encR = robotNav.getRightEncoder();

    String jsonMsg =
        "{\"type\":\"telemetry\",\"yaw\":" + String(yaw, 1) +
        ",\"l\":" + String(dLeft) + ",\"f\":" + String(dFront) +
        ",\"r\":" + String(dRight) +
        ",\"el\":" + String(encL) + ",\"er\":" + String(encR) +
        ",\"lc\":" + String(robotNav.leftCompensation, 1) +
        ",\"rc\":" + String(robotNav.rightCompensation, 1) +
        ",\"spd\":" + String(robotNav.turnSpeed) +
        ",\"kp\":" + String(robotNav.wallPID.getKp(), 2) +
        ",\"ki\":" + String(robotNav.wallPID.getKi(), 3) +
        ",\"kd\":" + String(robotNav.wallPID.getKd(), 2) +
        ",\"pid\":" + String(robotNav.pidRunActive ? "true" : "false") +
        ",\"auto\":" + String(robotNav.autoTestMode ? "true" : "false") +
        ",\"lspd\":" + String(robotNav.currentLeftSpeed) +
        ",\"rspd\":" + String(robotNav.currentRightSpeed) +
        ",\"fspd\":" + String(robotNav.baseForwardSpeed) +
        ",\"tld\":" + String(robotNav.targetLeftDist, 1) +
        ",\"trd\":" + String(robotNav.targetRightDist, 1) +
        ",\"wth\":" + String(robotNav.wallThreshold) +
        ",\"fstop\":" + String(robotNav.frontStopDist) + "}";
    bleManager.println(jsonMsg);
  }
}

void CommandHandler::printStatus() {
  uint16_t dLeft = robotNav.leftReady
                       ? robotNav.sensorLeft.readRangeContinuousMillimeters()
                       : 0;
  uint16_t dFront = robotNav.frontReady
                        ? robotNav.sensorFront.readRangeContinuousMillimeters()
                        : 0;
  uint16_t dRight = robotNav.rightReady
                        ? robotNav.sensorRight.readRangeContinuousMillimeters()
                        : 0;

  robotNav.mpu6050.update();
  float yaw = robotNav.mpu6050.getYaw();

  String statusMsg = "--- STATS ---\n";
  statusMsg += "Yaw: " + String(yaw, 2) + " deg\n";
  statusMsg += "TOF (L/F/R): " + String(dLeft) + " / " + String(dFront) +
               " / " + String(dRight) + " mm\n";
  statusMsg += "PARAMS -> LEFT_COMP: " + String(robotNav.leftCompensation, 1) +
               " | RIGHT_COMP: " + String(robotNav.rightCompensation, 1) +
               " | SPEED: " + String(robotNav.turnSpeed) + "\n";
  statusMsg += "PID GAINS -> Kp: " + String(robotNav.wallPID.getKp(), 2) +
               " | Ki: " + String(robotNav.wallPID.getKi(), 3) +
               " | Kd: " + String(robotNav.wallPID.getKd(), 2) + "\n";
  statusMsg += "PID ACTIVE: " + String(robotNav.pidRunActive ? "YES" : "NO");

  Serial.println(statusMsg);
  bleManager.println(statusMsg);
}
