
#include <Arduino.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <Wire.h>

#define SDA_PIN 19
#define SCL_PIN 18
#define XSHUT_LEFT 20
#define XSHUT_FRONT 9
#define XSHUT_RIGHT 8
#define M1_IN1 6
#define M1_IN2 7
#define M2_IN1 14
#define M2_IN2 15

VL53L0X sensorLeft;
VL53L0X sensorFront;
VL53L0X sensorRight;
MPU6050 mpu6050(0x68);

bool leftReady = false;
bool frontReady = false;
bool rightReady = false;
bool mpuReady = false;
bool turnTestDone = false;

void stopMotors() {
	digitalWrite(M1_IN1, LOW);
	digitalWrite(M1_IN2, LOW);
	digitalWrite(M2_IN1, LOW);
	digitalWrite(M2_IN2, LOW);
}

// Phanh ngắn mạch chủ động để hãm quán tính cơ khí động cơ N20
void brakeMotors() {
	digitalWrite(M1_IN1, HIGH);
	digitalWrite(M1_IN2, HIGH);
	digitalWrite(M2_IN1, HIGH);
	digitalWrite(M2_IN2, HIGH);
	delay(25);
	stopMotors();
}

// Bù quán tính riêng cho rẽ trái (tăng lên 18.0 độ để ngắt sớm hơn, triệt tiêu lố góc)
constexpr float LEFT_COMPENSATION = 18.0f;
// Bù quán tính riêng cho rẽ phải (giữ 8.0 độ đã chuẩn)
constexpr float RIGHT_COMPENSATION = 8.0f;
// Tốc độ PWM quay (0-255): 90 giúp xe quay đầm, phanh dứt khoát không bị trượt
constexpr uint8_t TURN_SPEED = 90;

/**
 * Bước 1 - 4: Rẽ phải tại chỗ 90 độ (hoặc góc tùy chọn) sử dụng MPU6050
 */
void turnRight(float angle = 90.0f, float compensation = RIGHT_COMPENSATION, uint8_t speed = TURN_SPEED) {
	if (!mpuReady) {
		Serial.println("MPU6050 chua san sang, khong the quay!");
		return;
	}
	bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2, compensation, speed);
	Serial.println(completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT");
}

/**
 * Bước 1 - 4: Rẽ trái tại chỗ 90 độ (hoặc góc tùy chọn) sử dụng MPU6050
 */
void turnLeft(float angle = 90.0f, float compensation = LEFT_COMPENSATION, uint8_t speed = TURN_SPEED) {
	if (!mpuReady) {
		Serial.println("MPU6050 chua san sang, khong the quay!");
		return;
	}

	bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2, compensation, speed);
	Serial.println(completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT");
}

void runTurnTest() {
	if (!mpuReady) return;

	Serial.println("==========================================");
	Serial.println(">> TEST: CHUAN BI QUAY TRAI 90 DO...");
	delay(800);
	turnLeft(90.0f);
	stopMotors();
	delay(1500); // Nghỉ 1.5 giây để bạn quan sát góc thực tế

	Serial.println(">> TEST: CHUAN BI QUAY PHAI 90 DO VE HUONG CU...");
	delay(800);
	turnRight(90.0f);
	stopMotors();
	delay(1500); // Nghỉ 1.5 giây để bạn quan sát góc thực tế
	Serial.println(">> TEST: HOAN TAT 1 CHU KY. TIEP TUC LAP LAI...");
}

bool initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address, const char *name) {
	digitalWrite(xshutPin, HIGH);
	delay(50);
	sensor.setTimeout(500);

	if (!sensor.init()) {
		Serial.print(name);
		Serial.println(" VL53 FAIL");
		return false;
	}

	sensor.setAddress(address);
	sensor.startContinuous();
	Serial.print(name);
	Serial.print(" VL53 OK @ 0x");
	Serial.println(address, HEX);
	return true;
}

void setup() {
	Serial.begin(115200);
	delay(500);

	pinMode(M1_IN1, OUTPUT);
	pinMode(M1_IN2, OUTPUT);
	pinMode(M2_IN1, OUTPUT);
	pinMode(M2_IN2, OUTPUT);
	stopMotors();
	Serial.println("MOTORS STOPPED");

	Wire.begin(SDA_PIN, SCL_PIN);
	delay(100);

	pinMode(XSHUT_LEFT, OUTPUT);
	pinMode(XSHUT_FRONT, OUTPUT);
	pinMode(XSHUT_RIGHT, OUTPUT);
	digitalWrite(XSHUT_LEFT, LOW);
	digitalWrite(XSHUT_FRONT, LOW);
	digitalWrite(XSHUT_RIGHT, LOW);
	delay(100);

	leftReady = initVL53(sensorLeft, XSHUT_LEFT, 0x30, "LEFT");
	frontReady = initVL53(sensorFront, XSHUT_FRONT, 0x31, "FRONT");
	rightReady = initVL53(sensorRight, XSHUT_RIGHT, 0x32, "RIGHT");

	Wire.beginTransmission(0x68);
	if (Wire.endTransmission() == 0) {
		Serial.println("MPU6050 FOUND @ 0x68");
		mpuReady = mpu6050.begin();
		if (mpuReady) {
			mpu6050.calibrate();
			Serial.println("MPU6050 READY - Z AXIS ONLY");
		} else {
			Serial.println("MPU6050 INIT FAIL");
		}
	} else {
		Serial.println("MPU6050 NOT FOUND @ 0x68");
	}

}

void loop() {
	if (mpuReady) {
		runTurnTest(); // Quay liên tục lặp đi lặp lại
	} else {
		stopMotors();
		delay(500);
	}
}
