#include "encoder.h"

// Cấp phát bộ nhớ và khởi tạo giá trị ban đầu cho các biến đếm
volatile long enc1A_count = 0;
volatile long enc1B_count = 0;
volatile long enc2A_count = 0;
volatile long enc2B_count = 0;

volatile uint8_t enc1State = 0;
volatile uint8_t enc2State = 0;

// Bảng chuyển trạng thái quadrature: mỗi bước hợp lệ là +1 hoặc -1.
const int8_t encoderTransition[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0
};

uint8_t readEncoderState(uint8_t pinA, uint8_t pinB) {
    return (digitalRead(pinA) << 1) | digitalRead(pinB);
}

void IRAM_ATTR enc1_ISR() {
    uint8_t newState = readEncoderState(ENC1_A, ENC1_B);
    enc1A_count += encoderTransition[(enc1State << 2) | newState];
    enc1State = newState;
}

void IRAM_ATTR enc2_ISR() {
    uint8_t newState = readEncoderState(ENC2_A, ENC2_B);
    enc2A_count += encoderTransition[(enc2State << 2) | newState];
    enc2State = newState;
}

void setupEncoders() {
    // Cấu hình chân đầu vào có điện trở kéo lên
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    enc1State = readEncoderState(ENC1_A, ENC1_B);
    enc2State = readEncoderState(ENC2_A, ENC2_B);

    // Bắt mọi thay đổi của A và B để không bỏ qua chiều quay.
    attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2_ISR, CHANGE);
}

void resetEncoders() {
    enc1A_count = 0;
    enc1B_count = 0;
    enc2A_count = 0;
    enc2B_count = 0;
}