#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Định nghĩa chân kết nối Encoder
#define ENC1_A 1
#define ENC1_B 2
#define ENC2_A 3
#define ENC2_B 4

// Dùng extern để cho phép các file khác (như main.cpp) đọc được giá trị các biến này
extern volatile long enc1A_count;
extern volatile long enc1B_count;
extern volatile long enc2A_count;
extern volatile long enc2B_count;

void setupEncoders();
void resetEncoders();

#endif