#ifndef PID_INCS_PID_UTILS_H
#define PID_INCS_PID_UTILS_H

#include <stdint.h>

typedef struct {
    float Kp;        // Hệ số tỷ lệ
    float Ki;        // Hệ số tích phân
    float Kd;        // Hệ số đạo hàm
    float integral;  // Giá trị tích phân
    float prev_error; // Lỗi trước đó
} PID_Controller;

float pid_compute(PID_Controller *pid, float setpoint, float measured_value, float dt); 
#endif // PID_INCS_PID_UTILS_H