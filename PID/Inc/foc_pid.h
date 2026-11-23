#ifndef FOC_PID_H
#define FOC_PID_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float kp;           // Hệ số tỷ lệ
    float ki;           // Hệ số tích phân
    float ts;           // Chu kì lấy mẫu Ts
    float ki_ts;        // Hằng số tính sẵn ki*ts
    float integral;     // Giá trị tích phân
    float out_min;      // Ngưỡng bão hòa dưới
    float out_max;      // Ngưỡng bão hòa trên
    float out_val;      // Giá trị đầu ra hiện tại
    bool  is_saturated; // Cờ báo đầu ra chạm ngưỡng bão hòa
    bool  input_fault;  // Tham số hoặc ref/meas/ff không hợp lệ (NaN, inf, quá lớn)
} FOC_PI_t;
// Khởi tạo bộ điều khiển PI
void FOC_PI_Init(FOC_PI_t *pi, float kp, float ki, float ts, float out_min, float out_max);
// Cập nhật hệ số kp,ki trực tuyến
void FOC_PI_SetGains(FOC_PI_t *pi, float kp, float ki);
// Tự tính Kp = L*wc, Ki = R*wc cho vòng dòng điện
void FOC_PI_TuneCurrent(FOC_PI_t *pi, float rs_ohm, float ls_henry, float wc_rad_s);
// Cập nhật giới hạn bão hòa đầu ra động
void FOC_PI_SetOutputLimits(FOC_PI_t *pi, float out_min, float out_max);
// Xóa sạch bộ nhớ tích phân và đầu ra về 0 (KHÔNG xóa input_fault)
void FOC_PI_Reset(FOC_PI_t *pi);
// Xóa cờ lỗi đầu vào
void FOC_PI_ClearFault(FOC_PI_t *pi);
// Nạp trước tích phân
void FOC_PI_PreloadIntegral(FOC_PI_t *pi, float preload_val);
// Nạp tích phân để chuyển mạch êm: lần FOC_PI_UpdateFF đầu cho đúng u_prev
void FOC_PI_PreloadBumpless(FOC_PI_t *pi, float u_prev, float ref_now, float meas_now, float ff_now);
// Hàm tính toán PI thực thi trong ngắt (Clamping Anti-Windup). ff được tính vào cả phép kẹp lẫn anti-windup.
static inline float FOC_PI_UpdateFF(FOC_PI_t *pi, float ref, float meas, float ff)
{
    float err = ref - meas;
    // NaN, inf hoặc giá trị vô lý: không để nhiễm vào tích phân (so sánh với NaN luôn false)
    if (!(fabsf(err) < 1.0e6f) || !(fabsf(ff) < 1.0e6f)) {
        pi->integral     = 0.0f;
        pi->out_val      = 0.0f;
        pi->is_saturated = false;
        pi->input_fault  = true;
        return 0.0f;
    }
    float p_term  = (pi->kp * err) + ff;
    float i_cand  = pi->integral + (pi->ki_ts * err);
    float u_unsat = p_term + i_cand;
    // Khóa tích phân nếu đầu ra vượt biên VÀ sai số vẫn đẩy cùng chiều vượt biên
    bool clamp_high = (u_unsat > pi->out_max) && (err > 0.0f);
    bool clamp_low  = (u_unsat < pi->out_min) && (err < 0.0f);
    if (!clamp_high && !clamp_low) {
        if (i_cand > pi->out_max)      { pi->integral = pi->out_max; }
        else if (i_cand < pi->out_min) { pi->integral = pi->out_min; }
        else                           { pi->integral = i_cand; }
    }
    float u_out = p_term + pi->integral;
    if (u_out > pi->out_max)      { u_out = pi->out_max; pi->is_saturated = true; }
    else if (u_out < pi->out_min) { u_out = pi->out_min; pi->is_saturated = true; }
    else                          { pi->is_saturated = false; }

    pi->out_val = u_out;
    return u_out;
}
static inline float FOC_PI_Update(FOC_PI_t *pi, float ref, float meas)
{
    return FOC_PI_UpdateFF(pi, ref, meas, 0.0f);
}
#ifdef __cplusplus
}
#endif
#endif // foc_pid_h