#ifndef FOC_LPF_H
#define FOC_LPF_H

#include "foc_math.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif
#define FOC_LPF_WCTS_MIN  1.0e-7f
#define FOC_LPF_WCTS_MAX  1.0e6f
//1.Tại sao lưu out_val trong struct, để lưu lại giá trị qua trừng chu kì ngắt nếu
// gọi biến cục bộ trong hàm thì sẽ bị xóa sạch sau khi hàm ended.
//2.Alpha sao phải lưu mà không tính từ fc_ts trong hàm Update.
//để không phải lặp lại các nhân chia tính toán
//3.Tại sao phải lưu ts
// bộ lọc thích nghi nơi wc thay đổi liên tục theo motor
//4.Tại sao có input_fault
// phát hiện ra lỗi số học NaN/Inf.

typedef struct{
    float alpha;        // hệ số lọc rời tạc
    float wc_rad_s;     //tần số cắt góc hiện tại
    float ts;           //chu kì lấy mẫu Ts
    float out_val;       //Trạng thái/giá trị đầu ra hiện tại y[k]

    bool is_initialized;//Cờ báo đã nạp mẫu đầu tiên
    bool input_fault;   //Cờ báo lỗi có tham số không hợp lệ NaN/Inf
    
} FOC_LPF_t;

//Nguyên mẫu các hàm khởi tạo và cấu hình
//1.Khởi tạo bộ lọc bằng tần số cắt(hz),chu kì lấy mẫu Ts, và giá trị khởi đầu init_val x[k]
void FOC_LPF_Init(FOC_LPF_t *lpf, float fc_hz, float ts,float init_val);
//2.giống hàm trên nhưng tần số là bằng rad/s, tiện làm với tốc độ góc we của PLL
void FOC_LPF_InitByWc(FOC_LPF_t *lpf, float wc_rad_s, float ts, float init_val);
//3.Dùng để thay đổi động tần số cắt wc ngay trong lúc motor vận hành và thay đổi tốc độ
void FOC_LPF_SetCutoffHz(FOC_LPF_t *lpf, float fc_hz);
void FOC_LPF_SetCutoffWc(FOC_LPF_t *lpf, float wc_rad_s);
//4.Xóa trạng thái cũ à dặt lại toàn bộ giá trị đầu ra LPF init_val cụ thể
void FOC_LPF_Reset(FOC_LPF_t *lpf, float init_val);
//Xóa hết cờ ngát cũ sau khi dã xử lí xong sự cố
void FOC_LPF_ClearFault(FOC_LPF_t *lpf);
static inline bool FOC_LPF_WcTsValid(float wc_ts)
{
    return (wc_ts >= FOC_LPF_WCTS_MIN) && (wc_ts <= FOC_LPF_WCTS_MAX);   // NaN -> false
}
static inline float FOC_LPF_Update(FOC_LPF_t *lpf, float input)
{
    if (!lpf->is_initialized || !isfinite(input)) {
        lpf->input_fault = true;
        return lpf->out_val; // Fail-Safe
    }

    lpf->out_val += lpf->alpha * (input - lpf->out_val);
    return lpf->out_val;
}
//Cập nhật hệ số alpha động theo tần số cắt góc wc_rad_s (Adaptive LPF)
static inline void FOC_LPF_UpdateAlphaDynamic(FOC_LPF_t *lpf, float wc_rad_s)
{
    if (!lpf->is_initialized || !isfinite(wc_rad_s)) {
        lpf->input_fault = true;
        return;
    }
    float wc_ts = wc_rad_s * lpf->ts;
    if (!FOC_LPF_WcTsValid(wc_ts)) {
        lpf->input_fault = true;
        return;
    }
    lpf->wc_rad_s = wc_rad_s;
    lpf->alpha    = wc_ts / (1.0f + wc_ts);
}
//Tính chính xác góc trễ pha rời rạc (Bản chuẩn Exact) theo 
static inline float FOC_LPF_GetPhaseLagExact(const FOC_LPF_t *lpf, float we_rad_s)
{
    if (!isfinite(we_rad_s) || !lpf->is_initialized) {
        return 0.0f;
    }
    float th = we_rad_s * lpf->ts;
    if (th > PI)  { th = PI; }
    if (th < -PI) { th = -PI; }

    float s, c;
    fast_sincos(th, &s, &c);
    float beta = 1.0f - lpf->alpha;
    return fast_atan2(beta * s, 1.0f - (beta * c));
}
//Tính góc trễ pha tối ưu tốc độ bằng Khai triển Taylor (Sai số <= 0.006° khi |we*ts| <= 1)
static inline float FOC_LPF_GetPhaseLagFast(const FOC_LPF_t *lpf, float we_rad_s){
    if (!isfinite(we_rad_s) || !lpf->is_initialized) { return 0.0f; }
    float th = we_rad_s * lpf->ts;
    if ((th > 1.0f) || (th < -1.0f)) { return FOC_LPF_GetPhaseLagExact(lpf, we_rad_s); }
    float t2  = th * th;
    float num = th * (1.0f - (t2 * (1.0f / 6.0f)) + (t2 * t2 * (1.0f / 120.0f)));              // sin(th)
    float omc = t2 * (0.5f - (t2 * (1.0f / 24.0f)) + (t2 * t2 * (1.0f / 720.0f)));             // 1 - cos(th)
    float den = (lpf->wc_rad_s * lpf->ts) + omc;                                               // = 1 + wc*Ts - cos(th)
    return fast_atan2(num, den);
}
static inline void FOC_LPF_GetLagRotation(const FOC_LPF_t *lpf, float we_rad_s, float *cos_lag, float *sin_lag){
    if (!isfinite(we_rad_s) || !lpf->is_initialized) { *cos_lag = 1.0f; *sin_lag = 0.0f; return; }
    float th = we_rad_s * lpf->ts, num, den;
    if ((th > 1.0f) || (th < -1.0f)) {
        float s, c;
        if (th >  PI) { th =  PI; }
        if (th < -PI) { th = -PI; }
        fast_sincos(th, &s, &c);
        num = s;  den = 1.0f + (lpf->wc_rad_s * lpf->ts) - c;
    } else {
        float t2 = th * th;
        num = th * (1.0f - (t2 * (1.0f / 6.0f)) + (t2 * t2 * (1.0f / 120.0f)));
        den = (lpf->wc_rad_s * lpf->ts) + (t2 * (0.5f - (t2 * (1.0f / 24.0f)) + (t2 * t2 * (1.0f / 720.0f))));
    }
    float inv = 1.0f / fast_sqrt((num * num) + (den * den));
    *cos_lag = den * inv;  *sin_lag = num * inv;
}
static inline float FOC_LPF_GetAmplitudeGainExact(const FOC_LPF_t *lpf, float we_rad_s){
    if (!isfinite(we_rad_s) || !lpf->is_initialized) {
        return 1.0f;
    }
    float th = we_rad_s * lpf->ts;
    if (th > PI)  { th = PI; }
    if (th < -PI) { th = -PI; }
    float s, c;
    fast_sincos(th, &s, &c);
    (void)s;
    float beta = 1.0f - lpf->alpha;
    return lpf->alpha / fast_sqrt(1.0f - (2.0f * beta * c) + (beta * beta));
}
#ifdef __cplusplus
}
#endif
#endif