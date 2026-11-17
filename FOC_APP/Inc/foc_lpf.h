#ifndef FOC_LPF_H
#define FOC_LPF_H

#include "foc_math.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

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
void FOC_LPF_IniByWc(FOC_LPF_t *lpf, float wc_rad_s, float ts, float init_val);
//3.Dùng để thay đổi động tần số cắt wc ngay trong lúc motor vận hành và thay đổi tốc độ
void FOC_LPF_SetCutoffHz(FOC_LPF_t *lpf, float fc_hz);
void FOC_LPF_SetCutoffWc(FOC_LPF_t *lpf, float wc_rad_s);
//4.Xóa trạng thái cũ à dặt lại toàn bộ giá trị đầu ra LPF init_val cụ thể
void FOC_LPF_Reset(FOC_LPF_t *lpf, float init_val);
//Xóa hết cờ ngát cũ sau khi dã xử lí xong sự cố

//Hàm thực thi thời gian thực 100kHz
/* Cập nhật bộ lọc thông thấp bậc 1 ở mỗi chu kỳ lấy mẫu:
         y[k] = y[k-1] + alpha * (x[k] - y[k-1])
*/
static inline float FOC_LPF_Update(FOC_LPF_t *lpf, float input){
    if(!isfinite(input)){ //chống NaN/Inf: kiểm tra input có hợp lệ không.
        lpf->input_fault = true;
        return lpf->out_val;
    }
    //khởi tạo mẫu đầu tiên
    if(!lpf->is_initialized){           //nếu đây là lần đầu bộ lọc chạy tức cờ is_initialized = false
        lpf->out_val        = input;    //gán thẳng giá trị đầu vào input làm đầu ra và bật cờ true
        lpf->is_initialized = true;     //Để loại bộ để trễ khi bộ lọc phải leo từ 0 lên giá trị thực tế mới cấp nguồn
        return lpf->out_val;
    }
    lpf->out_val += lpf->alpha *(input - lpf->out_val);  //công thức lọc IIR bậc 1
    return lpf->out_val;
}
//Cập nhập động hệ số lọc alpha phục vụ cho Adaptive LPF ngay trong ngắt và SMO
static inline void FOC_LPF_UpdateAlpphaDynamic(FOC_LPF_t *lpf, float wc_rad_s){
     if(!isfinite(wc_rad_s)||(wc_rad_s <= 0.0f)){
            lpf->input_fault = true;
            return;
     }
     lpf->wc_rad_s = wc_rad_s; //lưu lại giá trị wc hiện tại vào struct để các khối khác có thể đọc và sử dụng
     float wc_ts = wc_rad_s * lpf->ts;
     lpf->alpha  = wc_ts/(1.0f + wc_ts);
}
//Tính góc trễ pha (rad) do bộ lọc LPF gây ra ở tần số góc điện we_rad_s:
//phase_lag = atan2(we, wc)  (Dùng để cộng bù góc ước lượng trong SMO)
static inline float FOC_LPF_GetPhaseLag(FOC_LPF_t *lpf, float we_rad_s){
    if(lpf->wc_rad_s <= 0.0f){
        return 0.0f;
    }
    return fast_atan2(we_rad_s,lpf->wc_rad_s);
}
//Tính hệ số suy hao biên độ 
static inline float FOC_LPF_GetAmplitudeGain(FOC_LPF_t *lpf, float wc_rad_s,float w){

}

#endif