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



#endif