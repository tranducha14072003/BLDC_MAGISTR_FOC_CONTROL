#ifndef smo_utils_h_
#define smo_utils_h_

#include <stdint.h>
#include <stdbool.h>
#include "foc_lpf.h"
#include "foc_math.h"
#include "math."
typedef struct{
    // Thông số cơ bản của động cơ thực tế
    float rs;   //Điện trở stator
    float ls;   //Điện cảm stator (H)
    float ts;   //chu kì lấy mẫu
    float ts_over_ls;   // hằng số tính sẵn ts/ls tối uuw FPU
    float delta;
    float inv_delta;
    // Biến trạng thái ước lượng dòng điện (Current observer)
    float i_hat_alpha;
    float i_hat_beta;
    //Tín hiệu điều khiển trượt
    float z_alpha;
    float z_beta;
    bool input_fault;
    bool is_initialized;
}FOC_SMO_t;
//Khởi tạo thông số ban đầu
void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts, float delta);
//Hàm tính toán phần dòng điện ước lượng
void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide);

void FOC_SMO_ClearFault(FOC_SMO_t *smo);
#endif