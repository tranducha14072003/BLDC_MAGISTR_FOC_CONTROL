#ifndef SMO_PLL_H_
#define SMO_PLL_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    // Thông số cơ bản của động cơ thực tế
    float rs;   //Điện trở stator
    float ls;   //Điện cảm stator (H)
    float ts;   //chu kì lấy mẫu
    float ts_over_ls;   // hằng số tính sẵn ts/ls tối uuw FPU
     
    // Biến trạng thái ước lượng dòng điện (Current observer)
    float i_hat_alpha;
    float i_hat_beta;
    //Tín hiệu điều khiển trượt
    float z_alpha;
    float z_beta;
}SMO_PLL_t;
//Khởi tạo thông số ban đầu
void SMO_PLL_Init(SMO_PLL_t *smo, float rs, float ls, float ts);
//Hàm tính toán phần dòng điện ước lượng
void SMO_PLL_UpdateCurrentObserver(SMO_PLL_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide);

#endif