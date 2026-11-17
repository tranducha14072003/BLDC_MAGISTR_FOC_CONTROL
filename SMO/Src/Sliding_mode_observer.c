#include "Sliding_mode_observer.h"

//Hàm khởi tạo thông số cho SMO
void SMO_PLL_Init(SMO_PLL_t *smo, float rs, float ls, float ts){
    smo->rs         = rs;
    smo->ls         = ls;
    smo->ts         = ts;
    smo->ts_over_ls = ts/ls;
    smo->i_hat_alpha = 0.0f;
    smo->i_hat_beta  = 0.0f;
    smo->z_alpha     = 0.0f;
    smo->z_beta      = 0.0f;
}