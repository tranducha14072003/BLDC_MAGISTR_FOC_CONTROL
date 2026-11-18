#include "smo_utils.h"

//Hàm piecewise saturation  liên tục có kẹp biên đầu vào để chống tràn
//Có độ dày dải biên boundary layers thickness , delta = 0,1A)
static inline float smo_piecewise_sat(float error, float inv_delta){
    float y = error*inv_delta;
    if(y > 1.0f){
        return 1.0f;
    }
    if(y < -1.0f){
        return -1.0f;
    }
    //Vùng bên trong dải bien: biến thiên tuyến tính mượt mà từ -1 đến 1
    return y;
}
//Hàm khởi tạo thông số cho SMO
void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts, float delta){
    smo->input_fault = false;
    if (!isfinite(rs) || !isfinite(ls) || !isfinite(ts) || !isfinite(delta) ||
        !(rs >= 0.0f) || !(ls > 0.0f) || !(ts > 0.0f) || !(delta > 0.0f))
    {
        smo->is_initialized = false;
        smo->input_fault    = true;
        return;
    }
    smo->rs         = rs;
    smo->ls         = ls;
    smo->ts         = ts;
    smo->ts_over_ls = ts/ls;
    smo->delta       = delta;
    smo->i_hat_alpha = 0.0f;  
    smo->i_hat_beta  = 0.0f;
    smo->z_alpha     = 0.0f;  
    smo->z_beta      = 0.0f;
    smo->is_initialized = true;
    smo->inv_delta   = 1.0f/delta;
}

 

void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide)
{
   if (!smo->is_initialized || !isfinite(u_alpha) || !isfinite(u_beta) ||
    !isfinite(i_alpha) || !isfinite(i_beta) || !isfinite(k_slide))
{
    smo->input_fault = true;   // ← true, và KHÔNG sửa î (giữ trạng thái cũ)
    return;
}
    // Bước 1: Tính sai số dòng điện (Sai số = Ưu lượng - Thực tế)
    float error_alpha = smo->i_hat_alpha - i_alpha;
    float error_beta  = smo->i_hat_beta  - i_beta;

    // Bước 2: Đưa sai số qua hàm piecewise để lấy tín hiệu trượt mượt mà (z_alpha, z_beta)
    smo->z_alpha = k_slide * smo_piecewise_sat(error_alpha, smo->inv_delta);
    smo->z_beta  = k_slide * smo_piecewise_sat(error_beta, smo->inv_delta);

    // Bước 3: Cập nhật dòng điện ước lượng mới theo phương trình vi phân (Euler tiến)
    // Công thức: i_hat_new = i_hat_old + (Ts / Ls) * (u - Rs * i_hat - z)
    smo->i_hat_alpha += smo->ts_over_ls * (u_alpha - (smo->rs * smo->i_hat_alpha) - smo->z_alpha);
    smo->i_hat_beta  += smo->ts_over_ls * (u_beta  - (smo->rs * smo->i_hat_beta)  - smo->z_beta);
}
void FOC_SMO_ClearFault(FOC_SMO_t *smo)
{
    smo->input_fault = false;
}
