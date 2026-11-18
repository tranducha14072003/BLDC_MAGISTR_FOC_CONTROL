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
    smo->rs          = rs;
    smo->ls          = ls;
    smo->ts          = ts;
    smo->ts_over_ls  = ts/ls;
    smo->c_lin       = ls/ts;
    smo->delta       = delta;
    smo->inv_delta   = 1.0f/delta;
    smo->i_hat_alpha = 0.0f;  
    smo->i_hat_beta  = 0.0f;
    smo->z_alpha     = 0.0f;  
    smo->z_beta      = 0.0f;
     //Cấu hình bộ lọc với tần số cắt cố định 500hz trước khi cấu hình phức tạp
    FOC_LPF_Init(&smo->lpf_ea, f_hz_test, smo->ts, 0.0f);
    FOC_LPF_Init(&smo->lpf_eb, f_hz_test, smo->ts, 0.0f);
    if (!smo->lpf_ea.is_initialized || !smo->lpf_eb.is_initialized) {
        smo->is_initialized = false; 
        smo->input_fault = true; return; 
    }
    smo->e_alpha     = 0.0f;
    smo->e_beta      = 0.0f;
    smo->is_initialized = true;
}

void FOC_SMO_SetEmfFilterSpeed(FOC_SMO_t *smo, float we, float ke, float wc_min){
    if(!isfinite(we)) return;
    float wc = ke * fabsf(we);
    if(wc < wc_min){
        wc = wc_min; //Chống trượt về 0 khi động cơ đứng yên hoặc tua rất chậm
    }
    // Sử dụng các hàm có sẵn trong foc_lpf.h đã được test an toàn 100%
    FOC_LPF_SetCutoffWc(&smo->lpf_ea, wc);
    FOC_LPF_SetCutoffWc(&smo->lpf_eb, wc);
}

float FOC_SMO_GetTheta(const FOC_SMO_t *smo, float we){
    if(!isfinite(we)) return 0.0f;
    float c,s;
    //Lấy trực tiếp vector sin/cos bù trễ pha từ foc_lpf
    FOC_LPF_GetLagRotation(&smo->lpf_ea,we, &c, &s);
    //Phép xoay vector để kéo vector EMF về đúng pha thực tế ko trễ
    float ea_prime = smo->e_alpha * c - smo->e_beta * s;
    float eb_prime = smo->e_alpha * s + smo->e_beta * c;
    // Đảo chiều dấu nếu động cơ quay ngược (we < 0)
    float sg = (we >= 0.0f) ? 1.0f : -1.0f;
    return fast_atan2(-ea_prime * sg, eb_prime * sg);
}

void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide){
    //Bước 0: Kiểm tra tính hợp lệ của các tham số đầu vào
    if (!smo->is_initialized || !isfinite(u_alpha) || !isfinite(u_beta) || !isfinite(i_alpha) || !isfinite(i_beta) || !isfinite(k_slide)){
    smo->input_fault = true;   // ← true, và KHÔNG sửa î (giữ trạng thái cũ)
    return;
}
    // Bước 1: Tính sai số dòng điện (Sai số = Ưu lượng - Thực tế)
    float error_alpha = smo->i_hat_alpha - i_alpha;
    float error_beta  = smo->i_hat_beta  - i_beta;
    // Bước 2: Đưa sai số qua hàm piecewise để lấy tín hiệu trượt mượt mà (z_alpha, z_beta)
    //Tính za
    float za = smo->c_lin * error_alpha;
    if (za >  k_slide) za =  k_slide;
    if (za < -k_slide) za = -k_slide;
    smo->z_alpha = za;
    // Làm hoàn toàn tương tự cho trục beta (zb)
    float zb = smo->c_lin * error_beta;
    if (zb >  k_slide) zb =  k_slide;
    if (zb < -k_slide) zb = -k_slide;
    smo->z_beta  = zb;
    // Bước 3: Cập nhật dòng điện ước lượng mới theo phương trình vi phân (Euler tiến)
    // Công thức: i_hat_new = i_hat_old + (Ts / Ls) * (u - Rs * i_hat - z)
    smo->i_hat_alpha += smo->ts_over_ls * (u_alpha - (smo->rs * smo->i_hat_alpha) - smo->z_alpha);
    smo->i_hat_beta  += smo->ts_over_ls * (u_beta  - (smo->rs * smo->i_hat_beta)  - smo->z_beta);
    smo->e_alpha = FOC_LPF_Update(&smo->lpf_ea, smo->z_alpha);
    smo->e_beta  = FOC_LPF_Update(&smo->lpf_eb, smo->z_beta);
}
void FOC_SMO_ClearFault(FOC_SMO_t *smo){
    smo->input_fault = false;
}
