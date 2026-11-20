#include "smo_utils.h"

//Hàm khởi tạo thông số cho SMO
void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts)
{
    if (smo == NULL) {
        return;
    }

    smo->input_fault     = false;
    smo->is_initialized  = false;
    smo->pll_initialized = false;

    if (!isfinite(rs) || !isfinite(ls) || !isfinite(ts) ||
        !(rs > 0.0f)  || !(ls > 0.0f)  || !(ts > 0.0f))
    {
        smo->input_fault = true;
        return;
    }

    float ts_over_ls = ts / ls;
    float c_lin      = ls / ts;
    if (!isfinite(ts_over_ls) || !isfinite(c_lin)) {
        smo->input_fault = true;
        return;
    }

    smo->rs          = rs;
    smo->ls          = ls;
    smo->ts          = ts;
    smo->ts_over_ls  = ts_over_ls;
    smo->c_lin       = c_lin;
    smo->i_hat_alpha = 0.0f;
    smo->i_hat_beta  = 0.0f;
    smo->z_alpha     = 0.0f;
    smo->z_beta      = 0.0f;
    smo->e_alpha     = 0.0f;
    smo->e_beta      = 0.0f;
    smo->ke          = 2.0f;
    smo->wc_min      = TWO_PI * 50.0f;

    FOC_LPF_InitByWc(&smo->lpf_ea, smo->wc_min, smo->ts, 0.0f);
    FOC_LPF_InitByWc(&smo->lpf_eb, smo->wc_min, smo->ts, 0.0f);
    if (!smo->lpf_ea.is_initialized || !smo->lpf_eb.is_initialized) {
        smo->input_fault = true;
        return;
    }

    smo->is_initialized = true;
}

void FOC_SMO_SetEmfFilterSpeed(FOC_SMO_t *smo, float we){
    if ((smo == NULL) || !smo->is_initialized ||
        !isfinite(we) || !isfinite(smo->ke) || !isfinite(smo->wc_min) ||
        !(smo->ke > 0.0f) || !(smo->wc_min > 0.0f))
    {
        if (smo != NULL) {
            smo->input_fault = true;
        }
        return;
    }
    float wc = smo->ke * fabsf(we);
    if(wc < smo->wc_min){
        wc = smo->wc_min; //Chống trượt về 0 khi động cơ đứng yên hoặc tua rất chậm
    }
    // Sử dụng các hàm có sẵn trong foc_lpf.h đã được test an toàn 100%
    FOC_LPF_SetCutoffWc(&smo->lpf_ea, wc);
    FOC_LPF_SetCutoffWc(&smo->lpf_eb, wc);
    if (smo->lpf_ea.input_fault || smo->lpf_eb.input_fault) {
        smo->input_fault = true;
    }
}

bool FOC_SMO_GetTheta(const FOC_SMO_t *smo, float we, float *theta_out)
{
    if ((smo == NULL) || (theta_out == NULL) || !smo->is_initialized ||
        !isfinite(we) || !isfinite(smo->e_alpha) || !isfinite(smo->e_beta))
    {
        return false;
    }
    // Chặn trường hợp vector EMF bằng 0 (atan2(0, 0) trả 0.0f — một góc giả)
    if ((smo->e_alpha == 0.0f) && (smo->e_beta == 0.0f)) {
        return false;
    }
    float c, s;
    FOC_LPF_GetLagRotation(&smo->lpf_ea, we, &c, &s);
    float ea = (smo->e_alpha * c) - (smo->e_beta * s);
    float eb = (smo->e_alpha * s) + (smo->e_beta * c);
    float sg = (we >= 0.0f) ? 1.0f : -1.0f;
    *theta_out = fast_atan2(-ea * sg, eb * sg);
    return true;
}

void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide){
    //Bước 0: Kiểm tra tính hợp lệ của các tham số đầu vào
  if ((smo == NULL) || !smo->is_initialized ||
    !isfinite(u_alpha) || !isfinite(u_beta) ||
    !isfinite(i_alpha) || !isfinite(i_beta)  ||
    !(k_slide > 0.0f)||!isfinite(k_slide))
{
    if (smo != NULL) {
        smo->input_fault = true;
    }
    return; // Giữ nguyên i_hat, z, e cũ
}
    // Bước 1: Tính sai số dòng điện (Sai số = Ưu lượng - Thực tế)
    float error_alpha = smo->i_hat_alpha - i_alpha;
    float error_beta  = smo->i_hat_beta  - i_beta;
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
    if (smo == NULL) { 
        return; 
    }
    smo->input_fault = false;
    FOC_LPF_ClearFault(&smo->lpf_ea);  
    FOC_LPF_ClearFault(&smo->lpf_eb);
    FOC_PI_ClearFault(&smo->pi_pll);
}
// Chuẩn hóa góc về [0, 2*pi) với O(1) không dùng while vô hạn trong ISR
// (An toàn vì we_pll đã bị kẹp trong [-we_max, +we_max] và we_max * ts < 2*pi)
static inline float pll_wrap_2pi(float angle)
{
    if (angle >= TWO_PI) {
        angle -= TWO_PI;
    } else if (angle < 0.0f) {
        angle += TWO_PI;
        if(angle >= TWO_PI){
            angle = 0.0f;
        }
    }
    return angle;
}
void FOC_SMO_PLL_Init(FOC_SMO_t *smo, float wn, float zeta, float eps_e, float we_max)
{
    if (smo == NULL) {
        return;
    }
    smo->pll_initialized = false;
    // Cần SMO đã Init để có smo->ts hợp lệ; kiểm tra điều kiện rời rạc wn*ts < 0.5 và we_max*ts < PI
    if (!smo->is_initialized ||
        !isfinite(wn) || !isfinite(zeta) || !isfinite(eps_e) || !isfinite(we_max) ||
        !(wn > 0.0f)  || !(zeta > 0.0f)  || !(eps_e > 0.0f)  || !(we_max > 0.0f)  ||
        ((wn * smo->ts) >= 0.5f) || ((we_max * smo->ts) >= PI))
    {
        smo->input_fault = true;
        return;
    }
    smo->wn     = wn;
    smo->zeta   = zeta;
    smo->eps_e  = eps_e;
    smo->we_max = we_max;
    // Hệ loại 2 chuẩn: s^2 + Kp*s + Ki = 0 -> Kp = 2*zeta*wn, Ki = wn^2
    float kp = 2.0f * zeta * wn;
    float ki = wn * wn;
    FOC_PI_Init(&smo->pi_pll, kp, ki, smo->ts, -we_max, we_max);
    if (smo->pi_pll.input_fault) {
        smo->input_fault = true;
        return;
    }
    smo->theta_pll       = 0.0f;
    smo->theta_meas      = 0.0f;
    smo->we_pll          = 0.0f;
    smo->pll_err         = 0.0f;
    smo->pll_initialized = true;
}
void FOC_SMO_PLL_Update(FOC_SMO_t *smo, float ea, float eb)
{
    if ((smo == NULL) || !smo->pll_initialized || !isfinite(ea) || !isfinite(eb)) {
        if (smo != NULL) {
            smo->input_fault = true;
        }
        return;
    }
    // 1. Lưu góc tại đúng mốc đo t_k TRƯỚC khi tích phân
    smo->theta_meas = smo->theta_pll;
    // 2. Tách pha chuẩn hóa bằng sin/cos của góc hiện tại
    float s, c;
    fast_sincos(smo->theta_meas, &s, &c);
    float eps    = pll_phase_detector(ea, eb, s, c, smo->eps_e);
    smo->pll_err = eps;
    // 3. Bộ PI giữ sai số pha về 0 -> đầu ra là tốc độ góc điện we_pll
    smo->we_pll = FOC_PI_Update(&smo->pi_pll, eps, 0.0f);
    if (smo->pi_pll.input_fault) {
        smo->input_fault = true;
        return;
    }
    // 4. Tích phân sang mốc t_{k+1} và chuẩn hóa về [0, 2*pi)
    smo->theta_pll = pll_wrap_2pi(smo->theta_meas + (smo->we_pll * smo->ts));
}
/* nạp trạng thái PLL: theta0 = góc của ê thô, we0 = tốc độ đã biết */
void FOC_SMO_PLL_Reset(FOC_SMO_t *smo, float theta0, float we0)
{
    if (smo == NULL) return;
    if (!smo->pll_initialized || !isfinite(theta0) || !isfinite(we0) ||
        (fabsf(we0) > smo->we_max) || (fabsf(theta0) > 1.0e4f))   /* lưới an toàn, tùy chọn */
    {
        smo->input_fault = true;
        return;
    }
    float th = pll_wrap_2pi(fmodf(theta0, TWO_PI));    /* tính trước, không đổi trạng thái */

    FOC_PI_PreloadIntegral(&smo->pi_pll, we0);          /* đổi trạng thái sau cùng */
    if (smo->pi_pll.input_fault) {
        smo->input_fault = true;
        return;
    }
    smo->theta_pll  = th;
    smo->theta_meas = th;
    smo->we_pll     = we0;
    smo->pll_err    = 0.0f;
}