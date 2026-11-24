#include "smo_utils.h"

//Hàm này chỉ chạy một lần duy nhất 
void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts)
{

    if (smo == NULL) {
        return;
    }

    smo->input_fault    = false;
    smo->is_initialized = false;

    if (!isfinite(rs) || !isfinite(ls) || !isfinite(ts) || !(rs > 0.0f)  || !(ls > 0.0f)  || !(ts > 0.0f))
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
    smo->wc_min      = TWO_PI * 400.0f; // Đủ rộng cho tốc độ tối thiểu có EMF (~1300 rad/s)

    FOC_LPF_InitByWc(&smo->lpf_ea, smo->wc_min, smo->ts, 0.0f);
    FOC_LPF_InitByWc(&smo->lpf_eb, smo->wc_min, smo->ts, 0.0f);

    if (!smo->lpf_ea.is_initialized || !smo->lpf_eb.is_initialized) {
        smo->input_fault = true;
        return;
    }

    smo->is_initialized = true;
}

void FOC_SMO_SetEmfFilterSpeed(FOC_SMO_t *smo, float we)
{
    if ((smo == NULL) || !smo->is_initialized ||
        !isfinite(we) || !isfinite(smo->ke) || !isfinite(smo->wc_min) || !(smo->ke > 0.0f) || !(smo->wc_min > 0.0f))
    {
        if (smo != NULL) {
            smo->input_fault = true;
        }
        return;
    }

    float wc = smo->ke * fabsf(we);
    if (wc < smo->wc_min) {
        wc = smo->wc_min;
    }

    FOC_LPF_SetCutoffWc(&smo->lpf_ea, wc);
    FOC_LPF_SetCutoffWc(&smo->lpf_eb, wc);
    if (smo->lpf_ea.input_fault || smo->lpf_eb.input_fault) {
        smo->input_fault = true;
    }
}

void FOC_SMO_Update(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide)
{
    if ((smo == NULL) || !smo->is_initialized ||
        !isfinite(u_alpha) || !isfinite(u_beta) ||
        !isfinite(i_alpha) || !isfinite(i_beta) ||
        !isfinite(k_slide) || !(k_slide > 0.0f))
    {
        if (smo != NULL) {
            smo->input_fault = true;
        }
        return;
    }

    // 1. Sai số dòng điện = Ước lượng - Thực tế
    float err_a = smo->i_hat_alpha - i_alpha;
    float err_b = smo->i_hat_beta  - i_beta;

    // 2. Luật trượt deadbeat: z = clamp((Ls/Ts) * err, -k_slide, +k_slide)
    float za = smo->c_lin * err_a;
    if (za >  k_slide) { za =  k_slide; }
    if (za < -k_slide) { za = -k_slide; }
    smo->z_alpha = za;

    float zb = smo->c_lin * err_b;
    if (zb >  k_slide) { zb =  k_slide; }
    if (zb < -k_slide) { zb = -k_slide; }
    smo->z_beta = zb;

    // 3. Cập nhật dòng ước lượng (Euler tiến) & lọc Back-EMF qua LPF
    smo->i_hat_alpha += smo->ts_over_ls * (u_alpha - (smo->rs * smo->i_hat_alpha) - za);
    smo->i_hat_beta  += smo->ts_over_ls * (u_beta  - (smo->rs * smo->i_hat_beta)  - zb);

    smo->e_alpha = FOC_LPF_Update(&smo->lpf_ea, za);
    smo->e_beta  = FOC_LPF_Update(&smo->lpf_eb, zb);
}

void FOC_SMO_ClearFault(FOC_SMO_t *smo)
{
    if (smo == NULL) {
        return;
    }
    smo->input_fault = false;
    FOC_LPF_ClearFault(&smo->lpf_ea);
    FOC_LPF_ClearFault(&smo->lpf_eb);
}

bool FOC_SMO_GetTheta(const FOC_SMO_t *smo, float we, float *theta_out)
{
    if ((smo == NULL) || (theta_out == NULL) || !smo->is_initialized ||
        !isfinite(we) || !isfinite(smo->e_alpha) || !isfinite(smo->e_beta))
    {
        return false;
    }
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

float FOC_SMO_GetTotalLag(const FOC_SMO_t *smo, float we)
{
    if ((smo == NULL) || !smo->is_initialized || !isfinite(we)) {
        return 0.0f;
    }
    float lpf_lag = FOC_LPF_GetPhaseLagExact(&smo->lpf_ea, we);
    float obs_lag = FOC_SMO_OBS_LAG_SAMPLES * we * smo->ts;
    return lpf_lag + obs_lag;
}