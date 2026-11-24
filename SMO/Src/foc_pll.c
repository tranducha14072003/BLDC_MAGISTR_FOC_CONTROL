#include "foc_pll.h"

// Chuẩn hóa góc về [0, 2*pi) trong O(1), an toàn khi angle thuộc [-2*pi, 4*pi)
static inline float pll_wrap_2pi(float angle)
{
    if (angle >= TWO_PI) {
        angle -= TWO_PI;
    } else if (angle < 0.0f) {
        angle += TWO_PI;
        if (angle >= TWO_PI) {
            angle = 0.0f;
        }
    }
    return angle;
}

static inline void pll_drop_lock(FOC_PLL_t *pll)
{
    pll->locked   = false;
    pll->lock_cnt = 0U;
    pll->dir_cnt  = 0U;
}

void FOC_PLL_Init(FOC_PLL_t *pll, float wn, float zeta, float eps_e, float we_max, float ts)
{
    if (pll == NULL) {
        return;
    }

    pll->is_initialized = false;
    pll->input_fault    = false;
    pll->emf_ok         = false;
    pll_drop_lock(pll);

    if (!isfinite(wn) || !isfinite(zeta) || !isfinite(eps_e) || !isfinite(we_max) || !isfinite(ts) ||
        !(wn > 0.0f)  || !(zeta > 0.0f)  || !(eps_e > 0.0f)  || !(we_max > 0.0f)  || !(ts > 0.0f) ||
        ((wn * ts) >= 0.5f) || ((we_max * ts) >= PI))
    {
        pll->input_fault = true;
        return;
    }

    FOC_PI_Init(&pll->pi, 2.0f * zeta * wn, wn * wn, ts, -we_max, we_max);
    if (pll->pi.input_fault) {
        pll->input_fault = true;
        return;
    }

    pll->ts           = ts;
    pll->wn           = wn;
    pll->zeta         = zeta;
    pll->eps_e        = eps_e;
    pll->we_max       = we_max;
    pll->wn_applied   = wn;
    pll->wn_ratio     = 0.5f;

    pll->theta_pll    = 0.0f;
    pll->theta_meas   = 0.0f;
    pll->we           = 0.0f;
    pll->err          = 0.0f;
    pll->e_mag        = 0.0f;

    pll->dir          = 1.0f;
    pll->e_min_lock   = 1.0f;
    pll->e_min_unlock = 0.6f;
    pll->we_dir_hys   = (800.0f < (0.5f * we_max)) ? 800.0f : (0.5f * we_max);
    pll->dir_cnt_in   = 200U;
    pll->eps_lock_in  = 0.1f;
    pll->eps_lock_out = 0.3f;
    pll->lock_cnt_in  = 50U;
    pll->lock_cnt_out = 50U;

    pll->is_initialized = true;
}

void FOC_PLL_Update(FOC_PLL_t *pll, float ea, float eb, float wc_lpf)
{
    if (pll == NULL) {
        return;
    }
    if (!pll->is_initialized || !isfinite(ea) || !isfinite(eb) || !isfinite(wc_lpf) || (wc_lpf < 0.0f)) {
        pll->input_fault = true;
        pll_drop_lock(pll);
        return;
    }

    float mag = fast_sqrt((ea * ea) + (eb * eb));
    if (!isfinite(mag)) {
        pll->input_fault = true;
        pll_drop_lock(pll);
        return;
    }

    // 0. Lịch băng thông: nếu có LPF ngoài (wc_lpf > 0), kẹp wn_eff <= wn_ratio * wc_lpf
    float wn_eff = pll->wn;
    if (wc_lpf > 0.0f) {
        float wn_lim = pll->wn_ratio * wc_lpf;
        if (wn_lim < wn_eff) {
            wn_eff = wn_lim;
        }
    }
    if (!(wn_eff > 0.0f) || !isfinite(wn_eff)) {
        pll->input_fault = true;
        pll_drop_lock(pll);
        return;
    }
    if (wn_eff != pll->wn_applied) {
        FOC_PI_SetGains(&pll->pi, 2.0f * pll->zeta * wn_eff, wn_eff * wn_eff);
        if (pll->pi.input_fault) {
            pll->input_fault = true;
            pll_drop_lock(pll);
            return;
        }
        pll->wn_applied = wn_eff;
    }

    // 1. Lưu góc mốc đo t_k trước khi tích phân
    pll->theta_meas = pll->theta_pll;
    pll->e_mag      = mag;

    // 2. Cổng biên độ EMF có hysteresis
    if (!pll->emf_ok) {
        if (mag > pll->e_min_lock)   { pll->emf_ok = true; }
    } else {
        if (mag < pll->e_min_unlock) { pll->emf_ok = false; }
    }

    // 3. Tách pha chuẩn hóa (ê yếu -> eps = 0 để giữ nguyên tích phân tốc độ)
    float eps = 0.0f;
    if (pll->emf_ok) {
        float s, c;
        fast_sincos(pll->theta_meas, &s, &c);
        eps = pll->dir * FOC_PLL_PhaseDetector(ea, eb, s, c, mag, pll->eps_e);
    }
    pll->err = eps;

    // 4. PI -> tốc độ điện we
    pll->we = FOC_PI_Update(&pll->pi, eps, 0.0f);
    if (pll->pi.input_fault) {
        pll->input_fault = true;
        pll_drop_lock(pll);
        return;
    }

    // 5. Tích phân góc sang mốc t_{k+1}
    pll->theta_pll = pll_wrap_2pi(pll->theta_meas + (pll->we * pll->ts));

    // 6. Lật chiều quay theo tích phân PI có debounce + dịch +PI cho cả 2 góc
    bool flipped = false;
    if (pll->emf_ok) {
        float we_i = pll->pi.integral;
        bool  want = ((pll->dir > 0.0f) && (we_i < -pll->we_dir_hys)) ||
                     ((pll->dir < 0.0f) && (we_i >  pll->we_dir_hys));
        if (want) {
            if (++pll->dir_cnt >= pll->dir_cnt_in) {
                pll->dir        = -pll->dir;
                pll->dir_cnt    = 0U;
                pll->theta_pll  = pll_wrap_2pi(pll->theta_pll  + PI);
                pll->theta_meas = pll_wrap_2pi(pll->theta_meas + PI);
                flipped         = true;
            }
        } else {
            pll->dir_cnt = 0U;
        }
    } else {
        pll->dir_cnt = 0U;
    }

    // 7. Cờ khóa pha (hysteresis + debounce)
    if (flipped || !pll->emf_ok) {
        pll->locked   = false;
        pll->lock_cnt = 0U;
    } else if (!pll->locked) {
        if (fabsf(eps) <= pll->eps_lock_in) {
            if (++pll->lock_cnt >= pll->lock_cnt_in) {
                pll->locked   = true;
                pll->lock_cnt = 0U;
            }
        } else {
            pll->lock_cnt = 0U;
        }
    } else {
        if (fabsf(eps) > pll->eps_lock_out) {
            if (++pll->lock_cnt >= pll->lock_cnt_out) {
                pll->locked   = false;
                pll->lock_cnt = 0U;
            }
        } else {
            pll->lock_cnt = 0U;
        }
    }
}

void FOC_PLL_Reset(FOC_PLL_t *pll, float theta0, float we0)
{
    if (pll == NULL) {
        return;
    }
    if (!pll->is_initialized || !isfinite(theta0) || !isfinite(we0) ||
        (fabsf(we0) > pll->we_max) || (fabsf(theta0) > 1.0e4f))
    {
        pll->input_fault = true;
        pll_drop_lock(pll);
        return;
    }

    float th = pll_wrap_2pi(fmodf(theta0, TWO_PI));
    FOC_PI_PreloadIntegral(&pll->pi, we0);
    if (pll->pi.input_fault) {
        pll->input_fault = true;
        pll_drop_lock(pll);
        return;
    }

    pll->theta_pll  = th;
    pll->theta_meas = th;
    pll->we         = we0;
    pll->err        = 0.0f;
    pll->e_mag      = 0.0f;
    pll->emf_ok     = false;
    pll_drop_lock(pll);

    if (we0 < 0.0f) {
        pll->dir = -1.0f;
    } else if (we0 > 0.0f) {
        pll->dir = 1.0f;
    }
}

void FOC_PLL_ClearFault(FOC_PLL_t *pll)
{
    if (pll == NULL) {
        return;
    }
    pll->input_fault = false;
    if (pll->is_initialized) {
        FOC_PI_ClearFault(&pll->pi);
    }
}

bool FOC_PLL_SetEmfThresholds(FOC_PLL_t *pll, float emf_on, float emf_off)
{
    if (pll == NULL) { return false; }
    if (!pll->is_initialized || !isfinite(emf_on) || !isfinite(emf_off) ||
        !(emf_off > 0.0f) || !(emf_on > emf_off))
    {
        pll->input_fault = true;
        return false;
    }
    pll->e_min_lock   = emf_on;
    pll->e_min_unlock = emf_off;
    return true;
}

bool FOC_PLL_SetDirParams(FOC_PLL_t *pll, float we_dir_hys, uint16_t dir_cnt_in)
{
    if (pll == NULL) { return false; }
    if (!pll->is_initialized || !isfinite(we_dir_hys) ||
        !(we_dir_hys > 0.0f) || !(we_dir_hys < pll->we_max) || (dir_cnt_in < 1U))
    {
        pll->input_fault = true;
        return false;
    }
    pll->we_dir_hys = we_dir_hys;
    pll->dir_cnt_in = dir_cnt_in;
    pll->dir_cnt    = 0U;
    return true;
}

bool FOC_PLL_SetLockParams(FOC_PLL_t *pll, float eps_in, float eps_out, uint16_t cnt_in, uint16_t cnt_out)
{
    if (pll == NULL) { return false; }
    if (!pll->is_initialized || !isfinite(eps_in) || !isfinite(eps_out) ||
        !(eps_in > 0.0f) || !(eps_out > eps_in) || (cnt_in < 1U) || (cnt_out < 1U))
    {
        pll->input_fault = true;
        return false;
    }
    pll->eps_lock_in  = eps_in;
    pll->eps_lock_out = eps_out;
    pll->lock_cnt_in  = cnt_in;
    pll->lock_cnt_out = cnt_out;
    pll->lock_cnt     = 0U;
    return true;
}

bool FOC_PLL_SetDirection(FOC_PLL_t *pll, int dir)
{
    if (pll == NULL) { return false; }
    if (!pll->is_initialized || ((dir != 1) && (dir != -1))) {
        pll->input_fault = true;
        return false;
    }
    if (pll->emf_ok) {
        return false;
    }
    pll->dir = (float)dir;
    pll_drop_lock(pll);
    return true;
}

bool FOC_PLL_GetAngles(const FOC_PLL_t *pll,
                       float input_lag_rad,
                       float *theta_park,
                       float *theta_inv_park)
{
    if ((pll == NULL) || (theta_park == NULL) || (theta_inv_park == NULL) ||
        !pll->is_initialized || !isfinite(input_lag_rad) ||
        !isfinite(pll->we) || !isfinite(pll->theta_meas))
    {
        return false;
    }

    float th_k      = pll_wrap_2pi(pll->theta_meas + input_lag_rad);
    float pwm_adv   = FOC_PLL_PWM_ADV_SAMPLES * pll->we * pll->ts;
    *theta_park     = th_k;
    *theta_inv_park = pll_wrap_2pi(th_k + pwm_adv);
    return true;
}