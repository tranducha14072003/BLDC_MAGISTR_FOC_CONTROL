#ifndef FOC_PLL_H_
#define FOC_PLL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "foc_math.h"
#include "foc_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
#define FOC_NODISCARD __attribute__((warn_unused_result))
#else
#define FOC_NODISCARD
#endif

// Trễ tác động PWM từ mốc đo dòng t_k đến tâm chu kỳ PWM kế tiếp [t_{k+1}, t_{k+2}]
#ifndef FOC_PLL_PWM_ADV_SAMPLES
#define FOC_PLL_PWM_ADV_SAMPLES  1.5f
#endif

typedef struct {
    // Khối PI & thông số băng thông
    FOC_PI_t pi;
    float    ts;
    float    wn;             // Băng thông mục tiêu (rad/s)
    float    zeta;           // Hệ số tắt dần (~0.7)
    float    eps_e;          // Hằng số chống chia 0 (V)
    float    we_max;         // Giới hạn tốc độ điện (rad/s)
    float    wn_applied;     // Băng thông đang nạp trong PI
    float    wn_ratio;       // Giới hạn wn_eff <= wn_ratio * wc_lpf (mặc định 0.5)

    // Trạng thái góc & tốc độ
    float    theta_pll;      // Góc tích phân [0, 2*pi) ứng với mốc t_{k+1}
    float    theta_meas;     // Góc tại đúng mốc đo t_k (trước tích phân)
    float    we;             // Tốc độ góc điện đầu ra PI (rad/s)
    float    err;            // Sai số pha chuẩn hóa eps (rad)
    float    e_mag;          // Biên độ |e| = sqrt(ea^2 + eb^2) (V)

    // Cổng EMF, chiều quay & cờ khóa pha
    float    dir;            // +1.0f (thuận) hoặc -1.0f (ngược)
    float    e_min_lock;     // Ngưỡng bật emf_ok (V)
    float    e_min_unlock;   // Ngưỡng tắt emf_ok (V)
    float    we_dir_hys;     // Ngưỡng tích phân tốc độ để lật chiều (rad/s)
    uint16_t dir_cnt_in;     // Debounce lật chiều (số mẫu)
    uint16_t dir_cnt;
    float    eps_lock_in;    // Ngưỡng |eps| vào khóa (rad)
    float    eps_lock_out;   // Ngưỡng |eps| nhả khóa (rad)
    uint16_t lock_cnt_in;    // Debounce vào khóa (số mẫu)
    uint16_t lock_cnt_out;   // Debounce nhả khóa (số mẫu)
    uint16_t lock_cnt;

    // Cờ trạng thái
    bool     emf_ok;
    bool     locked;
    bool     input_fault;
    bool     is_initialized;
} FOC_PLL_t;

// Bộ tách pha vuông góc chuẩn hóa (nhận mag đã tính sẵn để tránh gọi fast_sqrt 2 lần)
static inline float FOC_PLL_PhaseDetector(float ea, float eb,
                                          float sin_t, float cos_t,
                                          float mag, float eps_e)
{
    return ((-ea * cos_t) - (eb * sin_t)) / (mag + eps_e);
}

void FOC_PLL_Init(FOC_PLL_t *pll, float wn, float zeta, float eps_e, float we_max, float ts);

// Cập nhật PLL trong ngắt 100 kHz:
//   ea, eb : Back-EMF từ SMO (hoặc observer khác)
//   wc_lpf : Tần số cắt hiện tại của LPF đầu vào (rad/s) để giới hạn wn_eff <= wn_ratio * wc_lpf
//            (Truyền 0.0f nếu không dùng LPF ngoài -> giữ nguyên băng thông wn)
void FOC_PLL_Update(FOC_PLL_t *pll, float ea, float eb, float wc_lpf);

void FOC_PLL_Reset(FOC_PLL_t *pll, float theta0, float we0);
void FOC_PLL_ClearFault(FOC_PLL_t *pll);

bool FOC_PLL_SetEmfThresholds(FOC_PLL_t *pll, float emf_on, float emf_off);
bool FOC_PLL_SetDirParams(FOC_PLL_t *pll, float we_dir_hys, uint16_t dir_cnt_in);
bool FOC_PLL_SetLockParams(FOC_PLL_t *pll, float eps_in, float eps_out, uint16_t cnt_in, uint16_t cnt_out);
bool FOC_PLL_SetDirection(FOC_PLL_t *pll, int dir);

// Xuất góc bù trễ cho Park (t_k) và Inverse Park (t_k + 1.5*Ts)
FOC_NODISCARD bool FOC_PLL_GetAngles(const FOC_PLL_t *pll,
                                     float input_lag_rad,
                                     float *theta_park,
                                     float *theta_inv_park);

#ifdef __cplusplus
}
#endif

#endif /* FOC_PLL_H_ */