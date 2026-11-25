#ifndef FOC_APP_H_
#define FOC_APP_H_

/*
 * foc_app.h - Chuỗi điều khiển FOC không cảm biến, KHÔNG phụ thuộc phần cứng (không HAL, không thanh ghi).
 * Một lần gọi FOC_App_Step() = một chu kỳ PWM (100 kHz), gọi từ ISR ADC. main.c chỉ còn: đọc ADC thô, gọi Step,
 * ghi 3 giá trị CCR và bật/tắt PWM theo out.pwm_enable.
 *
 * Thứ tự trong 1 bước (mốc t_k = lúc lấy mẫu dòng):
 *   1. FOC_Sensor_Update_100kHz      ADC thô -> ia, ib, vbus, cờ lỗi
 *   2. Clarke                        ia, ib -> i_alpha, i_beta
 *   3. SMO                           u = điện áp ĐÃ TÁC ĐỘNG trong [t_k, t_{k+1}] = điện áp tính ở bước k-1 (đệm 1 mẫu,
 *                                    dựng lại từ DUTY thực tế, không dùng giá trị trước SVPWM) -> e_alpha, e_beta
 *   4. PLL                           e -> we, theta (có kẹp băng thông theo LPF của SMO)
 *   5. Góc bù                        theta_park = góc tại t_k ; theta_inv = theta_park + 1.5*we*Ts (tâm PWM kế tiếp)
 *   6. FOC_Startup_Update            ALIGN/RAMP/HANDOVER/CLOSED, nhận (theta_park, we, locked) từ PLL
 *   7. Vòng tốc độ (10 kHz, chỉ CLOSED)
 *   8. Park -> FOC_Current_Update -> Park ngược (theta_inv) -> SVPWM -> duty
 * PLL chạy tự do từ ALIGN (EMF = 0 -> emf_ok = false -> ω̂ giữ 0); đo trên PC: tự kéo vào khi EMF vượt 1 V, khóa ~0.41 s,
 * gieo (Reset) PLL bằng góc/tốc độ I/F cho kết quả y hệt (đã thử và bỏ).
 *
 * An toàn: mọi lỗi (cảm biến, SMO/PLL, vòng dòng, khởi động) CHỐT lại, PWM tắt (pwm_enable = false, duty = giữa chu kỳ),
 * chỉ thoát bằng FOC_App_ClearFault. Thông số động cơ = 0 (chưa đo) -> FOC_App_Init trả false, KHÔNG có giá trị mặc định giả.
 */

#include <stdint.h>
#include <stdbool.h>
#include "foc_config.h"
#include "foc_math.h"
#include "foc_pid.h"
#include "foc_lpf.h"
#include "foc_sensing.h"
#include "foc_current.h"
#include "foc_startup.h"
#include "foc_smo.h"
#include "foc_pll.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FOC_NODISCARD
#if defined(__GNUC__) || defined(__clang__)
#define FOC_NODISCARD __attribute__((warn_unused_result))
#else
#define FOC_NODISCARD
#endif
#endif

typedef enum {
    FOC_APP_ERR_NONE = 0,
    FOC_APP_ERR_CONFIG,      // FOC_App_Init thất bại / chưa Init
    FOC_APP_ERR_SENSOR,      // OCP, OVP, UVLO, offset
    FOC_APP_ERR_OBSERVER,    // SMO hoặc PLL nhận đầu vào không hợp lệ
    FOC_APP_ERR_CURRENT,     // vòng dòng báo lỗi
    FOC_APP_ERR_STARTUP      // khởi động báo lỗi (xem FOC_App_StartupErr)
} FOC_AppErr_t;

typedef struct {
    // Thông số động cơ: PHẢI đo, = 0 là chưa đặt
    float rs, ls, flux;          // Ohm, H, Wb
    float ts;                    // s (FOC_TS)
    float current_bw_rad_s;      // băng thông vòng dòng
    // Quan sát
    float k_slide;               // V, kẹp của z (phải > biên độ EMF lớn nhất)
    float pll_wn, pll_zeta, pll_eps_e, we_max;
    // Vòng tốc độ (CLOSED)
    float speed_kp, speed_ki;    // A/(rad/s), A/(rad/s)/s
    float i_speed_max;           // A, kẹp iq
    float speed_accel;           // rad/s^2, dốc lệnh tốc độ
    FOC_StartupCfg_t startup;
} FOC_AppCfg_t;

typedef struct {
    uint32_t duty_u, duty_v, duty_w;   // đếm timer (0..PWM_PERIOD_CYCLES)
    bool     pwm_enable;
    FOC_StartupState_t state;
    FOC_AppErr_t       err;
} FOC_AppOut_t;

typedef struct {
    FOC_AppCfg_t   cfg;
    FOC_Sensor_t   sensor;
    FOC_SMO_t      smo;
    FOC_PLL_t      pll;
    FOC_Startup_t  st;
    FOC_Current_t  cur;
    FOC_PI_t       pi_w;

    float u_alpha_prev, u_beta_prev;   // điện áp tác động trong [t_k, t_{k+1}] (từ duty bước trước)
    float we_ref, we_ref_ramp;
    uint32_t speed_cnt;
    bool  was_closed;

    // Quan sát được (đọc ra để log/test)
    float theta_park, theta_inv, we_hat, lag;
    bool  obs_valid;
    float id_meas, iq_meas, vd, vq, iq_cmd;

    FOC_AppErr_t err;
    bool  is_initialized;
} FOC_App_t;

// Điền cấu hình mặc định (ts, vòng dòng, PLL, startup). rs/ls/flux/speed_kp/speed_ki để 0: người dùng PHẢI đặt.
void FOC_App_DefaultCfg(FOC_AppCfg_t *cfg);
// Kiểm tra cấu hình, khởi tạo mọi module. false: không dùng được (err = CONFIG).
FOC_NODISCARD bool FOC_App_Init(FOC_App_t *a, const FOC_AppCfg_t *cfg);
// Yêu cầu chạy: false nếu chưa Init, đang lỗi, chưa hiệu chuẩn offset, cảm biến lỗi, hoặc đang chạy.
FOC_NODISCARD bool FOC_App_Start(FOC_App_t *a);
// Tắt về IDLE (PWM tắt ở bước kế tiếp), không xóa lỗi.
void FOC_App_Stop(FOC_App_t *a);
// Xóa lỗi chốt, về IDLE. Lỗi CONFIG: phải Init lại.
void FOC_App_ClearFault(FOC_App_t *a);
// Lệnh tốc độ điện (rad/s, 0..we_max). false nếu vô lý (không đổi gì).
FOC_NODISCARD bool FOC_App_SetSpeedRef(FOC_App_t *a, float we_ref);
// Gọi mỗi chu kỳ PWM.
void FOC_App_Step(FOC_App_t *a, uint16_t raw_ia, uint16_t raw_ib, uint16_t raw_vbus, FOC_AppOut_t *out);

static inline FOC_StartupErr_t FOC_App_StartupErr(const FOC_App_t *a) { return a->st.err; }

#ifdef __cplusplus
}
#endif

#endif /* FOC_APP_H_ */