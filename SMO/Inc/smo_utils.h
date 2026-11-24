#ifndef SMO_UTILS_H
#define SMO_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include "foc_lpf.h"
#include "foc_math.h"
#include <math.h>
#include "foc_pid.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
#define FOC_NODISCARD __attribute__((warn_unused_result))
#else
#define FOC_NODISCARD
#endif

/* Độ trễ thời gian (tính theo số chu kì Ts) dùng để bù góc cho FOC
Mốc: theta_meas ứng với ê[k] ~ e(t_k - Ts/2) (sau khi bù trễ LPF)
Biến đổi Park cần góc tại t_k (lúc lấy mẫu dòng ia,ib):         +0,5Ts
Inverse Park cần góc tại center chu kì PWM kế tiếp,
t_k +1,5Ts (ADC tại t_k, CCR nạp ở t_{k+1}):                    +1,5Ts
Hai hằng số này phụ thuộc vào vị trí trigger ADC và thời điểm nạp CCR thật
*/
#define FOC_SMO_OBS_LAG_SAMPLES 0.5f
#define FOC_SMO_PWM_ADV_SAMPLES 1.5f

typedef struct{
    // Thông số cơ bản của động cơ thực tế
    float rs;   //Điện trở stator
    float ls;   //Điện cảm stator (H)
    float ts;   //chu kì lấy mẫu
    float ts_over_ls;   // hằng số tính sẵn ts/ls tối uuw FPU

    // Biến trạng thái ước lượng dòng điện (Current observer)
    float i_hat_alpha;
    float i_hat_beta;

    //Tín hiệu điều khiển trượt hay emf ước lượng
    float z_alpha;
    float z_beta;

    //Thêm hai bộ lọc cho tín hiệu bemf thu được từ SMO
    FOC_LPF_t lpf_ea;
    FOC_LPF_t lpf_eb;

    //Tín hiệu emf thô thu được sau lọc từ z chưa bù trễ
    float e_alpha; 
    float e_beta;

    //Hệ số tuyến tính ls/ts
    float c_lin;
    float ke;       //Hệ số tỉ lệ wc=ke.|we|
    float wc_min;   //wc nhỏ nhất: mức sàn cho động cơ khi tốc độ = 0
    /
    /Cờ báo an toàn kiểm tra giá trị hợp lệ;
    bool input_fault;
    bool is_initialized;

    //------Trường mới cho QPLL vòng khóa pha------
    FOC_PI_t pi_pll;          // Bộ PI của vòng khóa pha PLL
    float    wn;              // Băng thông góc tự nhiên (rad/s), vd: 2000
    float    zeta;            // Hệ số tắt dần, vd: 0.7
    float    eps_e;           // Hằng số chống chia 0 (V), vd: 0.1
    float    we_max;          // Giới hạn tốc độ góc điện đầu ra PI (rad/s)
    float    theta_pll;       // Trạng thái góc tích phân [0, 2*pi) -> ứng với t_{k+1}
    float    theta_meas;      // Góc tại mốc đo t_k (lưu trước khi tích phân)
    float    we_pll;          // Tốc độ góc điện đầu ra PI (rad/s)
    float    pll_err;         // Sai số pha eps từ Phase Detector (lưu để quan sát)
    bool     pll_initialized; //Cờ cấu hình hợp lệ của PLL

    //--Chiều quay, biên độ emf, cờ khóa ----
    float pll_dir;          // +1.0f và -1.0f: chiều quay quy ước của bộ tách pha
    float we_dir_hys;       // Ngưỡng "tích phân" để lật pll_dir
    uint16_t dir_count;     // Đếm mẫu liên tiếp tích phân vượt quá ngưỡng
    unit16_t dir_count_in;  // Cần bấy nhiêu mẫu liên tiếp mới lật
    float    e_mag;           // Độ lớn emf |e| (V), tính 1 lần mỗi mẫu
    float    e_mag;           // |ê| (V), tính 1 lần mỗi mẫu
    float    e_min_lock;      // Ngưỡng BẬT emf_ok (V)
    float    e_min_unlock;    // Ngưỡng TẮT emf_ok (V), < e_min_lock (hysteresis)
    bool     emf_ok;          // true: |ê| đủ lớn -> PLL được phép bám (cổng của eps)
    float    eps_lock_in;     // |eps| <= ngưỡng này (debounce) -> khóa
    float    eps_lock_out;    // |eps| >  ngưỡng này (debounce) -> mất khóa
    uint16_t lock_cnt;
    uint16_t lock_cnt_in;
    uint16_t lock_cnt_out;
    bool     pll_locked;      // emf_ok && |eps| nhỏ liên tục: góc đáng tin để dùng
 
    // ---- Lịch băng thông ----
    float    wn_ratio;        // wn_eff = min(wn, wn_ratio * wc_loc), mặc định 0.5
    float    wn_applied;      // wn đã nạp vào PI (tránh SetGains thừa mỗi mẫu)
}FOC_SMO_t;
//Khởi tạo thông số ban đầu
void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts);

//Hàm tính toán phần dòng điện ước lượng
void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide);

//Hàm cập nhập tần số cắt LPF theo tốc độ
void FOC_SMO_SetEmfFilterSpeed(FOC_SMO_t *smo, float we);

//Hàm trích xuất góc theta_raw
FOC_NODISCARD bool FOC_SMO_GetTheta(const FOC_SMO_t *smo, float we, float *theta_out);

//Hàm xóa cờ lỗi reset để không phải gọi lại hàm Init
void FOC_SMO_ClearFault(FOC_SMO_t *smo);

//Bộ tách pha vuông góc QPLL chuẩn hóa biên độ
static inline float FOC_PLL_PhaseDetector(float ea, float eb, float sin_t, float cos_t,float mag, float eps_e){
    return (-ea*cos_t - eb*sin_t) / (mag + eps_e);
}

//Hàm khởi tạo vòng khóa pha
void FOC_SMO_PLL_Init(FOC_SMO_t *smo, float wn, float zeta, float eps_e, float we_max);

//hàm cập nhập emf alpha và emf beta (gọi mỗi ISR, sau FOC_SMO_UpdateCurrentObserver)
void FOC_SMO_PLL_Update(FOC_SMO_t *smo, float ea, float eb);

/* nạp trạng thái PLL: theta0 = góc của ê thô, we0 = tốc độ đã biết */
void FOC_SMO_PLL_Reset(FOC_SMO_t *smo, float theta0, float we0);

//Đặt chiều quay theo lệnh (+1 và -1). Chỉ khi emf_ok == false( đang không bám), trả false nếu bị từ chối
bool FOC_SMO_SetDirection(FOC_SMO_t *smo, int dir);

//Đặt Offset ngưỡng giá trị biên độ (V) bật tắt emf_ok ,hợp lệ: 0 < off <on, hữu hạn 
bool FOC_SMO_PLL_SetEmfThresholds(FOC_SMO_t *smo, float on, float off);

//Đặt tham số tự lật chiều: ngưỡng tích phân (rad/s) và số mẫu liên tiếp , hợp lệ thì 0<we_dir_hys <= we_max , count >= 1
FOC_NODISCARD bool FOC_SMO_PLL_SetDirParams(FOC_SMO_t *smo, float we_dir_hys, uint16_t dir_cnt_in);

//Đặt tham số cờ khóa. Hợp lệ: 0 < eps_in < eps_out, cnt_in >= 1, cnt_out >= 1
FOC_NODISCARD bool FOC_SMO_PLL_SetLockParams(FOC_SMO_t *smo, float eps_in, float eps_out, uint16_t cnt_in, uint16_t cnt_out);

//Đặt chiều quay theo LỆNH (+1 / -1). Chỉ khi emf_ok == false (đang không bám); trả false nếu bị từ chối
FOC_NODISCARD bool FOC_SMO_PLL_SetDirection(FOC_SMO_t *smo, int dir);

//Xuất hai góc đã bù trễ cho vòng FOC. false nếu đối số/trạng thái không hợp lệ (khi đó *out không bị ghi)
FOC_NODISCARD bool FOC_SMO_PLL_GetCompensatedAngles(const FOC_SMO_t *smo, float *theta_park, float *theta_inv_park);

//Góc có đáng tin để dùng không (cho startup/handover)
static inline bool FOC_SMO_PLL_Locked(const FOC_SMO_t *smo){
    return(smo!=NULL) && smo->pll_initialized && !smo->input_fault && smo->pll_locked;
}
#ifdef __cplusplus
}
#endif
#endif