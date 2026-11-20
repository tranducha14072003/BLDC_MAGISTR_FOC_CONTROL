#ifndef smo_utils_h_
#define smo_utils_h_

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
    //Tín hiệu emf thô thu được sau lọc từ z
    float e_alpha;
    float e_beta;
    //Hệ số tuyến tính ls/ts
    float c_lin;
    float ke;
    float wc_min;
    //Cờ báo an toàn kiểm tra giá trị hợp lệ;
    bool input_fault;
    bool is_initialized;
    //
    FOC_PI_t pi_pll;
    float    wn;              // Băng thông góc tự nhiên (rad/s), vd: 2000
    float    zeta;            // Hệ số tắt dần, vd: 0.7
    float    eps_e;           // Hằng số chống chia 0 (V), vd: 0.1
    float    we_max;          // Giới hạn tốc độ góc điện đầu ra PI (rad/s)
    float    theta_pll;       // Trạng thái góc tích phân [0, 2*pi) -> ứng với t_{k+1}
    float    theta_meas;      // Góc tại mốc đo t_k (lưu trước khi tích phân)
    float    we_pll;          // Tốc độ góc điện đầu ra PI (rad/s)
    float    pll_err;         // Sai số pha eps từ Phase Detector (lưu để quan sát)
    bool     pll_initialized;
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
static inline float pll_phase_detector(float ea, float eb, float sin_t, float cos_t, float eps_e)
{
    float mag = fast_sqrt(ea*ea + eb*eb);
    return (-ea*cos_t - eb*sin_t) / (mag + eps_e);
}
//Hàm khởi tạo vòng khóa pha
void FOC_SMO_PLL_Init(FOC_SMO_t *smo, float wn, float zeta, float eps_e, float we_max);
//hàm cập nhập emf alpha và emf beta
void FOC_SMO_PLL_Update(FOC_SMO_t *smo, float ea, float eb);
/* nạp trạng thái PLL: theta0 = góc của ê thô, we0 = tốc độ đã biết */
void FOC_SMO_PLL_Reset(FOC_SMO_t *smo, float theta0, float we0);

#ifdef __cplusplus
}
#endif
#endif