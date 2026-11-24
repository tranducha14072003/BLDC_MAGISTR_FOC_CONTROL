#ifndef FOC_SMO_H_
#define FOC_SMO_H_

#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "foc_lpf.h"
#include "foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
#define FOC_NODISCARD __attribute__((warn_unused_result))
#else
#define FOC_NODISCARD
#endif

// Trễ nội tại của tín hiệu z deadbeat (tâm ứng với t_k - 0.5*Ts)
#ifndef FOC_SMO_OBS_LAG_SAMPLES
#define FOC_SMO_OBS_LAG_SAMPLES  0.5f
#endif

typedef struct {
    // Thông số vật lý & hằng số tính sẵn
    float     rs;            // Điện trở stator (Ohm)
    float     ls;            // Điện cảm stator (H)
    float     ts;            // Chu kỳ lấy mẫu (s)
    float     ts_over_ls;    // Ts / Ls
    float     c_lin;         // Ls / Ts (độ lợi tuyến tính deadbeat)

    // Trạng thái Current Observer
    float     i_hat_alpha;   // Dòng ước lượng alpha (A)
    float     i_hat_beta;    // Dòng ước lượng beta (A)
    float     z_alpha;       // Tín hiệu trượt alpha (V)
    float     z_beta;        // Tín hiệu trượt beta (V)

    // Lọc thông thấp Back-EMF thích nghi
    FOC_LPF_t lpf_ea;
    FOC_LPF_t lpf_eb;
    float     e_alpha;       // Back-EMF sau lọc alpha (V)
    float     e_beta;        // Back-EMF sau lọc beta (V)
    float     ke;            // Hệ số trượt tần số cắt: wc = ke * |we|
    float     wc_min;        // Tần số cắt sàn tối thiểu (rad/s)

    // Cờ trạng thái
    bool      input_fault;
    bool      is_initialized;
} FOC_SMO_t;

void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts);

void FOC_SMO_SetEmfFilterSpeed(FOC_SMO_t *smo, float we);

void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo,float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide);

void FOC_SMO_ClearFault(FOC_SMO_t *smo);

// Rút góc tức thời từ (e_alpha, e_beta) bằng xoay bù pha + atan2 (đối chiếu / handover)
FOC_NODISCARD bool FOC_SMO_GetTheta(const FOC_SMO_t *smo, float we, float *theta_out);

// Tổng góc trễ pha (rad, có dấu theo we) của (e_alpha, e_beta) so với mốc đo dòng t_k
float FOC_SMO_GetTotalLag(const FOC_SMO_t *smo, float we);

#ifdef __cplusplus
}
#endif

#endif /* FOC_SMO_H_ */