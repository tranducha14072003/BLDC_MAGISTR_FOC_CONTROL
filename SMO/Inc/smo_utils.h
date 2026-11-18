#ifndef smo_utils_h_
#define smo_utils_h_

#include <stdint.h>
#include <stdbool.h>
#include "foc_lpf.h"
#include "foc_math.h"
#include <math.h>

#ifndef f_hz_test
#define f_hz_test 500.0f    // tạm thời, để chỉnh khi đo
#endif
typedef struct{
    // Thông số cơ bản của động cơ thực tế
    float rs;   //Điện trở stator
    float ls;   //Điện cảm stator (H)
    float ts;   //chu kì lấy mẫu
    float ts_over_ls;   // hằng số tính sẵn ts/ls tối uuw FPU
    float delta;
    float inv_delta;
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
    //Cờ báo an toàn kiểm tra giá trị hợp lệ;
    bool input_fault;
    bool is_initialized;
}FOC_SMO_t;
//Khởi tạo thông số ban đầu
void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts, float delta);
//Hàm tính toán phần dòng điện ước lượng
void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide);
//Hàm cập nhập tần số cắt LPF theo tốc độ
void FOC_SMO_SetEmfFilterSpeed(FOC_SMO_t *smo, float we, float ke, float wc_min);
//Hàm trích xuất góc theta_raw
float FOC_SMO_GetTheta(const FOC_SMO_t *smo, float we);
//Hàm xóa cờ lỗi reset để không phải gọi lại hàm Init
void FOC_SMO_ClearFault(FOC_SMO_t *smo);
#endif