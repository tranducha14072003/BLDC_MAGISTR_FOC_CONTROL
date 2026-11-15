#include "foc_math.h"
#include <math.h>

//Cấp phát bộ nhớ Ram cho bảng tra cứu lượng giác
#ifndef PC_UNIT_TEST
#include "stm32g4xx.h" //truy cập trực tiếp thanh ghi CORDIC
#endif
//1.BẢNG TRA CỨU LƯỢNG GIÁC LUT VÀ TOÁN HỌC TỐC ĐỘ CAO
void init_trig_lut(void) {
    #ifndef PC_UNIT_TEST
    // 1. Bật xung nhịp cho ngoại vi CORDIC và đọc trễ 2 chu kỳ để đảm bảo bus AHB1 đã mở
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;
    volatile uint32_t dummy_read = RCC->AHB1ENR;
    (void)dummy_read;
    // 2. Cấu hình tường minh thanh ghi điều khiển CORDIC->CSR
    CORDIC->CSR = (0U << CORDIC_CSR_FUNC_Pos)      | // FUNC = 0: Cosine (Trả về Cos trước, Sin sau)
                  (6U << CORDIC_CSR_PRECISION_Pos) | // PRECISION = 6: 24 vòng lặp (6 chu kỳ xung nhịp)
                  (0U << CORDIC_CSR_SCALE_Pos)     | // SCALE = 0: Hệ số tỉ lệ mặc định cho hàm lượng giác
                  (1U << CORDIC_CSR_NRES_Pos)      | // NRES = 1: BẮT BUỘC trả về 2 kết quả (Cos và Sin)
                  (0U << CORDIC_CSR_NARGS_Pos)     | // NARGS = 0: Chỉ nạp 1 tham số góc duy nhất
                  (0U << CORDIC_CSR_ARGSIZE_Pos)   | // ARGSIZE = 0: Đầu vào 32-bit Q1.31
                  (0U << CORDIC_CSR_RESSIZE_Pos);    // RESSIZE = 0: Đầu ra 32-bit Q1.31

    // 3. Mồi thử 1 phép tính tại góc 0 rad khi khởi động để nạp thanh ghi Modulus mặc định
    CORDIC->WDATA = 0;
    volatile int32_t cos_init = (int32_t)CORDIC->RDATA;
    volatile int32_t sin_init = (int32_t)CORDIC->RDATA;
    (void)cos_init;
    (void)sin_init;
    #endif
}
//Chuẩn hóa góc quay về khoảng 0->2PI tránh tràn
void normalize_angle(float *theta)
{
    if(!(fabsf(*theta) < 1000.0f)) {*theta = 0.0f;} return; // Nếu theta = 0.0f thì không cần chuẩn hóa
    while (*theta < 0.0f){ *theta += TWO_PI;}
    while (*theta >= TWO_PI){ *theta -= TWO_PI;}
}
void fast_sincos(float theta, float *sin_theta, float *cos_theta) {
    #ifndef PC_UNIT_TEST
    // Chuẩn hóa góc về khoảng -PI->PI
    while (theta > PI)   theta -= TWO_PI;
    while (theta < -PI)  theta += TWO_PI;
    // Chuyển đổi góc sang định dạng Q1.31
    int32_t theta_q31 = (int32_t)(theta * RAD_TO_Q31);
    // Nạp tham số vào thanh ghi CORDIC
    CORDIC->WDATA = theta_q31;
    // Đọc kết quả từ thanh ghi RDATA
    int32_t cos_q31 = (int32_t)CORDIC->RDATA; // Cosine trước
    int32_t sin_q31 = (int32_t)CORDIC->RDATA; // Sine sau
    // Chuyển đổi kết quả từ Q1.31 sang float
    *cos_theta = cos_q31 * Q31_TO_FLOAT;
    *sin_theta = sin_q31 * Q31_TO_FLOAT;
    #else
    // Chạy Unit Test trên PC
    *cos_theta = cosf(theta);
    *sin_theta = sinf(theta);
    #endif
}
float fast_sin(float theta){
    float sin,cos;
    fast_sincos(theta,&sin,&cos);
    return sin; 
}
float fast_cos(float theta) {
    float sin,cos;
    fast_sincos(theta,&sin,&cos);
    return cos; 
}
//Hàm sqrt() tốc độ cao thay thế sqrtf()
float fast_sqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    // Sử dụng hàm sqrtf() của thư viện math.h để tính căn bậc hai
    return __builtin_sqrtf(x);
}
//Hàm atan2 tốc độ cao thay thế atan2f
float fast_atan2(float y, float x) 
{
    #ifndef PC_UNIT_TEST
    float abs_x = fabsf(x);
    float abs_y = fabsf(y);
    float max_val = (abs_x > abs_y) ? abs_x : abs_y;

    // 1. Bảo vệ tránh chia cho 0 khi cả x và y đều quá nhỏ
    if (max_val < 1e-6f) 
    {
        return 0.0f;
    }
    // 2. Chuẩn hóa Vector (x, y) về dải Q1.31 [-1, 1) để đạt độ chính xác bit cao nhất
    float inv_max = 2147483647.0f / max_val;
    int32_t q31_x = (int32_t)(x * inv_max);
    int32_t q31_y = (int32_t)(y * inv_max);
    // 3. Chuyển CORDIC sang chế độ PHASE (FUNC = 2, NARGS = 1, NRES = 0: Chỉ lấy 1 kết quả Góc)
    CORDIC->CSR = CORDIC_CSR_ATAN2_CONFIG;
    // 4. Nạp 2 tham số vào WDATA: Bắt buộc X (Cos/Real) trước, Y (Sin/Imag) sau
    CORDIC->WDATA = (uint32_t)q31_x;
    CORDIC->WDATA = (uint32_t)q31_y;
    // 5. Đọc kết quả góc Q1.31 từ RDATA (Với NRES = 0, đọc 1 lần là giải phóng xong CORDIC)
    int32_t q31_angle = (int32_t)CORDIC->RDATA;
    // 6. QUAN TRỌNG: Trả ngay CORDIC về chế độ COSINE/SINE mặc định cho hàm pre_calc_sin_cos()
    CORDIC->CSR = CORDIC_CSR_COS_SIN_CONFIG;
    // 7. Quy đổi từ Q1.31 sang Radian [-PI, PI] chỉ bằng 1 phép nhân FPU
    return (float)q31_angle * CORDIC_Q31_TO_RAD;
    #else
    if (fabsf(x) < 1e-6f && fabsf(y) < 1e-6f) return 0.0f;
    return atan2f(y, x);
    #endif
}
// hàm biến đổi Clark transform từ a-b-c sang alpha-beta
void clarke_transform(float i_a, float i_b, float *i_alpha, float *i_beta)
{
    *i_alpha = i_a;
    *i_beta = TWO_BY_SQRT3 * i_b + SQRT3_INV* i_a;
}   
// hàm biến đổi Park transform từ alpha-beta sang d-q
void park_transform(float i_alpha, float i_beta, float sin_theta,float cos_theta, float *i_d, float *i_q){
    *i_d = i_alpha * cos_theta + i_beta * sin_theta;
    *i_q = -i_alpha * sin_theta + i_beta * cos_theta;
}
//Hàm gộp chung clark và park để tối ưu hóa thời gian thực
void clarke_park_transform(float i_a, float i_b,float sin_theta, float cos_theta, float *i_d, float *i_q)
{
    float i_alpha = i_a;
    float i_beta = TWO_BY_SQRT3 * i_b + SQRT3_INV* i_a;

    *i_d = i_alpha * cos_theta + i_beta * sin_theta;
    *i_q = -i_alpha * sin_theta + i_beta * cos_theta;
}
//Hàm inverse Clarke transform từ alpha-beta sang a-b-c
void inverse_clarke_transform(float v_alpha, float v_beta, float *v_a, float *v_b,float *v_c)
{
    *v_a = v_alpha;
    *v_b = -0.5f * v_alpha + SQRT3_DIV_2 * v_beta;
    *v_c = -0.5f * v_alpha - SQRT3_DIV_2 * v_beta;
}
//Hàm inverse Park transform từ d-q sang alpha-beta
void inverse_park_transform(float v_d, float v_q, float sin_theta,float cos_theta, float *v_alpha, float *v_beta)
{
    *v_alpha = v_d * cos_theta - v_q * sin_theta;
    *v_beta = v_d * sin_theta + v_q * cos_theta;
}
//Hàm biến đổi trực tiếp từ ba pha Va, Vb, Vc sang Vd-Vq
void Vabc_to_Vdq(float v_a, float v_b, float sin_theta,float cos_theta, float *v_d, float *v_q)
{
    // Clarke transform
    float v_alpha = v_a;
    float v_beta = TWO_BY_SQRT3 * v_b + SQRT3_INV* v_a;
    // Park transform
    *v_d = v_alpha * cos_theta + v_beta * sin_theta;
    *v_q = -v_alpha * sin_theta + v_beta * cos_theta;
}

  void svpwm(float v_alpha, float v_beta, float v_dc, float inv_vdc, uint32_t pwm_period, uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w)
{
    if (v_dc < 1.0f) {
        uint32_t half_period = pwm_period >> 1;
        *pwm_u = *pwm_v = *pwm_w = half_period;
        return;
    }
    float v_max = v_dc * SVPWM_LIMIT_COEFF; // Giới hạn điện áp điều chế để tránh tràn (0.96)
    float v_max_sq = v_max * v_max;
    float vmag_sq  = v_alpha * v_alpha + v_beta * v_beta;
    
    if (vmag_sq > v_max_sq) {
        float scale = v_max / fast_sqrt(vmag_sq);
        v_alpha *= scale;
        v_beta  *= scale;
    }
    float v_a, v_b, v_c;
    inverse_clarke_transform(v_alpha, v_beta, &v_a, &v_b, &v_c);
    float v_phase_max = v_a;
    float v_phase_min = v_a;
    if (v_b > v_phase_max) v_phase_max = v_b;
    if (v_c > v_phase_max) v_phase_max = v_c;
    if (v_b < v_phase_min) v_phase_min = v_b;
    if (v_c < v_phase_min) v_phase_min = v_c;
    float v_neutral = -0.5f * (v_phase_max + v_phase_min);
    float duty_u = CLAMP(((v_a + v_neutral) * inv_vdc) + 0.5f, 0.02f, 0.98f);
    float duty_v = CLAMP(((v_b + v_neutral) * inv_vdc) + 0.5f, 0.02f, 0.98f);
    float duty_w = CLAMP(((v_c + v_neutral) * inv_vdc) + 0.5f, 0.02f, 0.98f);
    *pwm_u = (uint32_t)(duty_u * (float)pwm_period);
    *pwm_v = (uint32_t)(duty_v * (float)pwm_period);
    *pwm_w = (uint32_t)(duty_w * (float)pwm_period);
}
