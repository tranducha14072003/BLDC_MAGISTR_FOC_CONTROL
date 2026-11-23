#ifndef FOC_MATH_H
#define FOC_MATH_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
//Các hằng số toán học và lượng giác
#define PI 3.14159265358979323846f
#define SQRT3 1.73205080756887729353f
#define TWO_BY_SQRT3 1.15470053837925152902f
#define SQRT2 1.41421356237309504880f
#define SQRT2_INV 0.70710678118654752440f
#define SQRT3_DIV_2 0.86602540378443864676f
#define SQRT3_INV 0.57735026918962576451f
#define TWO_PI 6.28318530717958647692f
#define SVPWM_MAX_MODULATION    0.96f
#define SVPWM_LIMIT_COEFF       0.554256258f
#define CORDIC_CSR_COS_SIN_CONFIG  ((0U << CORDIC_CSR_FUNC_Pos)      | \
                                    (6U << CORDIC_CSR_PRECISION_Pos) | \
                                    (1U << CORDIC_CSR_NRES_Pos)      | \
                                    (0U << CORDIC_CSR_NARGS_Pos)     | \
                                    (0U << CORDIC_CSR_ARGSIZE_Pos)   | \
                                    (0U << CORDIC_CSR_RESSIZE_Pos))

#define CORDIC_CSR_ATAN2_CONFIG    ((2U << CORDIC_CSR_FUNC_Pos)      | \
                                    (6U << CORDIC_CSR_PRECISION_Pos) | \
                                    (0U << CORDIC_CSR_NRES_Pos)      | \
                                    (1U << CORDIC_CSR_NARGS_Pos)     | \
                                    (0U << CORDIC_CSR_ARGSIZE_Pos)   | \
                                    (0U << CORDIC_CSR_RESSIZE_Pos))

#define RAD_TO_Q31 683565275.576f
#define Q31_TO_FLOAT 4.656612873077392578125e-10f 
#define CORDIC_Q31_TO_RAD   1.462918079e-9f
//Marco giới hạn kẹp để ngăn chặn bị vọt lố hoặc tràn giới hạn Duty Cycle
#define CLAMP(x, min, max) ((x) <= (min) ? (min) : ((x) >= (max) ? (max) : (x)))
//Khai báo kiểu dữ liệu số phức(dành cho các phép biến đổi Clarke, Park, Inverse Clarke, Inverse Park)
typedef struct {
    float real; //Phần thực
    float imag; //Phần ảo
} complex_t;
//Chuyển đổi rad->degree và ngược lại
#define RAD_TO_DEG(rad) ((rad) * 57.29577951308232087680f)
#define DEG_TO_RAD(rad) ((rad) * 0.01745329251994329576f)

//Nguyên mẫu các hàm 
//Hàm khởi tạo bảng tra cứu lượng giác(Gọi 1 lần trong main)
void init_trig_lut(void);
//Chuẩn hóa góc quay về khoảng 0->2PI
void normalize_angle(float *theta);
//Hàm tra cứu sin/cos tốc độ cao thay thế sinf/cosf
float fast_sin(float theta);
float fast_cos(float theta);
//Tính nhanh sin(theta) và cos(theta) cùng lúc
void fast_sincos(float theta, float *sin_theta, float *cos_theta);
//Hàm căn bậc 2 và arctan tốc độ cao thay thế sqrtf/atan2f
float fast_sqrt(float x);
float fast_atan2(float y, float x);
//Hàm tính toán số phức cơ bản  
static inline void complex_sub(complex_t a, complex_t b, complex_t *result) 
{
    result->real = a.real - b.real;
    result->imag = a.imag - b.imag;
}
static inline void complex_mul(complex_t a, complex_t b, complex_t *result) 
{
    float r = a.real * b.real - a.imag * b.imag;
    float i = a.real * b.imag + a.imag * b.real;
    result->real = r;
    result->imag = i;
}   
static inline void complex_add(complex_t a, complex_t b, complex_t *result) 
{
    result->real = a.real + b.real;
    result->imag = a.imag + b.imag;
}
//Các hàm biến đổi Clarke, Park, Inverse Clarke, Inverse Park
void clarke_transform(float i_a, float i_b, float *i_alpha, float *i_beta);
void park_transform(float i_alpha, float i_beta, float sin_theta,float cos_theta, float *i_d, float *i_q);
//Hàm gộp chung clark và park để tối ưu hóa thời gian thực
void clarke_park_transform(float i_a, float i_b, float sin_theta,float cos_theta, float *i_d, float *i_q);
//Biến đổi ngược Inverse Transformace
void inverse_clarke_transform(float v_alpha, float v_beta, float *v_a, float *v_b,float *v_c);
void inverse_park_transform(float v_d, float v_q, float sin_theta,float cos_theta, float *v_alpha, float *v_beta);
//Phép biến đổi trực tiếp ba pha Va, Vb, Vc sang Vd-Vq
void Vabc_to_Vdq(float v_a, float v_b, float sin_theta,float cos_theta, float *V_d, float *V_q);
//Khối điều chế SVPWM
void svpwm(float v_alpha, float v_beta, float v_dc, float inv_vdc, uint32_t pwm_period, uint32_t *pwm_u, uint32_t *pwm_v, uint32_t *pwm_w);
#ifdef __cplusplus
}
#endif
#endif // FOC_MATH_H    