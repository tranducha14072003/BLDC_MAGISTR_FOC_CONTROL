/*
 * test_foc_math.c
 * Kịch bản Unit Test trên PC dành riêng cho module FOC_math.c
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "FOC_math.h"

// Macro kiểm tra sai số số thực (Đủ 4 tham số: actual, expected, tol, msg)
#define ASSERT_FLOAT_EQ(actual, expected, tol, msg) \
    do { \
        if (fabsf((actual) - (expected)) > (tol)) { \
            printf("  [FAIL] %s: Thuc te = %.5f, Ky vong = %.5f\n", msg, (double)(actual), (double)(expected)); \
            return -1; \
        } else { \
            printf("  [PASS] %s (Gia tri = %.4f)\n", msg, (double)(actual)); \
        } \
    } while (0)

int main(void) {
    printf("=========================================================\n");
    printf("   UNIT TEST RIENG MODULE FOC_MATH TREN PC (NO MCU)      \n");
    printf("=========================================================\n\n");

    init_trig_lut();

    // -----------------------------------------------------------------
    // TEST 1: Kiểm tra hàm căn bậc hai fast_sqrt và lượng giác Sin/Cos, Atan2
    // -----------------------------------------------------------------
    printf("--- TEST 1: Kiem tra fast_sqrt, Sin/Cos va fast_atan2 ---\n");
    ASSERT_FLOAT_EQ(fast_sqrt(16.0f), 4.0f, 1e-4f, "fast_sqrt(16.0) = 4.0");
    ASSERT_FLOAT_EQ(fast_sqrt(-5.0f), 0.0f, 1e-4f, "fast_sqrt(-5.0) bao ve so am = 0.0");

    float s, c;
    float angle_test = 0.52359878f; // 30 độ (PI/6 rad)
    fast_sincos(angle_test, &s, &c);
    ASSERT_FLOAT_EQ(s, 0.5000f,  1e-3f, "sin(30 deg) = 0.5000");
    ASSERT_FLOAT_EQ(c, 0.8660f,  1e-3f, "cos(30 deg) = 0.8660");

    float atan_res = fast_atan2(1.0f, 1.0f); // Góc 45 độ (PI/4 ~ 0.7854 rad)
    ASSERT_FLOAT_EQ(atan_res, 0.7854f, 1e-3f, "fast_atan2(1.0, 1.0) = PI/4 rad");

    // -----------------------------------------------------------------
    // TEST 2: Kiểm tra chuỗi Clarke -> Park -> Inverse Park
    // -----------------------------------------------------------------
    printf("\n--- TEST 2: Kiem tra chuoi Clarke -> Park -> Inverse Park ---\n");
    float theta = 1.04719755f; // Góc 60 độ (PI/3 rad)
    float i_alpha, i_beta, id, iq, v_alpha_rec, v_beta_rec;

    fast_sincos(theta, &s, &c);
    clarke_transform(10.0f, -5.0f, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, s, c, &id, &iq);
    inverse_park_transform(id, iq, s, c, &v_alpha_rec, &v_beta_rec);

    ASSERT_FLOAT_EQ(v_alpha_rec, i_alpha, 1e-4f, "Khoi phuc truc Alpha sau Park + InvPark");
    ASSERT_FLOAT_EQ(v_beta_rec,  i_beta,  1e-4f, "Khoi phuc truc Beta sau Park + InvPark");

    // -----------------------------------------------------------------
    // TEST 3: Kiểm tra SVPWM (ARR = 850, Vbus = 24V) & Xuất file CSV
    // -----------------------------------------------------------------
    printf("\n--- TEST 3: Kiem tra khoi SVPWM (ARR = 850, Vbus = 24V) ---\n");
    uint32_t u, v, w;
    float v_dc = 24.0f;

    // 3a. Tại điện áp 0V -> Duty = 50% (CCR = 425)
    svpwm(0.0f, 0.0f, v_dc, 1.0f/v_dc, 850, &u, &v, &w);
    ASSERT_FLOAT_EQ((float)u, 425.0f, 1.0f, "SVPWM tai 0V -> CCR_U = 425 (50% Duty)");

    svpwm(100.0f, 0.0f, v_dc, 1.0f/v_dc, 850, &u, &v, &w);
    ASSERT_FLOAT_EQ((float)u, 778.0f, 1.0f, "SVPWM tai 0 deg (Day lom yen ngua) -> CCR_U = 778");
    // 3b. Tại góc 0 độ (valpha = 100V, vbeta = 0V) -> Đáy lõm yên ngựa (Duty = 91.57% -> CCR_U = 778)
    svpwm(86.60254f, 50.0f, v_dc, 1.0f/v_dc, 850, &u, &v, &w);
    ASSERT_FLOAT_EQ((float)u, 833.0f, 1.0f, "SVPWM bao hoa dinh tren -> CCR_U = 833 (98% Duty)");
    ASSERT_FLOAT_EQ((float)w,  17.0f, 1.0f, "SVPWM bao hoa dinh duoi -> CCR_W = 17  (2% Duty)");

    // Xuất 1 chu kỳ điện (360 độ) ra file CSV để vẽ đồ thị sóng yên ngựa
    FILE *fp = fopen("svpwm_wave.csv", "w");
    if (fp) {
        fprintf(fp, "Angle_Deg,CCR_U,CCR_V,CCR_W\n");
        for (int deg = 0; deg <= 360; deg += 2) {
            float rad = (float)deg * (3.14159265f / 180.0f);
            float sin_t, cos_t, va_t, vb_t;
            fast_sincos(rad, &sin_t, &cos_t);
            inverse_park_transform(0.0f, 12.0f, sin_t, cos_t, &va_t, &vb_t);
            svpwm(va_t, vb_t, v_dc, 1.0f/v_dc, 850, &u, &v, &w);
            fprintf(fp, "%d,%u,%u,%u\n", deg, u, v, w);
        }
        fclose(fp);
        printf("  [PASS] Da xuat 180 diem dang song yen ngua ra file 'svpwm_wave.csv'!\n");
    }

    printf("\n=========================================================\n");
    printf("   TAT CA BAI TEST FOC_MATH THANH CONG 100%% (0 ERRORS)! \n");
    printf("=========================================================\n");
    return 0;
}
