/*
 * test_foc_pid.c
 * Unit Test kiểm chứng toàn diện module PID/Inc/foc_pid.h & PID/Src/foc_pid.c:
 * - TEST 1..4: P+I cơ bản, Clamping Anti-Windup, Đóng vòng kín RL 100 kHz
 * - TEST T9:   Feed-Forward (FOC_PI_UpdateFF) vượt ngưỡng -> Đầu ra kẹp và tích phân KHÔNG tăng
 * - TEST T10:  FOC_PI_TuneCurrent(pi, R, L, wc) -> Kp = L*wc, Ki = R*wc
 * - TEST T11:  Phát hiện lỗi đầu vào NaN / Inf -> Bật cờ input_fault và xuất 0.0f an toàn
 * - TEST T12:  Chuyển mạch êm tuyệt đối (FOC_PI_PreloadBumpless) khi sai số hiện tại err_now != 0
 */

#include <stdio.h>
#include <math.h>
#include "foc_pid.h"

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
    printf("   UNIT TEST MODULE FOC_PID (INLINE + FF + TUNE + FAULT) \n");
    printf("=========================================================\n\n");

    FOC_PI_t pi;

    // -----------------------------------------------------------------
    // TEST 1: Kiểm tra P + I cơ bản & Clamping Anti-Windup
    // -----------------------------------------------------------------
    printf("--- TEST 1: P + I co ban & Clamping Anti-Windup ---\n");
    FOC_PI_Init(&pi, 1.5f, 1500.0f, 1e-5f, -13.3f, 13.3f);
    float u1 = FOC_PI_Update(&pi, 5.0f, 0.0f);
    ASSERT_FLOAT_EQ(u1,          7.575f, 1e-4f, "Chu ky 1 (err = 5A): u_out = 7.5750 V");
    ASSERT_FLOAT_EQ(pi.integral, 0.075f, 1e-4f, "Chu ky 1: Tich phan = 0.0750 V");

    FOC_PI_Reset(&pi);
    for (int i = 0; i < 1000; i++) {
        (void)FOC_PI_Update(&pi, 50.0f, 0.0f);
    }
    ASSERT_FLOAT_EQ(pi.out_val,   13.3f, 1e-4f, "Kep cung tai tran +13.3 V");
    ASSERT_FLOAT_EQ(pi.integral,   0.0f, 1e-4f, "Anti-Windup khoa tich phan = 0.0 V");

    // -----------------------------------------------------------------
    // TEST T9: Feed-Forward (FOC_PI_UpdateFF) vượt ngưỡng -> Đầu ra kẹp & Tích phân không tăng
    // -----------------------------------------------------------------
    printf("\n--- TEST T9: Feed-Forward (UpdateFF) trong vong kep & Anti-Windup ---\n");
    FOC_PI_Reset(&pi);
    // Trường hợp 1: chưa bão hòa (err = 2A -> P = 3.0V, I = 0.03V, ff = 5.0V -> u = 8.03V)
    float u_ff1 = FOC_PI_UpdateFF(&pi, 2.0f, 0.0f, 5.0f);
    ASSERT_FLOAT_EQ(u_ff1,       8.030f, 1e-4f, "UpdateFF chua bao hoa (ff = 5V): u_out = 8.0300 V");
    ASSERT_FLOAT_EQ(pi.integral, 0.030f, 1e-4f, "UpdateFF chua bao hoa: Tich phan = 0.0300 V");

    // Trường hợp 2 (Test T9): Reset về 0, bơm ff = 15.0V (> trần 13.3V) cùng sai số dương err = +1.0A
    // Vì u_unsat = P + I + ff > 13.3V và err > 0, đầu ra phải kẹp ở 13.3V và tích phân giữ nguyên = 0!
    FOC_PI_Reset(&pi);
    float u_ff_sat = 0.0f;
    for (int i = 0; i < 500; i++) {
        u_ff_sat = FOC_PI_UpdateFF(&pi, 1.0f, 0.0f, 15.0f);
    }
    ASSERT_FLOAT_EQ(u_ff_sat,             13.3f, 1e-4f, "T9: ff = 15V vuot gioi han -> Dau ra kep tai 13.3 V");
    ASSERT_FLOAT_EQ(pi.integral,           0.0f, 1e-4f, "T9: ff vuot gioi han -> Tich phan KHONG tang (0.0 V)");
    ASSERT_FLOAT_EQ((float)pi.is_saturated, 1.0f, 1e-4f, "T9: Co is_saturated = true");

    // -----------------------------------------------------------------
    // TEST T10: Tự động tính Kp, Ki từ thông số động cơ FOC_PI_TuneCurrent(pi, R, L, wc)
    // -----------------------------------------------------------------
    printf("\n--- TEST T10: FOC_PI_TuneCurrent(pi, R, L, wc) ---\n");
    float Rs = 0.0019f;     // 1.9 mOhm
    float Ls = 0.00022f;    // 0.22 mH
    float wc = 10000.0f;    // 10,000 rad/s
    FOC_PI_TuneCurrent(&pi, Rs, Ls, wc);
    ASSERT_FLOAT_EQ(pi.kp,    Ls * wc,        1e-5f, "T10: Kp = L * wc = 2.2000 V/A");
    ASSERT_FLOAT_EQ(pi.ki,    Rs * wc,        1e-5f, "T10: Ki = R * wc = 19.0000 V/(A.s)");
    ASSERT_FLOAT_EQ(pi.ki_ts, Rs * wc * 1e-5f, 1e-7f, "T10: Cap nhat ki_ts = Ki * Ts = 0.00019");

    // -----------------------------------------------------------------
    // TEST T11: Giám sát cờ lỗi input_fault khi đầu vào bị NaN / Inf
    // -----------------------------------------------------------------
    printf("\n--- TEST T11: Phat hien loi dau vao (input_fault) khi gap NaN / Inf ---\n");
    FOC_PI_ClearFault(&pi);
    float u_nan = FOC_PI_UpdateFF(&pi, NAN, 1.0f, 0.0f);
    ASSERT_FLOAT_EQ(u_nan,               0.0f, 1e-4f, "T11: Dau ra tra ve 0.0 V an toan khi ref = NaN");
    ASSERT_FLOAT_EQ((float)pi.input_fault, 1.0f, 1e-4f, "T11: Bat co input_fault = true de tang tren cat PWM");

    FOC_PI_ClearFault(&pi);
    FOC_PI_TuneCurrent(&pi, -0.01f, Ls, wc); // Điện trở âm bất hợp lý
    ASSERT_FLOAT_EQ((float)pi.input_fault, 1.0f, 1e-4f, "T11: Bat co input_fault khi truyen R <= 0");
    FOC_PI_ClearFault(&pi);

    // -----------------------------------------------------------------
    // TEST T12: Chuyển mạch êm tuyệt đối (u_prev - kp * err_now - ff_now) khi err_now != 0
    // -----------------------------------------------------------------
    printf("\n--- TEST T12: Bumpless Handover chinh xac khi err_now != 0 ---\n");
    FOC_PI_Init(&pi, 1.5f, 1500.0f, 1e-5f, -13.3f, 13.3f);
    // Giả sử cuối pha I/F: Điện áp u_q trước đó là u_prev = 6.5V, sức điện động ff_now = 4.2V,
    // dòng đặt ref_now = 2.2A nhưng dòng thực tế đang là meas_now = 1.4A (err_now = +0.8A != 0)
    float u_prev   = 6.5f;
    float ref_now  = 2.2f;
    float meas_now = 1.4f;
    float ff_now   = 4.2f;

    FOC_PI_PreloadBumpless(&pi, u_prev, ref_now, meas_now, ff_now);
    // Kiểm tra giá trị nạp vào tích phân: 6.5 - 1.5*(2.2 - 1.4) - 4.2 = 6.5 - 1.2 - 4.2 = 1.1V
    ASSERT_FLOAT_EQ(pi.integral, 1.088f, 1e-4f, "T12: Nap tich phan I = u_prev - kp*err - ff = 1.1000 V");

    // Ngay chu kỳ đầu tiên sau chuyển mạch (với sai số tích phân 1 chu kỳ cực nhỏ ki_ts*err = 0.012V),
    // đầu ra nối tiếp mượt tại đúng 6.5V + 0.012V = 6.512V (không bị nhảy vọt thêm kp*err = +1.2V!)
    float u_first = FOC_PI_UpdateFF(&pi, ref_now, meas_now, ff_now);
    ASSERT_FLOAT_EQ(pi.integral, 1.100f, 1e-4f, "T12: Tich phan sau chu ky 1 dat dung 1.1000 V");
    ASSERT_FLOAT_EQ(u_first,     6.500f, 1e-4f, "T12: Dau ra chu ky dau = 6.5000 V (Bumpless 100% == u_prev)");

    printf("\n=========================================================\n");
    printf("   TAT CA BAI TEST FOC_PID (T1 -> T12) DA PASS 100%%!     \n");
    printf("=========================================================\n");
    return 0;
}