#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "smo_utils.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { \
        g_pass++; \
    } else { \
        g_fail++; \
        printf("  [FAIL] %s (line %d)\n", (msg), __LINE__); \
    } \
} while (0)

static float wrap_pi(float a)
{
    while (a >  PI) { a -= TWO_PI; }
    while (a < -PI) { a += TWO_PI; }
    return a;
}

static bool approx_eq(float a, float b, float tol)
{
    return isfinite(a) && isfinite(b) && (fabsf(a - b) <= tol);
}

/* =========================================================================
 * TEST 1: Init, biên lỗi, NaN/Inf, k_slide <= 0/+Inf, GetTheta & LPF fault
 * ========================================================================= */
static void test_1_init_and_faults(void)
{
    printf("\n=== TEST 1: INIT & FAULT HANDLING ===\n");
    FOC_SMO_t smo;

    // 1.1 Init sai tham số
    FOC_SMO_Init(&smo, -0.1f, 1e-4f, 1e-5f);
    CHECK(!smo.is_initialized && smo.input_fault, "Init Rs < 0 phai bao loi");

    FOC_SMO_Init(&smo, 0.0f, 1e-4f, 1e-5f);
    CHECK(!smo.is_initialized && smo.input_fault, "Init Rs == 0 phai bao loi");

    FOC_SMO_Init(&smo, 0.1f, 0.0f, 1e-5f);
    CHECK(!smo.is_initialized && smo.input_fault, "Init Ls == 0 phai bao loi");

    FOC_SMO_Init(&smo, 0.1f, 1e-4f, NAN);
    CHECK(!smo.is_initialized && smo.input_fault, "Init Ts == NaN phai bao loi");

    // 1.2 Init chuẩn
    FOC_SMO_Init(&smo, 0.15f, 1.0e-4f, 1.0e-5f);
    CHECK(smo.is_initialized && !smo.input_fault && !smo.pll_initialized, "Init chuan phai thanh cong");
    CHECK(approx_eq(smo.ts_over_ls, 0.1f, 1e-6f), "ts_over_ls = Ts/Ls");
    CHECK(approx_eq(smo.c_lin, 10.0f, 1e-5f), "c_lin = Ls/Ts");

    // 1.3 Kiểm tra k_slide lỗi (0, âm, NaN, +Inf)
    smo.i_hat_alpha = 1.23f;
    FOC_SMO_UpdateCurrentObserver(&smo, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    CHECK(smo.input_fault && approx_eq(smo.i_hat_alpha, 1.23f, 1e-6f), "k_slide == 0 phai fault va giu state");
    FOC_SMO_ClearFault(&smo);

    FOC_SMO_UpdateCurrentObserver(&smo, 1.0f, 0.0f, 0.0f, 0.0f, -5.0f);
    CHECK(smo.input_fault && approx_eq(smo.i_hat_alpha, 1.23f, 1e-6f), "k_slide < 0 phai fault va giu state");
    FOC_SMO_ClearFault(&smo);

    FOC_SMO_UpdateCurrentObserver(&smo, NAN, 0.0f, 0.0f, 0.0f, 10.0f);
    CHECK(smo.input_fault && approx_eq(smo.i_hat_alpha, 1.23f, 1e-6f), "u_alpha == NaN phai fault va giu state");
    FOC_SMO_ClearFault(&smo);

    FOC_SMO_UpdateCurrentObserver(&smo, 1.0f, 0.0f, 0.0f, 0.0f, INFINITY);
    CHECK(smo.input_fault && approx_eq(smo.i_hat_alpha, 1.23f, 1e-6f),
          "k_slide == +Inf phai fault va giu state [BUG #1]");

    // 1.4 Kiểm tra GetTheta khi e=(0,0) và khi we=NaN
    FOC_SMO_Init(&smo, 0.15f, 1.0e-4f, 1.0e-5f);
    float th_guard = 1.2345f;
    CHECK(!FOC_SMO_GetTheta(&smo, 1000.0f, &th_guard) && approx_eq(th_guard, 1.2345f, 1e-6f),
          "GetTheta phai tra false va khong ghi de khi e=(0,0)");
    smo.e_alpha = 1.0f;
    smo.e_beta  = 0.0f;
    CHECK(!FOC_SMO_GetTheta(&smo, NAN, &th_guard) && approx_eq(th_guard, 1.2345f, 1e-6f),
          "GetTheta phai tra false va khong ghi de khi we=NaN");

    // 1.5 Kéo cờ lỗi từ LPF lên SMO trong SetEmfFilterSpeed
    FOC_SMO_Init(&smo, 0.15f, 1.0e-4f, 1.0e-5f);
    FOC_SMO_SetEmfFilterSpeed(&smo, 1.0e12f); // wc * ts = 2e7 > FOC_LPF_WCTS_MAX (1e6)
    CHECK(smo.lpf_ea.input_fault && smo.input_fault, "Loi LPF trong SetEmfFilterSpeed phai keo len smo.input_fault");
}

/* =========================================================================
 * TEST 2: Toán học Current Observer (Deadbeat + Adaptive LPF + GetTheta)
 *         Sinh (u, i) từ nghiệm giải tích chính xác của mạch R-L + Back-EMF
 * ========================================================================= */
static void run_smo_rotation_case(float omega, float lambda, float k_slide)
{
    const float rs    = 0.05f;
    const float ls    = 50.0e-6f; // 50 uH
    const float ts    = 1.0e-5f;  // 100 kHz
    const float i_amp = 5.0f;     // Dòng thuần trục q: 5 A

    FOC_SMO_t smo;
    FOC_SMO_Init(&smo, rs, ls, ts);
    FOC_SMO_SetEmfFilterSpeed(&smo, omega);

    const float E_true = fabsf(omega) * lambda;
    const int settle_n = 4000;
    const int meas_n   = 2000;

    double cz_re = 0.0, cz_im = 0.0;
    double ce_re = 0.0, ce_im = 0.0;
    double sum_th_err = 0.0;
    double max_th_err = 0.0;
    int    th_cnt     = 0;

    for (int k = 0; k < settle_n + meas_n; k++) {
        float t_k   = (float)k * ts;
        float t_mid = t_k + (0.5f * ts);
        float th_k  = omega * t_k;
        float th_m  = omega * t_mid;

        // Dòng tại t_k và điện áp trung bình trên [t_k, t_{k+1}] (tại midpoint t_k + Ts/2)
        float ia_k  = -i_amp * sinf(th_k);
        float ib_k  =  i_amp * cosf(th_k);

        float ea_m  = -omega * lambda * sinf(th_m);
        float eb_m  =  omega * lambda * cosf(th_m);
        float ia_m  = -i_amp * sinf(th_m);
        float ib_m  =  i_amp * cosf(th_m);
        float dia_m = -omega * i_amp * cosf(th_m);
        float dib_m = -omega * i_amp * sinf(th_m);

        float ua = (rs * ia_m) + (ls * dia_m) + ea_m;
        float ub = (rs * ib_m) + (ls * dib_m) + eb_m;

        FOC_SMO_UpdateCurrentObserver(&smo, ua, ub, ia_k, ib_k, k_slide);

        if (k >= settle_n) {
            // Tương quan trực giao (bình phương tối thiểu) đo gain và góc trễ pha của z và e
            double sg = (omega >= 0.0f) ? 1.0 : -1.0;
            double ux = -(double)sinf(th_k) * sg;
            double uy =  (double)cosf(th_k) * sg;
            double px = -uy * sg;
            double py =  ux * sg;

            double za = (double)smo.z_alpha, zb = (double)smo.z_beta;
            double ea = (double)smo.e_alpha, eb = (double)smo.e_beta;

            cz_re += (za * ux) + (zb * uy);
            cz_im += (za * px) + (zb * py);
            ce_re += (ea * ux) + (eb * uy);
            ce_im += (ea * px) + (eb * py);

            float th_est = 0.0f;
            if (FOC_SMO_GetTheta(&smo, omega, &th_est)) {
                // z[k] lấy từ e trung bình ở bước trước [t_{k-1}, t_k] -> ứng với mốc t_k - 0.5*Ts
                float th_ref = wrap_pi(th_k - (0.5f * omega * ts));
                double err   = (double)wrap_pi(th_est - th_ref);
                sum_th_err  += err;
                if (fabs(err) > max_th_err) { max_th_err = fabs(err); }
                th_cnt++;
            }
        }
    }

    double rad2deg      = 180.0 / (double)PI;
    double z_gain       = hypot(cz_re, cz_im) / ((double)meas_n * (double)E_true);
    double z_lag_deg    = -atan2(cz_im, cz_re) * rad2deg;
    double e_gain       = hypot(ce_re, ce_im) / ((double)meas_n * (double)E_true);
    double e_lag_deg    = -atan2(ce_im, ce_re) * rad2deg;
    double lpf_lag_deg  = (double)FOC_LPF_GetPhaseLagExact(&smo.lpf_ea, fabsf(omega)) * rad2deg;
    double z_theory_deg = 0.5 * (double)fabsf(omega) * (double)ts * rad2deg;
    double mean_err_deg = (th_cnt > 0) ? ((sum_th_err / (double)th_cnt) * rad2deg) : 999.0;
    double max_err_deg  = max_th_err * rad2deg;

    printf("  w=%+8.1f | z_gain=%.4f z_lag=%5.2f deg (th=%5.2f) | e_gain=%.4f e_lag=%5.2f deg (z+lpf=%5.2f) | GetTheta err=%+.3f/%.3f deg\n",
           (double)omega, z_gain, z_lag_deg, z_theory_deg,
           e_gain, e_lag_deg, z_lag_deg + lpf_lag_deg, mean_err_deg, max_err_deg);

    CHECK(!smo.input_fault, "SMO khong duoc fault khi quay hop le");
    CHECK(fabs(z_gain - 1.0) < 0.03, "Bien do z phai bam sat E_true (~0.99)");
    CHECK(fabs((e_lag_deg - z_lag_deg) - lpf_lag_deg) < 0.25,
          "Do tre pha LPF do duoc phai khop GetPhaseLagExact (<0.25 deg)");
    CHECK(max_err_deg < 0.7, "Sai so goc FOC_SMO_GetTheta sau bu xoay phai < 0.7 deg");
}

static void test_2_smo_rotation_and_gettheta(void)
{
    printf("\n=== TEST 2: SMO DEADBEAT + ADAPTIVE LPF + GET_THETA ===\n");
    const float lambda = 0.001f; // 1 mWb -> 36000 rad/s => |E| = 36 V
    run_smo_rotation_case(  2000.0f, lambda, 50.0f);
    run_smo_rotation_case( 10000.0f, lambda, 50.0f);
    run_smo_rotation_case( 36000.0f, lambda, 50.0f);
    run_smo_rotation_case(-10000.0f, lambda, 50.0f);
}

/* =========================================================================
 * TEST 3: Phase Detector (5.1) + Mutation Test (đổi dấu từng số hạng)
 * ========================================================================= */
static bool run_pd_check(float (*fn)(float, float, float, float, float), bool verbose)
{
    const float E = 5.0f, eps_e = 0.1f, th = 1.0f;
    const float ea = -E * sinf(th), eb = E * cosf(th);
    const float exp_01 = (sinf(0.1f) * E) / (E + eps_e);
    float s, c, e0, e_m, e_p, e_pi, e_pi_p, e_pi_m;

    fast_sincos(1.0f, &s, &c);              e0     = fn(ea, eb, s, c, eps_e);
    fast_sincos(0.9f, &s, &c);              e_m    = fn(ea, eb, s, c, eps_e);
    fast_sincos(1.1f, &s, &c);              e_p    = fn(ea, eb, s, c, eps_e);
    fast_sincos(1.0f + PI, &s, &c);         e_pi   = fn(ea, eb, s, c, eps_e);
    fast_sincos(1.0f + PI + 0.05f, &s, &c); e_pi_p = fn(ea, eb, s, c, eps_e);
    fast_sincos(1.0f + PI - 0.05f, &s, &c); e_pi_m = fn(ea, eb, s, c, eps_e);

    if (verbose) {
        printf("  PD(1.0)=%+.5f | PD(0.9)=%+.5f (exp %+.5f) | PD(1.1)=%+.5f | PD(1+pi)=%+.5f (+/-: %+.4f/%+.4f)\n",
               (double)e0, (double)e_m, (double)exp_01, (double)e_p,
               (double)e_pi, (double)e_pi_p, (double)e_pi_m);
    }
    return approx_eq(e0, 0.0f, 1e-4f) &&
           approx_eq(e_m, +exp_01, 1e-4f) &&
           approx_eq(e_p, -exp_01, 1e-4f) &&
           approx_eq(e_pi, 0.0f, 1e-4f) &&
           (e_pi_p > 0.01f) && (e_pi_m < -0.01f);
}

static float pd_real(float ea, float eb, float s, float c, float eps) { return pll_phase_detector(ea, eb, s, c, eps); }
static float pd_mut1(float ea, float eb, float s, float c, float eps) { return ((+ea * c) - (eb * s)) / (fast_sqrt((ea * ea) + (eb * eb)) + eps); }
static float pd_mut2(float ea, float eb, float s, float c, float eps) { return ((-ea * c) + (eb * s)) / (fast_sqrt((ea * ea) + (eb * eb)) + eps); }
static float pd_mut3(float ea, float eb, float s, float c, float eps) { return ((+ea * c) + (eb * s)) / (fast_sqrt((ea * ea) + (eb * eb)) + eps); }

static void test_3_phase_detector(void)
{
    printf("\n=== TEST 3: PHASE DETECTOR & MUTATION ===\n");
    CHECK(run_pd_check(pd_real, true), "pll_phase_detector phai khop ly thuyet");
    CHECK(!run_pd_check(pd_mut1, false), "Mutation 1 (+ea*c - eb*s) phai bi bat");
    CHECK(!run_pd_check(pd_mut2, false), "Mutation 2 (-ea*c + eb*s) phai bi bat");
    CHECK(!run_pd_check(pd_mut3, false), "Mutation 3 (+ea*c + eb*s) phai bi bat");
}

/* =========================================================================
 * TEST 4: QPLL (khóa tốc độ đều, nạp I handover, tăng tốc, quay ngược, Reset)
 * ========================================================================= */
static void test_4_pll_dynamics(void)
{
    printf("\n=== TEST 4: QPLL LOCK, HANDOVER, ACCEL, REVERSE & RESET ===\n");
    const float ts = 1e-5f, wn = 2000.0f, zeta = 0.7f, eps_e = 0.1f, we_max = 50000.0f;
    const float E  = 5.0f;
    const float rad2deg = 180.0f / PI;

    // 4.1 Khóa từ lệch 1.5 rad (chưa nạp I và có nạp I = omega)
    float omegas[3] = { 2000.0f, 10000.0f, 36000.0f };
    for (int i = 0; i < 3; i++) {
        for (int preload = 0; preload < 2; preload++) {
            FOC_SMO_t smo;
            FOC_SMO_Init(&smo, 0.1f, 1e-4f, ts);
            FOC_SMO_PLL_Init(&smo, wn, zeta, eps_e, we_max);

            float w = omegas[i];
            float th_true = 1.5f;
            if (preload) {
                FOC_SMO_PLL_Reset(&smo, 0.0f, w);
            }

            int lock_k = -1, consec = 0;
            float err_meas_deg = 0.0f, err_next_deg = 0.0f;
            for (int k = 0; k < 25000; k++) {
                float ea = -E * sinf(th_true);
                float eb =  E * cosf(th_true);
                FOC_SMO_PLL_Update(&smo, ea, eb);

                float em = wrap_pi(th_true - smo.theta_meas);
                float en = wrap_pi(th_true - smo.theta_pll);
                err_meas_deg = em * rad2deg;
                err_next_deg = en * rad2deg;

                if ((fabsf(em) < 0.01f) && (fabsf(smo.we_pll - w) < (0.02f * w))) {
                    consec++;
                    if ((consec >= 50) && (lock_k < 0)) { lock_k = k - 50; }
                } else {
                    consec = 0;
                }
                th_true += w * ts;
                if (th_true >= TWO_PI) { th_true -= TWO_PI; }
            }

            double lock_ms = (lock_k >= 0) ? ((double)lock_k * (double)ts * 1000.0) : -1.0;
            printf("  w=%7.1f preload=%d | lock=%6.2f ms | err(t_k)=%+7.4f deg | err(t_k+1)=%+7.3f deg | w_hat=%8.2f\n",
                   (double)w, preload, lock_ms, (double)err_meas_deg, (double)err_next_deg, (double)smo.we_pll);
            CHECK(lock_k >= 0, "PLL phai khoa duoc");
            CHECK(fabsf(err_meas_deg) < 0.02f, "Sai so goc tai moc do t_k phai -> 0");
            CHECK(fabsf(smo.we_pll - w) < 1.0f, "Toc do uoc luong we_pll phai -> omega");
        }
    }

    // 4.2 Tăng tốc đều alpha = 15000 rad/s^2 -> sai số hệ loại 2 = (alpha/wn^2) * (E + eps_e)/E
    {
        FOC_SMO_t smo;
        FOC_SMO_Init(&smo, 0.1f, 1e-4f, ts);
        FOC_SMO_PLL_Init(&smo, wn, zeta, eps_e, we_max);
        float w = 5000.0f, alpha = 15000.0f, th = 0.0f;
        FOC_SMO_PLL_Reset(&smo, th, w);

        float err_deg = 0.0f;
        for (int k = 0; k < 5000; k++) {
            float ea = -E * sinf(th);
            float eb =  E * cosf(th);
            FOC_SMO_PLL_Update(&smo, ea, eb);
            err_deg = wrap_pi(th - smo.theta_meas) * rad2deg;
            w += alpha * ts;
            th += w * ts;
            if (th >= TWO_PI) { th -= TWO_PI; }
        }
        float theory_deg = (alpha / (wn * wn)) * ((E + eps_e) / E) * rad2deg;
        printf("  Accel 15000 rad/s^2  | err(t_k)=%+.4f deg (theory %+.4f deg)\n",
               (double)err_deg, (double)theory_deg);
        CHECK(approx_eq(err_deg, theory_deg, 0.03f), "Sai so tang toc he loai 2 phai khop alpha/wn^2");
    }

    // 4.3 Quay ngược w = -5000 rad/s với lệch nhiễu ban đầu nhỏ (+0.05 rad)
    //     Cần nhiễu lệch khỏi 0 để kiểm chứng tính ổn định tại 0 hay nhảy sang 180 độ!
    {
        FOC_SMO_t smo;
        FOC_SMO_Init(&smo, 0.1f, 1e-4f, ts);
        FOC_SMO_PLL_Init(&smo, wn, zeta, eps_e, we_max);
        float w = -5000.0f, lambda = 0.001f, th = 1.0f;
        FOC_SMO_PLL_Reset(&smo, th + 0.05f, w);

        float err_deg = 0.0f;
        for (int k = 0; k < 5000; k++) {
            float ea = -w * lambda * sinf(th);
            float eb =  w * lambda * cosf(th);
            FOC_SMO_PLL_Update(&smo, ea, eb);
            err_deg = wrap_pi(th - smo.theta_meas) * rad2deg;
            th += w * ts;
            if (th < 0.0f) { th += TWO_PI; }
        }
        printf("  Reverse w=-5000 (+0.05 rad) | err(t_k)=%+7.2f deg | w_hat=%+8.2f\n",
               (double)err_deg, (double)smo.we_pll);
        CHECK(fabsf(err_deg) < 1.0f, "Quay nguoc w<0 phai bam dung goc 0 deg [BUG #5 - lech 180 deg]");
    }

    // 4.4 Kiểm tra biên Reset: theta0 > 2*PI và theta0 = -1e-9f (bẫy làm tròn float)
    {
        FOC_SMO_t smo;
        FOC_SMO_Init(&smo, 0.1f, 1e-4f, ts);
        FOC_SMO_PLL_Init(&smo, wn, zeta, eps_e, we_max);
        FOC_SMO_PLL_Reset(&smo, 6.5f, 2000.0f);
        CHECK(!smo.input_fault, "PLL_Reset voi theta0=6.5 rad nen wrap thay vi fault [BUG #3]");

        FOC_SMO_ClearFault(&smo);
        FOC_SMO_PLL_Reset(&smo, -1.0e-9f, 0.0f);
        CHECK(!smo.input_fault && (smo.theta_pll >= 0.0f) && (smo.theta_pll < TWO_PI),
              "pll_wrap_2pi(-1e-9f) phai nam trong [0, 2*PI) [BUG #4]");
    }
}

/* =========================================================================
 * TEST 5: FULL PIPELINE khép vòng (SMO -> Adaptive LPF -> QPLL -> Bù trễ pha)
 * ========================================================================= */
static void test_5_full_pipeline_smo_to_pll(void)
{
    printf("\n=== TEST 5: FULL PIPELINE (CURRENT OBS -> LPF -> QPLL -> PHASE COMP) ===\n");
    const float rs = 0.05f, ls = 50.0e-6f, ts = 1.0e-5f, lambda = 0.001f;
    const float omega = 10000.0f, k_slide = 40.0f, i_amp = 5.0f;

    FOC_SMO_t smo;
    FOC_SMO_Init(&smo, rs, ls, ts);
    FOC_SMO_PLL_Init(&smo, 2000.0f, 0.7f, 0.1f, 50000.0f);
    FOC_SMO_PLL_Reset(&smo, 0.2f, 9500.0f); // Handover từ tốc độ gần đúng

    float err_final_deg = 0.0f;
    for (int k = 0; k < 5000; k++) {
        float t_k   = (float)k * ts;
        float t_mid = t_k + (0.5f * ts);
        float th_k  = omega * t_k;
        float th_m  = omega * t_mid;

        float ia_k  = -i_amp * sinf(th_k);
        float ib_k  =  i_amp * cosf(th_k);
        float ea_m  = -omega * lambda * sinf(th_m);
        float eb_m  =  omega * lambda * cosf(th_m);
        float ia_m  = -i_amp * sinf(th_m);
        float ib_m  =  i_amp * cosf(th_m);
        float dia_m = -omega * i_amp * cosf(th_m);
        float dib_m = -omega * i_amp * sinf(th_m);

        float ua = (rs * ia_m) + (ls * dia_m) + ea_m;
        float ub = (rs * ib_m) + (ls * dib_m) + eb_m;

        FOC_SMO_SetEmfFilterSpeed(&smo, smo.we_pll);
        FOC_SMO_UpdateCurrentObserver(&smo, ua, ub, ia_k, ib_k, k_slide);
        FOC_SMO_PLL_Update(&smo, smo.e_alpha, smo.e_beta);

        // Bù trễ LPF + 0.5 mẫu của z về đúng mốc t_k
        float lpf_lag = FOC_LPF_GetPhaseLagExact(&smo.lpf_ea, smo.we_pll);
        float z_lag   = 0.5f * smo.we_pll * ts;
        float th_comp = wrap_pi(smo.theta_meas + lpf_lag + z_lag);

        err_final_deg = wrap_pi(th_comp - wrap_pi(th_k)) * (180.0f / PI);
    }

    printf("  Full closed SMO+LPF+PLL (w=10000) | we_pll=%8.2f | err_final(t_k)=%+.3f deg | fault=%d\n",
           (double)smo.we_pll, (double)err_final_deg, (int)smo.input_fault);
    CHECK(!smo.input_fault, "Full pipeline khong duoc fault");
    CHECK(fabsf(smo.we_pll - omega) < 5.0f, "we_pll phai hoi tu sat omega that");
    CHECK(fabsf(err_final_deg) < 0.5f, "Goc cuoi sau bu tre LPF + 0.5*Ts phai < 0.5 deg");
}

int main(void)
{
    printf("=================================================================\n");
    printf("   FULL UNIT & MATH VERIFICATION SUITE FOR SMO_UTILS (.h / .c)   \n");
    printf("=================================================================\n");

    test_1_init_and_faults();
    test_2_smo_rotation_and_gettheta();
    test_3_phase_detector();
    test_4_pll_dynamics();
    test_5_full_pipeline_smo_to_pll();

    printf("\n=================================================================\n");
    printf("  SUMMARY: PASS = %d | FAIL = %d\n", g_pass, g_fail);
    printf("=================================================================\n");
    return (g_fail == 0) ? 0 : 1;
}