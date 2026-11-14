/* test_sensing.c - test FOC_Sensor_* ; ngưỡng lấy từ foc_config.h nên chạy được cả chế độ 24 V và 60 V. */
#include <stdio.h>
#include <math.h>
#include "foc_sensing.h"

static int fails = 0;
#define CHECK(name, cond) do { int ok_ = (cond); printf("  [%s] %s\n", ok_ ? "OK " : "LOI", name); if (!ok_) fails++; } while (0)

#define V_NORM      (0.5f * (FOC_MIN_VBUS_V + FOC_MAX_VBUS_V))   /* bus bình thường, giữa 2 ngưỡng */
#define V_OVER      (FOC_MAX_VBUS_V + 1.0f)                      /* vượt OVP 1 V */
#define V_UNDER     (FOC_MIN_VBUS_V - 3.0f)                      /* thấp hơn UVLO 3 V */
#define RAW_V(v)    ((uint16_t)((v) / ADC_TO_VBUS_FACTOR))
#define RAW_A(amp)  ((uint16_t)(2048 + (int)((amp) / ADC_TO_CURRENT_FACTOR)))   /* dòng -> raw, offset 2048 */

static void feed(FOC_Sensor_t *s, uint16_t a, uint16_t b, uint16_t v, int n)
{ for (int k = 0; k < n; k++) FOC_Sensor_Update_100kHz(s, a, b, v); }

static void good_calib(FOC_Sensor_t *s)
{ FOC_Sensor_Init(s); feed(s, 2048, 2048, RAW_V(V_NORM), OFFSET_CALIB_SAMPLES + 100); }

int main(void)
{
    FOC_Sensor_t s; int k;
    printf("== test foc_sensing: OCP %.0f A, OVP %.0f V, UVLO %.0f V (bus thu %.1f V) ==\n",
           (double)FOC_MAX_CURRENT_A, (double)FOC_MAX_VBUS_V, (double)FOC_MIN_VBUS_V, (double)V_NORM);

    printf("T1 hieu chuan binh thuong (jitter)\n");
    FOC_Sensor_Init(&s); unsigned long sa = 0, sb = 0;
    for (k = 0; k < 2000; k++) {
        uint16_t a = (uint16_t)(2048 + (k % 5) - 2), b = (uint16_t)(2050 + (k % 3) - 1);
        sa += a; sb += b; FOC_Sensor_Update_100kHz(&s, a, b, RAW_V(V_NORM));
    }
    CHECK("is_calibrated, khong loi", s.is_calibrated && !s.sensor_offset_fault);
    CHECK("offset_ia = trung binh (sai so < 1e-3)", fabs((double)s.offset_ia - (double)sa / 2000.0) < 1e-3);
    CHECK("offset_ib = trung binh (sai so < 1e-3)", fabs((double)s.offset_ib - (double)sb / 2000.0) < 1e-3);
    FOC_Sensor_Update_100kHz(&s, (uint16_t)(2048 + 20), 2050, RAW_V(V_NORM));
    CHECK("+20 count -> dung 20*ADC_TO_CURRENT_FACTOR A", fabsf(s.ia - 20.0f * ADC_TO_CURRENT_FACTOR) < 0.01f);
    CHECK("vbus do duoc ~ V_NORM (sai so 0.1 V)", fabsf(s.vbus - V_NORM) < 0.1f);

    printf("T2 cam bien hong (2 kenh = 700, 20000 mau)\n");
    FOC_Sensor_Init(&s); feed(&s, 700, 700, RAW_V(V_NORM), 20000);
    CHECK("sensor_offset_fault = 1", s.sensor_offset_fault);
    CHECK("is_calibrated van = 0",   !s.is_calibrated);

    printf("T3 OVP: bus nhay len %.0f V\n", (double)V_OVER);
    good_calib(&s); int n = 0;
    while (!s.over_voltage_fault && n < 1000) { FOC_Sensor_Update_100kHz(&s, 2048, 2048, RAW_V(V_OVER)); n++; }
    printf("     -> bao OVP sau %d mau = %d us\n", n, n * 10);
    CHECK("bao dung sau OVP_DEBOUNCE_SAMPLES mau", n == (int)OVP_DEBOUNCE_SAMPLES);

    printf("T4 nhieu 1 mau don le (khong duoc bao OVP)\n");
    good_calib(&s); FOC_Sensor_Update_100kHz(&s, 2048, 2048, RAW_V(V_OVER)); feed(&s, 2048, 2048, RAW_V(V_NORM), 100);
    CHECK("khong bao OVP", !s.over_voltage_fault);

    printf("T5 20 nhieu don le xen ke (khong lien tiep)\n");
    good_calib(&s);
    for (k = 0; k < 20; k++) { FOC_Sensor_Update_100kHz(&s, 2048, 2048, RAW_V(V_OVER)); FOC_Sensor_Update_100kHz(&s, 2048, 2048, RAW_V(V_NORM)); }
    CHECK("khong bao OVP", !s.over_voltage_fault);

    printf("T6 UVLO: bus rot xuong %.0f V, 5000 mau\n", (double)V_UNDER);
    good_calib(&s); feed(&s, 2048, 2048, RAW_V(V_UNDER), 5000);
    CHECK("under_voltage_fault = 1", s.under_voltage_fault);
    good_calib(&s); feed(&s, 2048, 2048, RAW_V(V_NORM), 5000);
    CHECK("bus binh thuong: khong bao UVLO", !s.under_voltage_fault);

    printf("T7 OCP: nguong %.0f A (thu duoi/tren 3 A)\n", (double)FOC_MAX_CURRENT_A);
    good_calib(&s); FOC_Sensor_Update_100kHz(&s, RAW_A(FOC_MAX_CURRENT_A - 3.0f), 2048, RAW_V(V_NORM));
    CHECK("duoi nguong 3 A: khong loi", !s.over_current_fault);
    good_calib(&s); FOC_Sensor_Update_100kHz(&s, RAW_A(FOC_MAX_CURRENT_A + 3.0f), 2048, RAW_V(V_NORM));
    CHECK("tren nguong 3 A: bao OCP", s.over_current_fault);
    good_calib(&s); FOC_Sensor_Update_100kHz(&s, RAW_A(0.625f * FOC_MAX_CURRENT_A), RAW_A(0.625f * FOC_MAX_CURRENT_A), RAW_V(V_NORM));
    CHECK("ia=ib=0.625*Imax -> |ic|=1.25*Imax: bao OCP qua kenh ic", s.over_current_fault);

    printf("T8 ADC cham bien\n");
    good_calib(&s); FOC_Sensor_Update_100kHz(&s, 4095, 2048, RAW_V(V_NORM)); CHECK("raw=4095 bao loi", s.over_current_fault);
    good_calib(&s); FOC_Sensor_Update_100kHz(&s, 2048, 5, RAW_V(V_NORM));    CHECK("raw=5 bao loi",    s.over_current_fault);

    printf("T9 ClearFaults sau loi offset, hieu chuan lai bang du lieu tot\n");
    FOC_Sensor_Init(&s); feed(&s, 700, 700, RAW_V(V_NORM), 3000); FOC_Sensor_ClearFaults(&s);
    CHECK("sau Clear: het loi, chua hieu chuan", !s.sensor_offset_fault && !s.is_calibrated && s.calib_cnt == 0U);
    feed(&s, 2048, 2048, RAW_V(V_NORM), 2000);
    CHECK("hieu chuan lai: offset = 2048", s.is_calibrated && s.offset_ia == 2048.0f && s.offset_ib == 2048.0f);

    printf("T10 ClearFaults khi KHONG loi offset giu nguyen hieu chuan\n");
    good_calib(&s); FOC_Sensor_Update_100kHz(&s, 4095, 2048, RAW_V(V_NORM)); FOC_Sensor_ClearFaults(&s);
    CHECK("van is_calibrated, offset 2048, het OCP", s.is_calibrated && s.offset_ia == 2048.0f && !s.over_current_fault);

    printf("\n=> %d kiem tra LOI\n", fails);
    return fails ? 1 : 0;
}