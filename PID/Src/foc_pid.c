

#include "foc_pid.h"
#include <math.h>

/* Khởi tạo bộ điều khiển PI và kiểm tra tính hợp lệ của tham số.
Tham số sai: input_fault = true, hệ số = 0, giới hạn ±1 (đầu ra gần như vô hiệu).
*/
void FOC_PI_Init(FOC_PI_t *pi, float kp, float ki, float ts, float out_min, float out_max)
{
    pi->input_fault  = false;
    pi->is_saturated = false;
    pi->integral     = 0.0f;
    pi->out_val      = 0.0f;

    if (!isfinite(kp) || !isfinite(ki) || !isfinite(ts) ||
        !isfinite(out_min) || !isfinite(out_max) ||
        kp < 0.0f || ki < 0.0f || ts <= 0.0f || out_min >= out_max)
    {
        pi->input_fault = true;
        pi->kp          = 0.0f;
        pi->ki          = 0.0f;
        pi->ts          = 1e-5f;
        pi->ki_ts       = 0.0f;
        pi->out_min     = -1.0f;
        pi->out_max     =  1.0f;
        return;
    }

    pi->kp      = kp;
    pi->ki      = ki;
    pi->ts      = ts;
    pi->ki_ts   = ki * ts;
    pi->out_min = out_min;
    pi->out_max = out_max;
}

//Cập nhật hệ số Kp, Ki trực tuyến.
//Biến integral lưu theo đơn vị đầu ra (đã nhân Ki) nên đổi Ki không làm đầu ra nhảy.
//Đặt Ki = 0 sẽ xóa tích phân.
 
void FOC_PI_SetGains(FOC_PI_t *pi, float kp, float ki)
{
    if (!isfinite(kp) || !isfinite(ki) || kp < 0.0f || ki < 0.0f) {
        pi->input_fault = true;
        return;
    }

    pi->kp    = kp;
    pi->ki    = ki;
    pi->ki_ts = ki * pi->ts;

    if (pi->ki == 0.0f) {
        pi->integral = 0.0f;
    }
}

/*
Tự tính Kp, Ki cho vòng dòng điện từ thông số động cơ (khử cực - zero của tải R-L).
   Công thức: Kp = L * wc, Ki = R * wc
rs_ohm:   Điện trở pha stator Rs (Ohm)
ls_henry: Điện cảm pha stator Ls (Henry)
wc_rad_s: Băng thông vòng dòng mong muốn (rad/s, ví dụ 2*pi*1500).
Từ chối nếu wc*ts > 0.63 (wc > ~fs/10): trễ 1 chu kỳ PWM làm vòng mất ổn định.
Phải gọi sau FOC_PI_Init (cần ts hợp lệ).
*/
void FOC_PI_TuneCurrent(FOC_PI_t *pi, float rs_ohm, float ls_henry, float wc_rad_s)
{
    if (!isfinite(rs_ohm) || !isfinite(ls_henry) || !isfinite(wc_rad_s) ||
        rs_ohm <= 0.0f || ls_henry <= 0.0f || wc_rad_s <= 0.0f ||
        wc_rad_s * pi->ts > 0.63f)
    {
        pi->input_fault = true;
        return;
    }

    float kp = ls_henry * wc_rad_s;
    float ki = rs_ohm   * wc_rad_s;
    FOC_PI_SetGains(pi, kp, ki);
}


//Cập nhật giới hạn bão hòa đầu ra động (theo Vbus thực tế, vòng tròn điện áp).
// Nếu giới hạn mới hẹp hơn tích phân hiện tại thì tích phân bị kẹp lại.
void FOC_PI_SetOutputLimits(FOC_PI_t *pi, float out_min, float out_max)
{
    if (!isfinite(out_min) || !isfinite(out_max) || out_min >= out_max) {
        pi->input_fault = true;
        return;
    }

    pi->out_min = out_min;
    pi->out_max = out_max;

    if (pi->integral > pi->out_max) {
        pi->integral = pi->out_max;
    } else if (pi->integral < pi->out_min) {
        pi->integral = pi->out_min;
    }
}

// Xóa trạng thái tích phân và đầu ra về 0. KHÔNG xóa input_fault (dùng FOC_PI_ClearFault).
void FOC_PI_Reset(FOC_PI_t *pi)
{
    pi->integral     = 0.0f;
    pi->out_val      = 0.0f;
    pi->is_saturated = false;
}

void FOC_PI_ClearFault(FOC_PI_t *pi)
{
    pi->input_fault = false;
}

 //Nạp trực tiếp một giá trị vào biến tích phân (kẹp trong [out_min, out_max]).
 
void FOC_PI_PreloadIntegral(FOC_PI_t *pi, float preload_val)
{
    if (!isfinite(preload_val)) {
        pi->input_fault = true;
        return;
    }

    if (preload_val > pi->out_max) {
        pi->integral = pi->out_max;
    } else if (preload_val < pi->out_min) {
        pi->integral = pi->out_min;
    } else {
        pi->integral = preload_val;
    }
    pi->out_val = pi->integral;
}

/* Nạp tích phân để chuyển mạch êm (bumpless transfer), ví dụ khi chuyển I/F -> SMO:
 * I = u_prev - (Kp + Ki*Ts) * err_now - ff_now
 * Khi đó lần gọi FOC_PI_UpdateFF(pi, ref_now, meas_now, ff_now) kế tiếp cho đầu ra đúng bằng u_prev.
 * (Ki*Ts*err có mặt vì UpdateFF cộng tích phân TRƯỚC khi tính đầu ra.)
 * Nếu giá trị I cần nạp nằm ngoài [out_min, out_max] thì không thể chuyển mạch êm:
 * tích phân bị kẹp và out_val KHÔNG được đặt bằng u_prev.
 */
void FOC_PI_PreloadBumpless(FOC_PI_t *pi, float u_prev, float ref_now, float meas_now, float ff_now)
{
    if (!isfinite(u_prev) || !isfinite(ref_now) || !isfinite(meas_now) || !isfinite(ff_now)) {
        pi->input_fault = true;
        return;
    }

    float err_now    = ref_now - meas_now;
    float i_bumpless = u_prev - ((pi->kp + pi->ki_ts) * err_now) - ff_now;
    bool  in_range   = (i_bumpless <= pi->out_max) && (i_bumpless >= pi->out_min);

    FOC_PI_PreloadIntegral(pi, i_bumpless);
    if (in_range) {
        pi->out_val = u_prev;
    }
}