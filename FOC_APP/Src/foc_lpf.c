
 //Thực thi các hàm khởi tạo, cài đặt tần số cắt và quản lý trạng thái
 // cho bộ lọc thông thấp bậc 1 (FOC_LPF_t)
#include "foc_lpf.h"
// Hàm nội bộ đặt lại toàn bộ struct về trạng thái lỗi an toàn (Fail-Safe)
static void lpf_invalidate(FOC_LPF_t *lpf)
{
    lpf->input_fault    = true;
    lpf->alpha          = 0.0f;
    lpf->wc_rad_s       = 0.0f;
    lpf->ts             = 1e-5f;
    lpf->out_val        = 0.0f;
    lpf->is_initialized = false;
}
 //Khởi tạo bộ lọc theo tần số góc wc_rad_s (rad/s)
void FOC_LPF_InitByWc(FOC_LPF_t *lpf, float wc_rad_s, float ts, float init_val)
{
    lpf->input_fault = false;
    float wc_ts      = wc_rad_s * ts;
    if (!isfinite(wc_rad_s) || !isfinite(ts) || !isfinite(init_val) ||
        (wc_rad_s <= 0.0f)  || (ts <= 0.0f)  || !FOC_LPF_WcTsValid(wc_ts))
    {
        lpf_invalidate(lpf);
        return;
    }
    lpf->ts             = ts;
    lpf->wc_rad_s       = wc_rad_s;
    lpf->alpha          = wc_ts / (1.0f + wc_ts);
    lpf->out_val        = init_val;
    lpf->is_initialized = true;
}


// Khởi tạo bộ lọc theo tần số cắt fc_hz (Hertz)
void FOC_LPF_Init(FOC_LPF_t *lpf, float fc_hz, float ts, float init_val)
{
    if (!isfinite(fc_hz) || (fc_hz <= 0.0f)) {
        lpf_invalidate(lpf);
        return;
    }
    FOC_LPF_InitByWc(lpf, TWO_PI * fc_hz, ts, init_val);
}
// Cập nhật tần số cắt theo wc_rad_s (Gọi hàm UpdateAlphaDynamic nội bộ)
void FOC_LPF_SetCutoffWc(FOC_LPF_t *lpf, float wc_rad_s)
{
    FOC_LPF_UpdateAlphaDynamic(lpf, wc_rad_s);
}
//Cập nhật tần số cắt theo fc_hz (Hertz)
void FOC_LPF_SetCutoffHz(FOC_LPF_t *lpf, float fc_hz)
{
    if (!isfinite(fc_hz) || (fc_hz <= 0.0f)) {
        lpf->input_fault = true;
        return;
    }
    FOC_LPF_SetCutoffWc(lpf, TWO_PI * fc_hz);
}
// Đặt lại giá trị đầu ra (Reset) khi dừng motor hoặc thay đổi trạng thái
void FOC_LPF_Reset(FOC_LPF_t *lpf, float init_val)
{
    if (!isfinite(init_val) || !lpf->is_initialized) {
        lpf->input_fault = true;
        return;
    }
    lpf->out_val = init_val;
}
//Xóa cờ lỗi đầu vào của bộ lọc
void FOC_LPF_ClearFault(FOC_LPF_t *lpf)
{
    lpf->input_fault = false;
}