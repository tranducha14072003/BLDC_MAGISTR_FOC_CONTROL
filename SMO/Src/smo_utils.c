#include "smo_utils.h"

//Hàm này chỉ chạy một lần duy nhất lúc khởi động hệ thống 
//Không chạy trong ngắt ISR 100khz
//Mục tiêu là tính mọi phép nhân chia nặng tốt bộ nhớ GPU và chặn mọi tham số bẩn trước khi cho phép ngắt hoạt động. 
void FOC_SMO_Init(FOC_SMO_t *smo, float rs, float ls, float ts) //con trỏ *smo, trỏ tới vùng nhớ FOC_SMO_t cần cấu hình
{
    //Chặn con trỏ NULL 
    if (smo == NULL) {
        return;
    }
    //Hạ cờ an toàn trước khi làm gì
    smo->input_fault    = false;  //Báo hàm vừa nhận dữ liệu không hợp lệ
    smo->is_initialized = false;   //Khóa an toàn, chỉ khi nào cơ này == TRUE thì hàm Update trong ISR mới chạy được
    //Kiểm tra xem thông số vật lí đầu vào có phải NaN hay Inf không
    if (!isfinite(rs) || !isfinite(ls) || !isfinite(ts) || !(rs > 0.0f)  || !(ls > 0.0f)  || !(ts > 0.0f))
    {
        smo->input_fault = true;
        return;
    }
    //Tính sẵn hằng số và kiểm tra tràn số học
    float ts_over_ls = ts / ls;     
    float c_lin      = ls / ts;
    if (!isfinite(ts_over_ls) || !isfinite(c_lin)) {
        smo->input_fault = true;
        return;
    }
    //Lưu thông số và xóa sạch các biến trạng thái cũ
    smo->rs          = rs;  
    smo->ls          = ls;
    smo->ts          = ts;
    smo->ts_over_ls  = ts_over_ls;  //Dùng để tích phân điện áp thành dòng điện ở mỗi chu kì
    smo->c_lin       = c_lin;       //Độ lợi tuyến tính deadbeat biến sai số dòng điện thành điện áp trượt z
    smo->i_hat_alpha = 0.0f;        //Dòng điện ước lượng của mô hình SMO trên trục alpha-beta
    smo->i_hat_beta  = 0.0f;
    smo->z_alpha     = 0.0f;        //Tín hiệu điều khiển trượt thô trước khi lọc
    smo->z_beta      = 0.0f;
    smo->e_alpha     = 0.0f;        //Emf mượt sau khi đã qua 2 bộ lọc LPF
    smo->e_beta      = 0.0f;
    smo->ke          = 2.0f;        //Tỉ số tần số cắt wc =|ke|.we
    smo->wc_min      = TWO_PI * 400.0f; //sàn tần số cắt LPF. Đủ rộng cho tốc độ tối thiểu có EMF (~1300 rad/s)
    //Khởi tạo hai bộ lọc LPF con và xác nhận chúng ko lỗi 
    FOC_LPF_InitByWc(&smo->lpf_ea, smo->wc_min, smo->ts, 0.0f);
    FOC_LPF_InitByWc(&smo->lpf_eb, smo->wc_min, smo->ts, 0.0f);
    if (!smo->lpf_ea.is_initialized || !smo->lpf_eb.is_initialized) {
        smo->input_fault = true;
        return;
    }
    //Chỉ khi nào sẵn sàng và vượt qua hết các vòng kiểm tra mới "sẵn sàng"
    smo->is_initialized = true;
}

//Hàm này chạy trong ngắt ISR 100kHz, ngay trước Update với nhiệm vụ
//Kéo tần số cắt của hai LPF back-emf trượt theo tốc độ hiện tại của động cơ adaptive wc chứ ko cố định một số
void FOC_SMO_SetEmfFilterSpeed(FOC_SMO_t *smo, float we)
{
    //Chặn con trỏ Null
    if(smo == NULL){
        return;
    }
    //Kiểm tra xem hàm Init đã thành công chưa và các tham số đầu vào có hợp lệ không
    if (!smo->is_initialized ||!isfinite(we) || !isfinite(smo->ke) || !isfinite(smo->wc_min) || !(smo->ke > 0.0f) || !(smo->wc_min > 0.0f))
    {
        smo->input_fault = true;
        return;
    }
    //Tính tần số cắt tỉ lệ theo tốc độ quay: wc = ke.|we|
    float wc = smo->ke * fabsf(we);
    //Kẹp sàn dưới wc_min để chống trượt về 0 khi động cơ đứng yên hoặc tua chậm
    if (wc < smo->wc_min) {
        wc = smo->wc_min;
    }
    //Cập nhập đồng bộ cho cả hai trục alpha và beta với hàm có sẵn trong lọc
    //Gọi hàm bên trong foc_lpf.c để nó tự tính lại hệ số alpha với wc thay đổi theo we
    FOC_LPF_SetCutoffWc(&smo->lpf_ea, wc);
    FOC_LPF_SetCutoffWc(&smo->lpf_eb, wc);
    //Kiểm tra xem, tuy we hữu hạn nhưng có thể lớn bất thường, khi vào hàm LPF tích wc*ts có thể vượt quá giới hạn
    // khi đó LPF tự chối và chỉ bật cơ lỗi bên trong LPF 
    if (smo->lpf_ea.input_fault || smo->lpf_eb.input_fault) {
        smo->input_fault = true; //dòng này đệ tầng điều khiển cấp cao nhìn thấy cờ SMO để biết cấu hình không hợp lệ
    }
}

//Đây trái tim của bộ quan sát SMO, chạy mỗi 10us trong ngắt 100kHz
//Nhiệm vụ của hàm là chạy một "mô hình động cơ ảo" song song với động cơ thật
//Dùng độ lệch giữa dòng ước lượng đó và dòng thực tế từ cảm biến để tìm ra Back-emf
void FOC_SMO_UpdateCurrentObserver(FOC_SMO_t *smo, float u_alpha, float u_beta, float i_alpha, float i_beta, float k_slide)
{
    if(smo == NULL){
        return;
    }
    if (!smo->is_initialized ||!isfinite(u_alpha) || !isfinite(u_beta) ||!isfinite(i_alpha) || !isfinite(i_beta) ||!isfinite(k_slide) || !(k_slide > 0.0f)){
        smo->input_fault = true;
        return; //Giữ nguyên i_hat,z,e cũ
    }
    // 1. Sai số dòng điện = Ước lượng - Thực tế
    float err_a = smo->i_hat_alpha - i_alpha; //biến cục bộ err_a và err_b sai số dòng điện
    float err_b = smo->i_hat_beta  - i_beta;

    // 2. Luật trượt deadbeat: z = clamp((Ls/Ts) * err, -k_slide, +k_slide)
    // za zb là biến lưu tạm điện áp trượt trước khi vào struct
    float za = smo->c_lin * err_a;
    if (za >  k_slide) { za =  k_slide; }
    if (za < -k_slide) { za = -k_slide; }
    smo->z_alpha = za;

    float zb = smo->c_lin * err_b;
    if (zb >  k_slide) { zb =  k_slide; }
    if (zb < -k_slide) { zb = -k_slide; }
    smo->z_beta = zb;

    // 3. Cập nhật dòng ước lượng (Euler tiến) & lọc Back-EMF qua LPF
    //i_hat_alpha và beta là dòng điện ước lượng cho chu kì tiếp theo k+1
    // i[k+1] = i[k] + (Ts/Ls)*(u - Rs*i[k] - z[k])   ---z[k]
    smo->i_hat_alpha += smo->ts_over_ls * (u_alpha - (smo->rs * smo->i_hat_alpha) - za);
    smo->i_hat_beta  += smo->ts_over_ls * (u_beta  - (smo->rs * smo->i_hat_beta)  - zb);
    //Tín hiệu za,ab được qua bộ lọc LPF để mượt mà trước khi đi ra khỏi SMO
    smo->e_alpha = FOC_LPF_Update(&smo->lpf_ea, za);
    smo->e_beta  = FOC_LPF_Update(&smo->lpf_eb, zb);
}

void FOC_SMO_ClearFault(FOC_SMO_t *smo)
{
    if (smo == NULL) {
        return;
    }
    smo->input_fault = false;
    FOC_LPF_ClearFault(&smo->lpf_ea);
    FOC_LPF_ClearFault(&smo->lpf_eb);
}

bool FOC_SMO_GetTheta(const FOC_SMO_t *smo, float we, float *theta_out)
{
    if ((smo == NULL) || (theta_out == NULL) || !smo->is_initialized || !isfinite(we) || !isfinite(smo->e_alpha) || !isfinite(smo->e_beta)){
        return false;
    }

    if ((smo->e_alpha == 0.0f) && (smo->e_beta == 0.0f)) {
        return false;
    }

    float c, s;
    FOC_LPF_GetLagRotation(&smo->lpf_ea, we, &c, &s);

    float ea = (smo->e_alpha * c) - (smo->e_beta * s);
    float eb = (smo->e_alpha * s) + (smo->e_beta * c);
    float sg = (we >= 0.0f) ? 1.0f : -1.0f;

    *theta_out = fast_atan2(-ea * sg, eb * sg);
    return true;
}

float FOC_SMO_GetTotalLag(const FOC_SMO_t *smo, float we)
{
    if ((smo == NULL) || !smo->is_initialized || !isfinite(we)) {
        return 0.0f;
    }
    float lpf_lag = FOC_LPF_GetPhaseLagExact(&smo->lpf_ea, we);
    float obs_lag = FOC_SMO_OBS_LAG_SAMPLES * we * smo->ts;
    return lpf_lag + obs_lag;
}