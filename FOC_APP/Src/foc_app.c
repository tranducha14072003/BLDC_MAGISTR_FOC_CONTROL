/* ISR 100 kHz — thứ tự bắt buộc

 * 1. đọc ia, ib từ cảm biến dòng (đã trừ offset)         
 -----------------------> qua biến đổi Clarke -> thành i_alpha, i_beta

 * 2. SMO_UpdateCurrentObserver(u_prev, i)  
 ---------------------->  u_prev = điện áp ÁP Ở CHU KỲ TRƯỚC
 //Bộ quan sát dòng điện từ i_alpha và i_beta ở bước 1 lấy ra z ~ EMF từ error_current

 * 3. SMO_SetEmfFilterSpeed(we_pll)
LPF thích nghi lọc z thành e (emf sạch hơn)

 * 4. PLL_Update(e_alpha, e_beta)
PLL khoá vào hướng của ê, cho ra θ̂ mịn và ω̂ (không cần đạo hàm góc, vì đạo hàm sẽ khuếch đại nhiễu).
 
* 5. startup/FOC -> v_alpha, v_beta mới

* 6. SVPWM -> CCR;  lưu u_prev = điện áp thực áp ở chu kỳ tới

 */