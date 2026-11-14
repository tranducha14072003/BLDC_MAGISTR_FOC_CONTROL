//THực thi quỳ đổi ADC-12 bit sang dòng điện và điện áp
//tự động calibration offset và giám sát bảo vệ OCP và UVLO ở 100kHz
#include "foc_sensing.h"
#include <math.h>

void FOC_Sensor_Init(FOC_Sensor_t *sensor){
    sensor->ia                  = 0.0f;
    sensor->ib                  = 0.0f;
    sensor->ic                  = 0.0f;
    sensor->vbus                = VBUS_NOMINAL_VOLT;  
    sensor->vbus_raw            = VBUS_NOMINAL_VOLT;
    sensor->inv_vbus            = 1.0f / VBUS_NOMINAL_VOLT; 

    sensor->offset_ia           = 0.0f;
    sensor->offset_ib           = 0.0f;
    sensor->calib_cnt           = 0U;
    sensor->vbus_div_cnt        = 0U;
    sensor->ovp_debounce_cnt    = 0U;

    sensor->is_calibrated       = false;
    sensor->sensor_offset_fault = false;
    sensor->over_current_fault  = false;
    sensor->over_voltage_fault  = false;
    sensor->under_voltage_fault = false;

}
void FOC_Sensor_ClearFaults(FOC_Sensor_t *sensor){
    if(sensor->sensor_offset_fault){
    sensor->offset_ia           = 0.0f;
    sensor->offset_ib           = 0.0f;
    sensor->calib_cnt           = 0U;
    sensor->is_calibrated       = false;
    }
    sensor->ovp_debounce_cnt    = 0U;
    sensor->sensor_offset_fault = false;
    sensor->over_current_fault  = false;
    sensor->over_voltage_fault  = false;
    sensor->under_voltage_fault = false;
}
void FOC_Sensor_Update_100kHz(FOC_Sensor_t *sensor, uint16_t raw_ia, uint16_t raw_ib, uint16_t raw_vbus)
{
    // 1. Đo và lọc thông thấp điện áp nguồn DC-Link (Vbus)
    float vbus_meas = (float)raw_vbus * ADC_TO_VBUS_FACTOR;
    sensor->vbus_raw = vbus_meas;
    //Kiểm tra OVP tức thời trên mẫu thô(yêu cầu 2 mẫu liên tiếp để chống nhiễu)
    if(vbus_meas > FOC_MAX_VBUS_V){
        if(sensor->ovp_debounce_cnt < 255U){
            sensor->ovp_debounce_cnt ++;
        }
        if(sensor->ovp_debounce_cnt >= OVP_DEBOUNCE_SAMPLES){
             sensor->over_voltage_fault = true;
        } 
    }else{
            sensor->ovp_debounce_cnt = 0U;
        }
    // Mạch lọc thông thấp LPF DISCRESTE
    if (sensor->calib_cnt == 0U && !sensor->is_calibrated) {
        sensor->vbus = vbus_meas;
        if (sensor->vbus > 1.0f) {
            sensor->inv_vbus = 1.0f / sensor->vbus;
        }
    } else {
        sensor->vbus += VBUS_LPF_ALPHA * (vbus_meas - sensor->vbus);
    }
    // Cập nhật inv_vbus mỗi 64 chu kỳ PWM (640 us ~ 1.56 kHz) để tiết kiệm lệnh chia FPU
    sensor->vbus_div_cnt++;
    if ((sensor->vbus_div_cnt & 0x3FU) == 0U) {
        if (sensor->vbus > 1.0f) {
            sensor->inv_vbus = 1.0f / sensor->vbus;
        }
    }
    // Giám sát quá áp OVP ngay lập tức
    if (sensor->vbus > FOC_MAX_VBUS_V) {
        sensor->over_voltage_fault = true;
    }
    // 2. Giai đoạn tự động tìm điểm 0 (Auto-Offset Calibration) trong 20 ms đầu
    if (!sensor->is_calibrated) {
        if(sensor->sensor_offset_fault){
            return;
        }
        sensor->offset_ia += (float)raw_ia;
        sensor->offset_ib += (float)raw_ib;
        sensor->calib_cnt++;

        if (sensor->calib_cnt >= OFFSET_CALIB_SAMPLES) {
            float avg_ia = sensor->offset_ia / (float)OFFSET_CALIB_SAMPLES;
            float avg_ib = sensor->offset_ib / (float)OFFSET_CALIB_SAMPLES;

            // Kiểm tra tính hợp lệ của phần cứng INA240 (phải nằm quanh mốc 1.65V ~ 2048)
            if (avg_ia < OFFSET_ADC_MIN_VALID || avg_ia > OFFSET_ADC_MAX_VALID ||
                avg_ib < OFFSET_ADC_MIN_VALID || avg_ib > OFFSET_ADC_MAX_VALID)
            {
                sensor->sensor_offset_fault = true;
                sensor->is_calibrated = false;
            } else {
                sensor->offset_ia     = avg_ia;
                sensor->offset_ib     = avg_ib;
                sensor->is_calibrated = true;
            }
        }
        return;
    }
    if (sensor->vbus < FOC_MIN_VBUS_V){
        sensor->under_voltage_fault = true;
    }
    if (raw_ia <= ADC_SATURATION_LOW_LSB || raw_ia >= ADC_SATURATION_HIGH_LSB ||
        raw_ib <= ADC_SATURATION_LOW_LSB || raw_ib >= ADC_SATURATION_HIGH_LSB)
    {
        sensor->over_current_fault = true;
    }
    // 4. Quy đổi sang dòng điện 3 pha thực tế (Ampe) & Định luật Kirchhoff 1
    sensor->ia = ((float)raw_ia - sensor->offset_ia) * ADC_TO_CURRENT_FACTOR * CURRENT_SIGN_A;
    sensor->ib = ((float)raw_ib - sensor->offset_ib) * ADC_TO_CURRENT_FACTOR * CURRENT_SIGN_B;
    sensor->ic = -(sensor->ia + sensor->ib);
    // 5. Bảo vệ quá dòng phần mềm (OCP) cho cả 3 pha
    if (fabsf(sensor->ia) > FOC_MAX_CURRENT_A || fabsf(sensor->ib) > FOC_MAX_CURRENT_A ||
        fabsf(sensor->ic) > FOC_MAX_CURRENT_A)
    {
        sensor->over_current_fault = true;
    }
}