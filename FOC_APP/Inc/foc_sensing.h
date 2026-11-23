#ifndef FOC_SENSING_H
#define FOC_SENSING_H
#include <stdint.h>
#include "foc_config.h"
#include <stdbool.h>

#define ADC_RESOLUTION_MAX          4096.0f         // ADC 12-bit (0 .. 4095)
#define ADC_VREF_VOLT               3.3f            // Điện áp tham chiếu VDDA = 3.3V
#define SHUNT_RESISTANCE_OHM        0.0002f         // Điện trở Shunt = 0.2 mOhm
#define CURRENT_AMP_GAIN            20.0f           // IC INA240A1 Gain = 20 V/V
// Hệ số quy đổi 1 count ADC sang Ampe: (3.3 / 4096) / (0.0002 * 20) = 0.201465 A/count
#define ADC_TO_CURRENT_FACTOR  ((ADC_VREF_VOLT / ADC_RESOLUTION_MAX) / (SHUNT_RESISTANCE_OHM * CURRENT_AMP_GAIN)) // Hệ số quy đổi 1 count ADC sang Ampe
//Ngưỡng chạm biên thanh ghi ADC - 12bit
#define ADC_SATURATION_LOW_LSB          8U
#define ADC_SATURATION_HIGH_LSB         4087U
#define CURRENT_SIGN_A              (1.0f)
#define CURRENT_SIGN_B              (1.0f)
//CẤU HÌNH HIỆU CHỈNH CALIBRATION OFFSET CHO INA240
#define OFFSET_CALIB_SAMPLES 2000U  //2000 mẫu tương đương 20ms
#define OFFSET_ADC_MIN_VALID 1700.0f // Ngưỡng ADC tối thiểu để xác định giá trị Offset hợp lệ (tương đương 1.37V)
#define OFFSET_ADC_MAX_VALID 2400.0f // Ngưỡng ADC tối đa để xác định giá trị Offset hợp lệ (tương đương 1.93V)

//Thông số mạch đo điện áp nguồn Vbus
#define VBUS_DIVIDER_RATIO          20.09f    
#define ADC_TO_VBUS_FACTOR       ((ADC_VREF_VOLT / ADC_RESOLUTION_MAX) * VBUS_DIVIDER_RATIO) 
#define VBUS_LPF_ALPHA              0.01f            // Hệ số lọc LPF cho Vbus (0.01 ~ 1ms)
#define OVP_DEBOUNCE_SAMPLES 2U

 //Cấu trúc dữ liệu cảm biến dòng và cờ bảo vệ
 typedef struct{
    float ia;
    float ib;
    float ic;
    float vbus;
    float vbus_raw;
    float inv_vbus;
//Biến nội bộ hiểu chuẩn điểm 0
    float offset_ia;
    float offset_ib;
    uint32_t calib_cnt; //bộ đếm mẫu hiệu chuẩn
    uint8_t vbus_div_cnt;
    uint8_t ovp_debounce_cnt;
//Trạng thái và cờ lỗi bảo vệ phần mềm
    bool     is_calibrated;         // true khi đã hoàn tất đo điểm 0 an toàn
    bool     sensor_offset_fault;   // Lỗi phần cứng INA240 (Offset trôi ngoài [1700, 2400])
    bool     over_current_fault;    // Lỗi quá dòng OCP (|ia|, |ib| hoặc |ic| > FOC_MAX_CURRENT_A)
    bool     over_voltage_fault;    // Lỗi quá áp OVP (vbus > FOC_MAX_VBUS_V)
    bool     under_voltage_fault;   // Lỗi thấp áp UVLO (vbus < FOC_MIN_VBUS_V)
}FOC_Sensor_t;
// Nguyên mẫu hàm
//Khởi tạo các giá trị ban đầu cho cấu trúc cảm biến, xóa các lỗi cờ cũ và gồm cả bước hiệu chuẩn điểm 0
void FOC_Sensor_Init(FOC_Sensor_t *sensor);
//Hàm xử lí lõi, được gọi định kì mỗi 10us chu kì bên trong ngắt ADC hoàn thành,nó nhận các giá trị thô và đổi sang đơn vị vật lí
void FOC_Sensor_Update_100kHz(FOC_Sensor_t *sensor, uint16_t raw_ia, uint16_t raw_ib, uint16_t raw_vbus);
//Dùng để xóa thủ công các cờ báo lỗ íau khi hệ thống đã gặp sự cố và người dùng muốn khởi động lại động cơ
void FOC_Sensor_ClearFaults(FOC_Sensor_t *sensor);
// Hàm kiểm tra nhanh xem hệ thống có đang dính bất kỳ lỗi nào không
//4 cờ giám sát an toàn 
static inline bool FOC_Sensor_HasFault(const FOC_Sensor_t *sensor) {
    return (sensor->sensor_offset_fault || //điểm 0 của INA240 bị lệch quá xa mức 1,65V
            sensor->over_current_fault  || //Dòng diện pha vượt quá ngưỡng chịu đựng của motor/mosfet
            sensor->over_voltage_fault  || //Điện áp nguồn tăng vọt quá giới hạn
            sensor->under_voltage_fault);  //Điện áp nguồn bị sụt áp thấp
}
#endif // FOC_SENSING_H