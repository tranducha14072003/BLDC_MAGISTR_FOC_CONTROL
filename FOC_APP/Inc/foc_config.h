#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H
#include <stdint.h>
// 1. Thông số Timer & PWM
#define TIM1_CLOCK_HZ 170000000U 
#define FOC_PWM_FREQ_HZ 100000
#define FOC_TS (1.0F / (float)FOC_PWM_FREQ_HZ)
#define SPEED_TS (FOC_TS * 10.0f) // Thời gian lấy mẫu tốc độ động cơ (10 lần so với FOC_TS)
#define PWM_PERIOD_CYCLES 850U
#define PWM_HALF_PERIOD (PWM_PERIOD_CYCLES >> 1)

#define USE_24V_SAFE_TEST_MODE  1   // 1: Bàn test 24V | 0: Thực chiến 60V (14S)
#if (USE_24V_SAFE_TEST_MODE == 1)
    #define FOC_MAX_CURRENT_A   40.0f  // Cắt quá dòng OCP ở 40A khi test bàn lab
    #define FOC_MAX_VBUS_V      30.0f   // Cắt quá áp OVP ở 30V
    #define FOC_MIN_VBUS_V      10.0f        
    #define VBUS_NOMINAL_VOLT   24.0f 
#else
    #define FOC_MAX_CURRENT_A   320.0f   // Cắt quá dòng OCP thực chiến: 320A
    #define FOC_MAX_VBUS_V      65.0f    
    #define FOC_MIN_VBUS_V      38.0f    //Cắt thấp áp OVP pin 14S: 
    #define VBUS_NOMINAL_VOLT 50.0f    // Ngắt quá áp UVLO pin 14S: 
#endif /* FOC_INC_FOC_CONFIG_H_ */
#endif







