/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc_config.h"
#include "FOC_math.h"
#include "foc_sensing.h"
#include "foc_pid.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define FOC_BUILD_LEVEL 3
//Các hàm đối tượng dữ liệu của hệ thống FOC
FOC_Sensor_t g_sensor;
FOC_PI_t     g_pi_id;
FOC_PI_t     g_pi_iq;
// Các biến trạng thái theo dõi trực tiếp trên cửa sổ Live Watch khi Debug
volatile float    g_theta_e   = 0.0f;   // Góc điện hiện tại (rad)
volatile float    g_we_open   = 0.0f;   // Tốc độ góc điện vòng hở (rad/s)
volatile float    g_i_alpha   = 0.0f;
volatile float    g_i_beta    = 0.0f;
volatile float    g_id_meas   = 0.0f;
volatile float    g_iq_meas   = 0.0f;
volatile float    g_id_ref    = 0.0f;   // Lệnh dòng trục d (Luôn giữ = 0A)
volatile float    g_iq_ref    = 2.0f;   // Lệnh dòng trục q thử nghiệm ở Build 3 (2.0A)
volatile float    g_vd_cmd    = 0.0f;
volatile float    g_vq_cmd    = 2.0f;   // Điện áp Vq thử nghiệm ở Build 1 & 2 (2.0V)
volatile uint32_t g_ccr_u     = PWM_HALF_PERIOD;
volatile uint32_t g_ccr_v     = PWM_HALF_PERIOD;
volatile uint32_t g_ccr_w     = PWM_HALF_PERIOD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
// 1. Khởi tạo bộ tăng tốc lượng giác phần cứng CORDIC (Mốc 1)
  init_trig_lut();
  // 2. Khởi tạo module Cảm biến & Bảo vệ (Mốc 2)
  FOC_Sensor_Init(&g_sensor);
  // 3. Khởi tạo 2 bộ điều khiển PI dòng điện Id và Iq ở Ts = 10 us (Mốc 3)
  // Thiết kế theo băng thông 1500 Hz (w_cc = 9424.8 rad/s): Kp = Ls*w_cc ~ 2.07, Ki = Rs*w_cc ~ 18.0
  float v_max_init = VBUS_NOMINAL_VOLT  * SVPWM_LIMIT_COEFF; // ~13.3V ở nguồn 24V
  FOC_PI_Init(&g_pi_id, 2.07f, 300.0f, FOC_TS, -v_max_init, v_max_init);
  FOC_PI_Init(&g_pi_iq, 2.07f, 300.0f, FOC_TS, -v_max_init, v_max_init);
  // 4. Đặt sẵn giá trị thanh ghi PWM ở mức trung tính 50% (425) trước khi bật cầu GaN
  TIM1->CCR1 = PWM_HALF_PERIOD;
  TIM1->CCR2 = PWM_HALF_PERIOD;
  TIM1->CCR3 = PWM_HALF_PERIOD;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void FOC_Run_100kHz_ISR(uint16_t raw_ia, uint16_t raw_ib, uint16_t raw_vbus)
{
    // TẦNG 1 (MỐC 2): ĐỌC CẢM BIẾN, HIỆU CHUẨN OFFSET & KIỂM TRA BẢO VỆ
    FOC_Sensor_Update_100kHz(&g_sensor, raw_ia, raw_ib, raw_vbus);

    // Nếu phát hiện bất kỳ lỗi nào (OCP, OVP, UVLO, Lỗi phần cứng INA240):
    // Khóa cứng ngõ ra Main Output Enable (MOE) của TIM1 ngay lập tức!
    if (FOC_Sensor_HasFault(&g_sensor)) {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCR1  = PWM_HALF_PERIOD;
        TIM1->CCR2  = PWM_HALF_PERIOD;
        TIM1->CCR3  = PWM_HALF_PERIOD;
        FOC_PI_Reset(&g_pi_id);
        FOC_PI_Reset(&g_pi_iq);
        return;
    }

    // Trong 20 ms đầu tiên (2000 mẫu): Khi đang đo điểm 0 (Offset) của INA240,
    // giữ cứng cả 3 pha ở mức 50% Duty Cycle (Điện áp dây = 0V, dòng = 0A)
    if (!g_sensor.is_calibrated) {
        TIM1->CCR1 = PWM_HALF_PERIOD;
        TIM1->CCR2 = PWM_HALF_PERIOD;
        TIM1->CCR3 = PWM_HALF_PERIOD;
        return;
    }
    // TẦNG 2 (MỐC 1): TẠO GÓC QUAY & BIẾN ĐỔI CLARKE -> PARK
    // Tăng tốc mềm (Ramp) tần số góc điện vòng hở lên 200 rad/s để test Build 1, 2, 3
    if (g_we_open < 200.0f) {
        g_we_open += 200.0f * FOC_TS; // Tăng tốc mất 1 giây
    }
    g_theta_e += g_we_open * FOC_TS;
    normalize_angle((float *)&g_theta_e);

    float sin_t, cos_t;
    fast_sincos(g_theta_e, &sin_t, &cos_t);

#if (FOC_BUILD_LEVEL >= 2)
    // Từ Build 2 trở đi: Thực hiện biến đổi Clarke và Park trên dòng điện thực tế
    float i_alpha, i_beta, id, iq;
    clarke_transform(g_sensor.ia, g_sensor.ib, &i_alpha, &i_beta);
    park_transform(i_alpha, i_beta, sin_t, cos_t, &id, &iq);

    g_i_alpha = i_alpha;
    g_i_beta  = i_beta;
    g_id_meas = id;
    g_iq_meas = iq;
#endif
    // TẦNG 3 (MỐC 3): BỘ ĐIỀU KHIỂN PI DÒNG ĐIỆN TRỤC D - Q
    float vd_out, vq_out;
#if (FOC_BUILD_LEVEL >= 3)
    // Ở Build 3: Cập nhật trần điện áp động theo Vbus thực tế (0.96 / sqrt(3) * Vbus)
    float v_limit = g_sensor.vbus * SVPWM_LIMIT_COEFF;
    FOC_PI_SetOutputLimits(&g_pi_id, -v_limit, v_limit);
    FOC_PI_SetOutputLimits(&g_pi_iq, -v_limit, v_limit);

    // Đóng vòng kín điều khiển dòng điện Id, Iq bằng bộ PI Anti-Windup
    vd_out   = FOC_PI_Update(&g_pi_id, g_id_ref, g_id_meas);
    vq_out   = FOC_PI_Update(&g_pi_iq, g_iq_ref, g_iq_meas);
    g_vd_cmd = vd_out;
    g_vq_cmd = vq_out;
#else
    // Ở Build 1 & Build 2: Dùng điện áp đặt vòng hở nhỏ (Vd = 0V, Vq = 2.0V)
    vd_out = g_vd_cmd;
    vq_out = g_vq_cmd;
#endif
    // TẦNG 4 (MỐC 1): INVERSE PARK & PHÁT XUNG SVPWM RA THANH GHI TIM1
    float v_alpha, v_beta;
    uint32_t u, v, w;
    inverse_park_transform(vd_out, vq_out, sin_t, cos_t, &v_alpha, &v_beta);
    svpwm(v_alpha, v_beta, g_sensor.vbus,1/g_sensor.vbus  , PWM_PERIOD_CYCLES, &u, &v, &w);
    // Ghi trực tiếp vào thanh ghi phần cứng của Timer 1
    TIM1->CCR1 = u;
    TIM1->CCR2 = v;
    TIM1->CCR3 = w;
    g_ccr_u = u;
    g_ccr_v = v;
    g_ccr_w = w;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
