/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

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

volatile uint16_t currentPulse = 1000;
volatile uint32_t lastTick = 0;
volatile uint32_t rpm = 0;
char msg[64];

// Valores de descarte
#define DIFF_MIN_MS 2 		// Intervalo menor a 2 [ms] --> RPM>30k
#define DIFF_MAX_MS 2000   // Intervalo mayor a 2 [s]

/* --- Config específica de RPM --- */
#define TIMER_TICK_HZ     1000000UL  // 1 MHz (1 us/tick) por PSC=71 con timer_clk=72 MHz
#define TIMER_MAX_COUNT   65536UL    // 16 bits -> 0..65535 => 65536 cuentas
#define PULSOS_POR_REV    1          // ajustá según tu rueda/marca (1 si 1 pulso por vuelta)

// Variables para medición por captura
volatile uint32_t ic_overflows = 0;          // cuántos overflows desde el arranque
volatile uint32_t last_abs_count = 0;        // cuenta absoluta (captura + overflows previos)
volatile uint8_t  first_capture = 1;         // para inicializar
volatile float    rpm_inst = 0.0f;           // RPM instantánea calculada

// (Opcional) filtro exponencial para suavizar
#define ALFA  0.2f
float rpm_filtrada = 0.0f;

extern volatile float rpm_inst;
extern float rpm_filtrada;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void PWM_SetPulse_us(uint16_t pulse_us) {
    // CCR = pulse_us / 1 µs * factor de timer
    // Si TIM1 está a 72 MHz con prescaler= 71, entonces 1 tick = 1 µs
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
    currentPulse = pulse_us;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {  // Verifica que sea nuestro PA0
        uint32_t now = HAL_GetTick();  // tiempo actual en ms
        uint32_t diff = now - lastTick;

        if (diff >= DIFF_MIN_MS && diff <= DIFF_MAX_MS) {
            rpm = 60000 / diff;
        } else {
            rpm = 0; // fuera de rango o timeout
        }
        lastTick = now;
    }
}


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
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  if (HAL_GetTick() - t0 >= 100)   // cada 100 ms
	      {
	        t0 += 100;

	        // snapshot atómico
	        float rpm_send;
	        __disable_irq();
	        rpm_send = rpm_filtrada;       // o rpm_inst si querés cruda
	        __enable_irq();

	        // Para evitar depender de printf con float, mando milésimas de RPM:
	        int rpm_mil = (int)(rpm_send * 1000.0f);

	        char msg[64];
	        int len = snprintf(msg, sizeof(msg), "RPMx1000:%d\r\n", rpm_mil);

	        // Enviar por USB-CDC
	        // Nota: CDC_Transmit_FS devuelve USBD_OK o USBD_BUSY; si está ocupado, podés reintentar o descartar.
	        if (len > 0) {
	          uint8_t status = CDC_Transmit_FS((uint8_t*)msg, (uint16_t)len);
	          (void)status; // si querés, chequeá USBD_BUSY para reintentar
	        }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Se llama en cada OVERFLOW (Update Event) del TIM3
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    ic_overflows++;   // acumulamos overflows para construir el "contador absoluto"
  }
}

// Se llama en cada CAPTURA (flanco en CH1)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

    // 1) Leo el valor capturado (0..65535)
    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    // 2) Tomo un snapshot atómico del contador de overflows
    //    (la secuencia minimiza race conditions con overflow justo entre lectura y uso)
    __disable_irq();
    uint32_t of = ic_overflows;
    // Si justo ocurrió overflow y la captura pertenece al período anterior,
    // ajustamos el conteo. (HAL ya latchea, esto es por seguridad)
    if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) != RESET &&
        __HAL_TIM_GET_COUNTER(htim) < cap) {
      of++;
    }
    __enable_irq();

    // 3) Construyo el conteo absoluto (en ticks de 1 us)
    //    abs_count = of * 65536 + cap
    uint32_t abs_count = of * TIMER_MAX_COUNT + cap;

    // 4) Cálculo del período (delta de ticks) contra la captura anterior
    if (!first_capture) {
      uint32_t delta_ticks = abs_count - last_abs_count;   // en 1 us/tick

      if (delta_ticks > 0) {
        // f = 1 / periodo;   rpm = f * 60 / PPR
        // periodo_us = delta_ticks (porque 1 tick = 1 us)
        // rpm = 60e6 / (delta_us * PPR)
        float rpm = (60.0f * 1000000.0f) / ((float)delta_ticks * (float)PULSOS_POR_REV);

        rpm_inst = rpm;
        // filtro simple para suavizar
        rpm_filtrada = (ALFA * rpm_inst) + (1.0f - ALFA) * rpm_filtrada;
      }
    } else {
      first_capture = 0;
      rpm_filtrada = 0.0f;
    }

    // 5) Guardar para la próxima
    last_abs_count = abs_count;
  }
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

#ifdef  USE_FULL_ASSERT
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
