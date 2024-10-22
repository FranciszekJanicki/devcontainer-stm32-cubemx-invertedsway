/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.hpp"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include <cstdint>
#include <cstring>

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
char buffer[128];
MPU6050* mpu6050_handle{nullptr};
L298N* l298n_handle{nullptr};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

template <typename Value>
static auto make_filter() noexcept
{
    return [sum = Value{}, N = double{0.0}](const Value& input) mutable {
        sum += input;
        return sum / N++;
    };
}

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
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */

    MPU6050 mpu6050{&huart2, &hi2c1, MPU6050::ADDRESS, MPU6050::GYRO_FS_250, MPU6050::ACCEL_FS_2};
    mpu6050_handle = &mpu6050;

    L298N l298n{&huart2,
                L298N::Motors{
                    L298N::Motor{
                        L298N::MotorChannel::CHANNEL1,
                        &htim2,
                        TIM_CHANNEL_1,
                        GPIOA,
                        IN1_Pin,
                        IN3_Pin,
                        OUT1_Pin,
                        OUT3_Pin,
                    },
                    L298N::Motor{},
                }};

    auto temp_filter{make_filter<MPU6050::TempScaled>()};
    auto accel_filter{make_filter<MPU6050::AccelScaled>()};
    auto rpy_filter{make_filter<MPU6050::RollPitchYaw>()};
    auto gyro_filter{make_filter<MPU6050::GyroScaled>()};

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (true) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if (auto accel_scaled{mpu6050.get_accelerometer_scaled()}; accel_scaled.has_value()) {
            const auto& [ax, ay, az]{accel_filter(accel_scaled.value())};
            memset(buffer, 0, 128);
            sprintf(buffer, "ACC: X: %.2f Y:%.2f Z:%.2f", ax, ay, az);
            uart_send_string(&huart2, buffer);
        }

        if (auto gyro_scaled{mpu6050.get_gyroscope_scaled()}; gyro_scaled.has_value()) {
            const auto& [gx, gy, gz]{gyro_filter(gyro_scaled.value())};
            memset(buffer, 0, 128);
            sprintf(buffer, "GYRO: A: %.2f B:%.2f C:%.2f", gx, gy, gz);
            uart_send_string(&huart2, buffer);
        }

        if (auto roll_pitch_yaw{mpu6050.get_roll_pitch_yaw()}; roll_pitch_yaw.has_value()) {
            const auto& [r, p, y]{rpy_filter(roll_pitch_yaw.value())};
            memset(buffer, 0, 128);
            sprintf(buffer, "RPY: R: %.2f P:%.2f Y:%.2f", r, p, y);
            uart_send_string(&huart2, buffer);
        }

        if (auto temperature{mpu6050.get_temperature_celsius()}; temperature.has_value()) {
            const auto temp{temp_filter(temperature.value())};
            memset(buffer, 0, 128);
            sprintf(buffer, "TEMP: %.2f", temp);
            uart_send_string(&huart2, buffer);
        }

        HAL_Delay(1000);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INTR_Pin) {
        if (auto interrupts{mpu6050_handle->get_int_status_register()}; interrupts.has_value()) {
            sprintf(buffer, "int_ status triggered: %X\n\n\r", interrupts.value());
            uart_send_string(&huart2, buffer);

            if (interrupts.value() & (0x01 << MPU6050::INTERRUPT_FIFO_OFLOW_BIT)) // Bit 4 (0x10)
            {
                sprintf(buffer, "FIFO Overflow detected\n\r");
                uart_send_string(&huart2, buffer);
            }

            if (interrupts.value() & (0x01 << MPU6050::INTERRUPT_MOT_BIT)) // Bit 6 (0x40)
            {
                sprintf(buffer, "Motion detected\n\r");
                uart_send_string(&huart2, buffer);
            }

            if (interrupts.value() & (0x01 << MPU6050::INTERRUPT_ZMOT_BIT)) // Bit 5 (0x20)
            {
                sprintf(buffer, "Zero Motion detected\n\r");
                uart_send_string(&huart2, buffer);
            }

            if (interrupts.value() & (0x01 << MPU6050::INTERRUPT_FF_BIT)) // Bit 7 (0x80)
            {
                sprintf(buffer, "Freefall detected\n\r");
                uart_send_string(&huart2, buffer);
            }
        } else {
        }
        // mpu6050.get_motion_status_register();
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
    while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
