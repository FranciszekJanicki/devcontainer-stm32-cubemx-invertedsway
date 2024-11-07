#include "main.h"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include "tim.h"
#include "usart.h"
#include <cstdio>
#include <span>
#include <system.hpp>
#include <type_traits>
#include <utility>

using namespace InvertedSway;

constexpr MPU6050::Scaled ANGLE = 0;
constexpr MPU6050::Scaled SAMPLING_TIME = 1;

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

// for using System object in callback
System* system_handle{nullptr};

namespace BetterMain {

    using Arguments = std::span<const char*>;

    template <typename Value>
    [[nodiscard]] constexpr auto main(const Arguments arguments) noexcept -> decltype(auto)
        requires(std::is_integral_v<Value>)
    {
        HAL_Init();
        SystemClock_Config();

        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_TIM2_Init();
        MX_I2C1_Init();
        MX_TIM1_Init();

        L298N l298n{L298N::MotorChannels{
            L298N::make_motor_channel(L298N::Channel::CHANNEL1,
                                      &htim2,
                                      TIM_CHANNEL_1,
                                      GPIOB,
                                      IN1_Pin,
                                      IN3_Pin,
                                      OUT1_Pin,
                                      OUT3_Pin),
            L298N::make_motor_channel(L298N::Channel::CHANNEL2),
        }};

        MPU6050 mpu6050{&hi2c1,
                        MPU6050::ADDRESS,
                        MPU6050::GYRO_FS_250,
                        MPU6050::ACCEL_FS_2,
                        make_recursive_average<MPU6050::GyroRaw, MPU6050::Raw>(),
                        make_recursive_average<MPU6050::AccelRaw, MPU6050::Raw>()};

        auto kalman = make_kalman<MPU6050::Scaled>(0.0, 0.0, 0.0, 0.0, 0.0);

        auto regulator = make_regulator<MPU6050::Scaled>(RegulatorAlgo::PID, 0.0, 0.0, 0.0, 0.0);

        Encoder encoder{&htim1, ANGLE};

        System system{std::move(mpu6050),
                      std::move(l298n),
                      std::move(kalman),
                      std::move(regulator),
                      std::move(encoder)};
        system_handle = &system;

        while (true) {
            ;
            ;
        }

        return Value{0};
    }

}; // namespace BetterMain

int main(const int argc, const char* argv[])
{
    return BetterMain::main<int>(BetterMain::Arguments{argv, argc});
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == nullptr) {
        return;
    }
    if (htim->Instance == TIM1) {
        system_handle->balance_sway(ANGLE, SAMPLING_TIME);
    }
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == INTR_Pin) {
//         const auto interrupts{mpu6050_handle->get_int_status_register()};
//         mpu6050_handle->get_motion_status_register();
//         printf( "int_ status triggered: %X\n\n\r", interrupts);

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_FIFO_OFLOW_BIT)) // Bit 4 (0x10)
//         {
//             printf( "FIFO Overflow detected\n\r");
//         }

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_MOT_BIT)) // Bit 6 (0x40)
//         {
//             printf( "Motion detected\n\r");
//         }

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_ZMOT_BIT)) // Bit 5 (0x20)
//         {
//             printf( "Zero Motion detected\n\r");
//         }

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_FF_BIT)) // Bit 7 (0x80)
//         {
//             printf( "Freefall detected\n\r");
//         }
//     }
// }

void Error_Handler()
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{}
#endif /* USE_FULL_ASSERT */
