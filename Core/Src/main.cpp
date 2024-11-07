#include "main.h"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include "system_clock.h"
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

int main(const int argc, const char* argv[])
{
    return BetterMain::main<int>(BetterMain::Arguments{argv, argc});
}

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
