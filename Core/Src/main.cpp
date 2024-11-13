#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu6050.hpp"
#include "system_clock.h"
#include "tim.h"
#include "usart.h"
#include <cstdio>
#include <utility>

int main()
{
    using namespace InvertedSway;

    HAL_Init();
    SystemClock_Config();

    MX_USART2_UART_Init();
    MX_I2C1_Init();

    MPU6050 mpu6050{&hi2c1,
                    MPU6050::ADDRESS,
                    MPU6050::GYRO_FS_250,
                    MPU6050::ACCEL_FS_2,
                    MPU6050::make_gyro_filter(),
                    MPU6050::make_accel_filter()};

    while (true) {
        const auto& [ax, ay, az]{mpu6050.get_accelerometer_scaled()};
        const auto& [gx, gy, gz]{mpu6050.get_gyroscope_scaled()};
        const auto temp{mpu6050.get_temperature_celsius()};

        printf("ACC: X: %.2f Y:%.2f Z:%.2f \n\rGYR: X: %.2f Y:%.2f Z:%.2f\n\rTEMP: %.2f\n\r",
               ax,
               ay,
               az,
               gx,
               gy,
               gz,
               temp);

        HAL_Delay(1000);
    }

    return 0;
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
