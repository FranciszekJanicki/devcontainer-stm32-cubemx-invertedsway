#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu6050.h"
#include "system_clock.h"
#include "tim.h"
#include "usart.h"
#include <stdio.h>

int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;
float ax, ay, az, gx, gy, gz, temperature, roll, pitch;

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_USART2_UART_Init();
    MX_I2C1_Init();

    MPU6050_Init(&hi2c1);

    MPU6050_SetInterruptMode(MPU6050_INTMODE_ACTIVEHIGH);
    MPU6050_SetInterruptDrive(MPU6050_INTDRV_PUSHPULL);
    MPU6050_SetInterruptLatch(MPU6050_INTLATCH_WAITCLEAR);
    MPU6050_SetInterruptLatchClear(MPU6050_INTCLEAR_STATUSREAD);

    MPU6050_SetIntEnableRegister(0);

    MPU6050_SetDHPFMode(MPU6050_DHPF_5);

    MPU6050_SetIntMotionEnabled(1);
    MPU6050_SetIntZeroMotionEnabled(1);
    MPU6050_SetIntFreeFallEnabled(1);

    MPU6050_SetFreeFallDetectionDuration(2);
    MPU6050_SetFreeFallDetectionThreshold(5);

    MPU6050_SetMotionDetectionDuration(5);
    MPU6050_SetMotionDetectionThreshold(2);

    MPU6050_SetZeroMotionDetectionDuration(2);
    MPU6050_SetZeroMotionDetectionThreshold(4);

    while (1) {
        MPU6050_GetAccelerometerScaled(&ax, &ay, &az);
        MPU6050_GetGyroscopeScaled(&gx, &gy, &gz);
        temperature = MPU6050_GetTemperatureCelsius();
        printf("ACC: X: %.2f Y:%.2f Z:%.2f \n\rGYR: X: %.2f Y:%.2f Z:%.2f\n\rTEMP: %.2f\n\r",
               ax,
               ay,
               az,
               gx,
               gy,
               gz,
               temperature);

        MPU6050_GetRollPitch(&roll, &pitch);
        printf("RPY: Roll: %.2f Pitch: %.2f\n\r\n\r", roll, pitch);
        HAL_Delay(1000);
    }
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
