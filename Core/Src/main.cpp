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

enum struct Event {
    TIMER_ELAPSED,
    PRINT_DUTKIEWICZ,
    PRINT_DUPA,
    PRINT_KARDYS,
    PRINT_BERNAT,
    PRINT_BOLTRUKIEWICZ,
    NONE,
};

struct EventHandler {
    void set_event(const Event event) noexcept
    {
        this->event = event;
    }

    [[nodiscard]] Event get_event() noexcept
    {
        return std::exchange(this->event, Event::NONE);
    }

    Event event{Event::NONE};
};

static EventHandler event_handler{};

namespace BetterMain {

    using Arguments = std::span<const char*>;

    template <typename Value>
    [[nodiscard]] constexpr auto main(const Arguments arguments) noexcept(false) -> decltype(auto)
        requires(std::is_integral_v<Value>)
    {
        [[assume(true == true)]]

        using namespace InvertedSway;

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
                        MPU6050::make_gyro_filter(),
                        MPU6050::make_accel_filter()};

        auto kalman{make_kalman<MPU6050::Scaled>(0.0, 0.0, 0.0, 0.0, 0.0)};

        auto regulator{make_regulator<MPU6050::Scaled>(RegulatorAlgo::PID, 0.0, 0.0, 0.0, 0.0)};

        Encoder encoder{&htim1};

        System system{std::move(mpu6050),
                      std::move(l298n),
                      std::move(kalman),
                      std::move(regulator),
                      std::move(encoder)};

        const MPU6050::Scaled angle_degrees{0};

        // timer2 is set to meausre 1000 ms (counter period is 62499)
        const MPU6050::Scaled sampling_time{1};

        HAL_TIM_Base_Start_IT(&htim2);

        while (true) {
            switch (event_handler.get_event()) {
                case Event::TIMER_ELAPSED: {
                    system.balance_sway(angle_degrees, sampling_time);
                    break;
                }
                case Event::PRINT_DUTKIEWICZ: {
                    printf("DUTKIEWICZ\n\r");
                    break;
                }
                case Event::PRINT_BERNAT: {
                    printf("BERNAT\n\r");
                    break;
                }
                case Event::PRINT_KARDYS: {
                    printf("KARDYS\n\r");
                    break;
                }
                case Event::PRINT_BOLTRUKIEWICZ: {
                    printf("BOLTRUKIEWICZ\n\r");
                    break;
                }
                case Event::PRINT_DUPA: {
                    printf("DUPA\n\r");
                    break;
                }
                case Event::NONE: {
                    break;
                }
                default: {
                    break;
                }
            }
        }

        return Value{0};
    }

}; // namespace BetterMain

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == nullptr) {
        return;
    }
    if (htim->Instance == TIM2) {
        event_handler.set_event(Event::TIMER_ELAPSED);
        HAL_TIM_Base_Start_IT(htim);
    }
}

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
