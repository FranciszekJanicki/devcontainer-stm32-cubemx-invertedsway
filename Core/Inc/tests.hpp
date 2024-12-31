#ifndef TESTS_HPP
#define TESTS_HPP

namespace Tests {

    void MOTOR_TEST() noexcept;

    void MOTOR_BOOST_TEST() noexcept;

    void MOTOR_DRIVER_TEST() noexcept;

    void MPU_TEST() noexcept;

    void MPU_DMP_TEST() noexcept;

    void KALMAN_TEST() noexcept;

    void ENCODER_TEST() noexcept;

    // #include "usart.h"
    // #include <cstdio>
    // #include <vector>

    // #define fn auto
    // #define let auto const
    // #define vec std::vector<double>
    // #define let_mut auto
    // #define println(x) printf("%d\n\r", x)
    // #define in :

    //     fn range(let min, let max, let N) -> vec
    //     {
    //         let_mut result = vec();
    //         let delta = (max - min) / (N - 1);
    //         for (let_mut i = 0; i < N; ++i) {
    //             result.push_back(min + i * delta);
    //         }
    //         return result;
    //     }

    //     fn DUTKIEWICZ_TEST() noexcept
    //     {
    //         MX_USART2_UART_Init();

    //         let dutkiewicz = 1000;

    //         for (let_mut dudek in range(0, dutkiewicz, 100)) {
    //             println("%d\n\r", dudek);
    //         }
    //     }

}; // namespace Tests

#endif // TESTS_HPP