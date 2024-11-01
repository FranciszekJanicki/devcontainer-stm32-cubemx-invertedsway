#ifndef L298N_HPP
#define L298N_HPP

#include "common.hpp"
#include "motor.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <expected>
#include <utility>

struct L298N {
public:
    enum struct Channel {
        CHANNEL1,
        CHANNEL2,
    };

    using Raw = Motor::Raw;
    using Speed = std::double_t;
    using Voltage = std::double_t;
    using Torque = std::double_t;
    using Direction = Motor::Direction;
    using Error = Motor::Error;
    using ExpectedDirection = Motor::ExpectedDirection;
    using ExpectedRaw = Motor::ExpectedRaw;
    using ExpectedVoltage = std::expected<Voltage, Error>;
    using ExpectedSpeed = std::expected<Speed, Error>;
    using ExpectedTorque = std::expected<Torque, Error>;
    using Unexpected = Motor::Unexpected;
    using MotorChannel = std::pair<Channel, Motor>;
    using MotorChannels = std::array<MotorChannel, 2>;
    MotorChannels static constexpr Voltage MAX_VOLTAGE_V{12};
    static constexpr Voltage MIN_VOLTAGE_V{0};

    static constexpr Speed MIN_SPEED_RPM{0};
    static constexpr Speed MAX_SPEED_RPM{1000};

    static constexpr Speed MIN_TORQUE_NM{0};
    static constexpr Speed MAX_TORQUE_NM{1000};

    L298N(const MotorChannels& motor_channels) noexcept;
    L298N(MotorChannels&& motor_channels) noexcept;

    L298N(const L298N& other) noexcept = delete;
    L298N(L298N&& other) noexcept = delete;

    L298N& operator=(const L298N& other) noexcept = delete;
    L298N& operator=(L298N&& other) noexcept = delete;

    ~L298N() noexcept;

    const MotorChannels& motor_channels() const& noexcept;
    MotorChannels&& motor_channels() && noexcept;

    void motor_channels(const MotorChannels& motors) noexcept;
    void motor_channels(MotorChannels&& motors) noexcept;

    [[nodiscard]] ExpectedRaw get_compare_raw(const Channel channel) const noexcept;
    [[nodiscard]] Error set_compare_raw(const Channel channel, const Raw raw) const noexcept;

    [[nodiscard]] ExpectedVoltage get_compare_voltage(const Channel channel) const noexcept;
    [[nodiscard]] Error set_compare_voltage(const Channel channel, const Voltage voltage) const noexcept;

    [[nodiscard]] ExpectedSpeed get_compare_speed(const Channel channel) const noexcept;
    [[nodiscard]] Error set_compare_speed(const Channel channel, const Speed speed) const noexcept;

    [[nodiscard]] ExpectedTorque get_compare_torque(const Channel channel) const noexcept;
    [[nodiscard]] Error set_compare_torque(const Channel channel, const Torque torque) const noexcept;

    [[nodiscard]] Error set_direction(const Channel channel, const Direction direction) const noexcept;
    [[nodiscard]] ExpectedDirection get_direction(const Channel channel) const noexcept;

    [[nodiscard]] Error set_forward(const Channel channel) const noexcept;
    [[nodiscard]] Error set_backward(const Channel channel) const noexcept;

    [[nodiscard]] Error set_soft_stop(const Channel channel) const noexcept;
    [[nodiscard]] Error set_fast_stop(const Channel channel) const noexcept;

    [[nodiscard]] Error toggle_direction(const Channel channel) const noexcept;

private:
    static Speed raw_to_speed(const Raw raw) noexcept;
    static Raw speed_to_raw(const Speed speed) noexcept;

    static Voltage raw_to_voltage(const Raw raw) noexcept;
    static Raw voltage_to_raw(const Voltage voltage) noexcept;

    static Torque raw_to_torque(const Raw raw) noexcept;
    static Raw torque_to_raw(const Torque torque) noexcept;

    const Motor& get_motor(const Channel channel) const noexcept;
    Motor& get_motor(const Channel channel) noexcept;

    MotorChannels motor_channels_{};
};

#endif // L298N_HPP