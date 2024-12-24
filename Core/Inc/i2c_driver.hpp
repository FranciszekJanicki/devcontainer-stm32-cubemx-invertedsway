#ifndef I2C_DRIVER_HPP
#define I2C_DRIVER_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <cstdint>

namespace I2CDriver {

    using namespace InvertedSway;

    void write_words(I2CHandle const i2c_bus,
                     std::uint16_t const dev_address,
                     std::uint8_t const reg_address,
                     std::uint16_t* write_data,
                     std::uint8_t write_size) noexcept;

    void write_word(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint16_t write_data) noexcept;

    void write_bytes(I2CHandle const i2c_bus,
                     std::uint16_t const dev_address,
                     std::uint8_t const reg_address,
                     std::uint8_t* write_data,
                     std::uint8_t const write_size) noexcept;

    void write_byte(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint8_t write_data) noexcept;

    void write_bit(I2CHandle const i2c_bus,
                   std::uint16_t const dev_address,
                   std::uint8_t const reg_address,
                   bool const write_data,
                   std::uint8_t const write_position) noexcept;

    void write_bits(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint8_t const write_data,
                    std::uint8_t const write_position,
                    std::uint8_t const write_size) noexcept;

    void read_words(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint16_t* read_data,
                    std::uint8_t const read_size) noexcept;

    std::uint16_t
    read_word(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept;

    void read_bytes(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint8_t* read_data,
                    std::uint8_t const read_size) noexcept;

    std::uint8_t
    read_byte(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept;

    bool read_bit(I2CHandle const i2c_bus,
                  std::uint16_t const dev_address,
                  std::uint8_t const reg_address,
                  std::uint8_t const read_position) noexcept;

    std::uint8_t read_bits(I2CHandle const i2c_bus,
                           std::uint16_t const dev_address,
                           std::uint8_t const reg_address,
                           std::uint8_t const read_position,
                           std::uint8_t const read_size) noexcept;

}; // namespace I2CDriver

#endif // I2C_DRIVER_HPP