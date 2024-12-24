#include "i2c_driver.hpp"
#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <cstdint>

namespace I2CDriver {

    constexpr auto I2C_TIMEOUT{1000};

    void write_words(I2CHandle const i2c_bus,
                     std::uint16_t const dev_address,
                     std::uint8_t const reg_address,
                     std::uint16_t* write_data,
                     std::uint8_t write_size) noexcept
    {}

    void write_word(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint16_t write_data) noexcept
    {
        write_words(i2c_bus, dev_address, reg_address, &write_data, 1);
    }

    void write_bytes(I2CHandle const i2c_bus,
                     std::uint16_t const dev_address,
                     std::uint8_t const reg_address,
                     std::uint8_t* write_data,
                     std::uint8_t const write_size) noexcept

    {
        HAL_I2C_Mem_Write(i2c_bus, dev_address << 1, reg_address, 1, write_data, write_size, I2C_TIMEOUT);
    }

    void write_byte(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint8_t write_data) noexcept
    {
        write_bytes(i2c_bus, dev_address, reg_address, 1, &write_data, 1);
    }

    void write_bit(I2CHandle const i2c_bus,
                   std::uint16_t const dev_address,
                   std::uint8_t const reg_address,
                   bool const write_data,
                   std::uint8_t const write_position) noexcept
    {
        std::uint8_t write = read_byte(i2c_bus, dev_address, reg_address);
        if (write_data) {
            write |= (1 << write_position);
        } else {
            write &= ~(1 << write_position);
        }

        write_byte(i2c_bus, dev_address, reg_address, write);
    }

    void write_bits(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint8_t const write_data,
                    std::uint8_t const write_position,
                    std::uint8_t const write_size) noexcept
    {
        std::uint8_t write = read_byte(i2c_bus, dev_address, reg_address);
        uint8_t mask = ((1 << write_size) - 1) << (write_position - write_size + 1);
        std::uint8_t temp = write_data;
        temp <<= (write_position - write_size + 1);
        temp &= mask;
        write &= ~(mask);
        write |= temp;

        write_byte(i2c_bus, dev_address, reg_address, write);
    }

    void read_words(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint16_t* read_data,
                    std::uint8_t const read_size) noexcept
    {
        /* TO DO- IMPLEMENT PROPER WORD READING */
        HAL_I2C_Mem_Read(i2c_bus, dev_address << 1, reg_address, 1, read_data, read_size, I2C_TIMEOUT);
    }

    std::uint16_t
    read_word(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {
        std::uint16_t read;
        read_words(i2c_bus, dev_address, reg_address, &read, 1);

        return read;
    }

    void read_bytes(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint8_t* read_data,
                    std::uint8_t const read_size) noexcept
    {
        HAL_I2C_Mem_Read(i2c_bus, dev_address << 1, reg_address, 1, read_data, read_size, I2C_TIMEOUT);
    }

    std::uint8_t
    read_byte(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {
        std::uint8_t read;
        read_bytes(i2c_bus, dev_address, reg_address, &read, 1);

        return read;
    }

    bool read_bit(I2CHandle const i2c_bus,
                  std::uint16_t const dev_address,
                  std::uint8_t const reg_address,
                  std::uint8_t const read_position) noexcept
    {
        return read_byte(i2c_bus, dev_address, reg_address) & (1 << read_position);
    }

    std::uint8_t read_bits(I2CHandle const i2c_bus,
                           std::uint16_t const dev_address,
                           std::uint8_t const reg_address,
                           std::uint8_t const read_position,
                           std::uint8_t const read_size) noexcept
    {
        std::uint8_t read = read_byte(i2c_bus, dev_address, reg_address);
        std::uint8_t mask = ((1 << read_size) - 1) << (read_position - read_size + 1);
        read &= mask;

        return read >> (read_position - read_size + 1);
    }

}; // namespace I2CDriver