#ifndef I2C_DRIVER_SIGMA_HPP
#define I2C_DRIVER_SIGMA_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <array>
#include <bitset>
#include <cstdint>

namespace I2CDriverSigma {

    using namespace InvertedSway;

    using Bit = bool;
    using Byte = std::uint8_t;
    using Word = std::uint16_t;
    using DWord = std::uint32_t;

    template <std::size_t SIZE>
    using Bits = Byte;
    template <std::size_t SIZE>
    using Bytes = std::array<Byte, SIZE>;
    template <std::size_t SIZE>
    using Words = std::array<Word, SIZE>;
    template <std::size_t SIZE>
    using DWords = std::array<DWord, SIZE>;

    static constexpr auto I2C_TIMEOUT{100};

    template <std::size_t READ_SIZE>
    DWords<READ_SIZE>
    read_dwords(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {}

    DWord read_dword(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {
        return read_dwords<1UL>(i2c_bus, dev_address, reg_address).front();
    }

    template <std::size_t READ_SIZE>
    Words<READ_SIZE>
    read_words(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {}

    Word read_word(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {
        return read_words<1UL>(i2c_bus, dev_address, reg_address).front();
    }

    template <std::size_t READ_SIZE>
    Bytes<READ_SIZE>
    read_bytes(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {
        Bytes<READ_SIZE> read;
        HAL_I2C_Mem_Read(i2c_bus,
                         dev_address << 1,
                         reg_address,
                         sizeof(reg_address),
                         read.data(),
                         read.size(),
                         I2C_TIMEOUT);
        return read;
    }

    Byte read_byte(I2CHandle const i2c_bus, std::uint16_t const dev_address, std::uint8_t const reg_address) noexcept
    {
        return read_bytes<1UL>(i2c_bus, dev_address, reg_address).front();
    }

    Bit read_bit(I2CHandle const i2c_bus,
                 std::uint16_t const dev_address,
                 std::uint8_t const reg_address,
                 std::uint8_t const read_position) noexcept
    {
        return read_byte(i2c_bus, dev_address, reg_address) & (1 << read_position);
    }

    template <std::size_t READ_SIZE>
    Bits<READ_SIZE> read_bits(I2CHandle const i2c_bus,
                              std::uint16_t const dev_address,
                              std::uint8_t const reg_address,
                              std::uint8_t const read_position) noexcept
    {
        Byte read = read_byte(i2c_bus, dev_address, reg_address);
        Byte mask = ((1 << READ_SIZE) - 1) << (read_position - READ_SIZE + 1);
        read &= mask;

        return read >> (read_position - READ_SIZE + 1);
    }

    template <std::size_t WRITE_SIZE>
    void write_dwords(I2CHandle const i2c_bus,
                      std::uint16_t const dev_address,
                      std::uint8_t const reg_address,
                      DWords<WRITE_SIZE> write_data) noexcept
    {}

    void write_dword(I2CHandle const i2c_bus,
                     std::uint16_t const dev_address,
                     std::uint8_t const reg_address,
                     DWord write_data) noexcept
    {
        write_dwords(i2c_bus, dev_address, reg_address, DWords<1UL>{write_data});
    }

    template <std::size_t WRITE_SIZE>
    void write_words(I2CHandle const i2c_bus,
                     std::uint16_t const dev_address,
                     std::uint8_t const reg_address,
                     Words<WRITE_SIZE> write_data) noexcept
    {}

    void write_word(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    Word write_data) noexcept
    {
        write_words(i2c_bus, dev_address, reg_address, Words<1UL>{write_data});
    }

    template <std::size_t WRITE_SIZE>
    void write_bytes(I2CHandle const i2c_bus,
                     std::uint16_t const dev_address,
                     std::uint8_t const reg_address,
                     Bytes<WRITE_SIZE> write_data) noexcept
    {
        HAL_I2C_Mem_Write(i2c_bus,
                          dev_address << 1,
                          reg_address,
                          sizeof(reg_address),
                          write_data.data(),
                          write_data.size(),
                          I2C_TIMEOUT);
    }

    void write_byte(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    Byte write_data) noexcept
    {
        write_bytes(i2c_bus, dev_address, reg_address, Bytes<1UL>{write_data});
    }

    void write_bit(I2CHandle const i2c_bus,
                   std::uint16_t const dev_address,
                   std::uint8_t const reg_address,
                   std::uint8_t const write_position,
                   Bit write_data) noexcept
    {
        Byte write = read_byte(i2c_bus, dev_address, reg_address);
        if (write_data) {
            write |= (1 << write_position);
        } else {
            write &= ~(1 << write_position);
        }

        write_byte(i2c_bus, dev_address, reg_address, write);
    }

    template <std::size_t WRITE_SIZE>
    void write_bits(I2CHandle const i2c_bus,
                    std::uint16_t const dev_address,
                    std::uint8_t const reg_address,
                    std::uint8_t const write_position,
                    Bits<WRITE_SIZE> write_data) noexcept
    {
        Byte write = read_byte(i2c_bus, dev_address, reg_address);
        Byte mask = ((1 << WRITE_SIZE) - 1) << (write_position - WRITE_SIZE + 1);
        Byte temp = write_data;
        temp <<= (write_position - WRITE_SIZE + 1);
        temp &= mask;
        write &= ~(mask);
        write |= temp;

        write_byte(i2c_bus, dev_address, reg_address, write);
    }

}; // namespace I2CDriverSigma

#endif // I2C_DRIVER_SIGMA_HPP