#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "utility.hpp"
#include <array>
#include <bitset>
#include <cstdint>

using namespace Utility;

namespace InvertedSway {

    struct I2CDevice {
        template <std::size_t READ_SIZE>
        [[nodiscard]] DWords<READ_SIZE> read_dwords(std::uint8_t const reg_address) const noexcept
        {
            return bytes_to_dwords(this->read_bytes<4 * READ_SIZE>(reg_address));
        }

        [[nodiscard]] DWord read_dword(std::uint8_t const reg_address) const noexcept
        {
            return this->read_dwords<1>(reg_address)[0];
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] Words<READ_SIZE> read_words(std::uint8_t const reg_address) const noexcept
        {
            return bytes_to_words(this->read_bytes<2 * READ_SIZE>(reg_address));
        }

        [[nodiscard]] Word read_word(std::uint8_t const reg_address) const noexcept
        {
            return this->read_words<1>(reg_address)[0];
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] Bytes<READ_SIZE> read_bytes(std::uint8_t const reg_address) const noexcept
        {
            Bytes<READ_SIZE> read;
            HAL_I2C_Mem_Read(this->i2c_bus,
                             this->device_address << 1,
                             reg_address,
                             sizeof(reg_address),
                             read.data(),
                             read.size(),
                             I2C_TIMEOUT);
            return read;
        }

        void read_bytes(std::uint8_t const reg_address, Byte* read_data, std::size_t const read_size) const noexcept
        {
            HAL_I2C_Mem_Read(this->i2c_bus,
                             this->device_address << 1,
                             reg_address,
                             sizeof(reg_address),
                             read_data,
                             read_size,
                             I2C_TIMEOUT);
        }

        [[nodiscard]] Byte read_byte(std::uint8_t const reg_address) const noexcept
        {
            return this->read_bytes<1>(reg_address)[0];
        }

        [[nodiscard]] Bit read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            return this->read_byte(reg_address) & (1 << read_position);
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] Bits<READ_SIZE> read_bits(std::uint8_t const reg_address,
                                                std::uint8_t const read_position) const noexcept
        {
            Byte read = this->read_byte(reg_address);
            Byte mask = ((1 << READ_SIZE) - 1) << (read_position - READ_SIZE + 1);
            read &= mask;
            read >>= (read_position - READ_SIZE + 1);
            return read;
        }

        [[nodiscard]] Byte read_bits(std::uint8_t const reg_address,
                                     std::uint8_t const read_position,
                                     std::size_t const read_size) const noexcept
        {
            Byte read = this->read_byte(reg_address);
            Byte mask = ((1 << read_size) - 1) << (read_position - read_size + 1);
            read &= mask;
            read >>= (read_position - read_size + 1);
            return read;
        }

        template <std::size_t WRITE_SIZE>
        void write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
        {
            this->write_bytes(reg_address, dwords_to_bytes(write_data));
        }

        void write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept
        {
            this->write_dwords(reg_address, DWords<1>{write_data});
        }

        template <std::size_t WRITE_SIZE>
        void write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept
        {
            this->write_bytes(reg_address, words_to_bytes(write_data));
        }

        void write_word(std::uint8_t const reg_address, Word const write_data) const noexcept
        {
            this->write_words(reg_address, Words<1>{write_data});
        }

        template <std::size_t WRITE_SIZE>
        void write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept
        {
            Bytes<WRITE_SIZE> write{write_data};
            HAL_I2C_Mem_Write(this->i2c_bus,
                              this->device_address << 1,
                              reg_address,
                              sizeof(reg_address),
                              write.data(),
                              write.size(),
                              I2C_TIMEOUT);
        }

        void write_bytes(std::uint8_t const reg_address, Byte* write_data, std::size_t const write_size) const noexcept
        {
            HAL_I2C_Mem_Write(this->i2c_bus,
                              this->device_address << 1,
                              reg_address,
                              sizeof(reg_address),
                              write_data,
                              write_size,
                              I2C_TIMEOUT);
        }

        void write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept
        {
            this->write_bytes(reg_address, Bytes<1>{write_data});
        }

        void write_bit(std::uint8_t const reg_address,
                       Bit const write_data,
                       std::uint8_t const write_position) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            if (write_data) {
                write |= (1 << write_position);
            } else {
                write &= ~(1 << write_position);
            }
            this->write_byte(reg_address, write);
        }

        template <std::size_t WRITE_SIZE>
        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            Byte mask = ((1 << WRITE_SIZE) - 1) << (write_position - WRITE_SIZE + 1);
            Byte temp = write_data << (write_position - WRITE_SIZE + 1);
            temp &= mask;
            write &= ~(mask);
            write |= temp;
            this->write_byte(reg_address, write);
        }

        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position,
                        std::size_t const write_size) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            Byte mask = ((1 << write_size) - 1) << (write_position - write_size + 1);
            Byte temp = write_data << (write_position - write_size + 1);
            temp &= mask;
            write &= ~(mask);
            write |= temp;
            this->write_byte(reg_address, write);
        }

        I2CBusHandle i2c_bus{nullptr};
        std::uint16_t device_address{};

        static constexpr auto I2C_TIMEOUT{1000};
    };

}; // namespace InvertedSway

#endif // I2C_DEVICE_HPP