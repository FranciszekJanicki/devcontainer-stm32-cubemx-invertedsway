#ifndef I2C_DRIVER_SIGMA_HPP
#define I2C_DRIVER_SIGMA_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <array>
#include <bitset>
#include <cstdint>

namespace InvertedSway {

    struct I2CDevice {
    public:
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

        /* C++ style overloads */

        template <std::size_t READ_SIZE>
        DWords<READ_SIZE> read_dwords(std::uint8_t const reg_address) const noexcept
        {}

        DWord read_dword(std::uint8_t const reg_address) const noexcept
        {
            return this->read_dwords<1>(reg_address).front();
        }

        template <std::size_t READ_SIZE>
        Words<READ_SIZE> read_words(std::uint8_t const reg_address) const noexcept
        {}

        Word read_word(std::uint8_t const reg_address) const noexcept
        {
            return this->read_words<1>(reg_address).front();
        }

        template <std::size_t READ_SIZE>
        Bytes<READ_SIZE> read_bytes(std::uint8_t const reg_address) const noexcept
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

        Byte read_byte(std::uint8_t const reg_address) const noexcept
        {
            return this->read_bytes<1>(reg_address).front();
        }

        Bit read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            return this->read_byte(reg_address) & (1 << read_position);
        }

        template <std::size_t READ_SIZE>
        Bits<READ_SIZE> read_bits(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            Byte read = this->read_byte(reg_address);
            Byte mask = ((1 << READ_SIZE) - 1) << (read_position - READ_SIZE + 1);
            read &= mask;

            return read >> (read_position - READ_SIZE + 1);
        }

        template <std::size_t WRITE_SIZE>
        void write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> write_data) const noexcept
        {}

        void write_dword(std::uint8_t const reg_address, DWord write_data) const noexcept
        {
            this->write_dwords(reg_address, DWords<1>{write_data});
        }

        template <std::size_t WRITE_SIZE>
        void write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> write_data) const noexcept
        {}

        void write_word(std::uint8_t const reg_address, Word write_data) const noexcept
        {
            this->write_words(reg_address, Words<1>{write_data});
        }

        template <std::size_t WRITE_SIZE>
        void write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> write_data) const noexcept
        {
            HAL_I2C_Mem_Write(this->i2c_bus,
                              this->device_address << 1,
                              reg_address,
                              sizeof(reg_address),
                              write_data.data(),
                              write_data.size(),
                              I2C_TIMEOUT);
        }

        void write_byte(std::uint8_t const reg_address, Byte write_data) const noexcept
        {
            this->write_bytes(reg_address, Bytes<1>{write_data});
        }

        void write_bit(std::uint8_t const reg_address, Bit write_data, std::uint8_t const write_position) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            if (write_data) {
                write |= (1 << write_position);
            } else {
                write &= ~(1 << write_position);
            }

            write_byte(reg_address, write);
        }

        template <std::size_t WRITE_SIZE>
        void write_bits(std::uint8_t const reg_address,
                        Bits<WRITE_SIZE> write_data,
                        std::uint8_t const write_position) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            Byte mask = ((1 << WRITE_SIZE) - 1) << (write_position - WRITE_SIZE + 1);
            Byte temp = write_data;
            temp <<= (write_position - WRITE_SIZE + 1);
            temp &= mask;
            write &= ~(mask);
            write |= temp;

            this->write_byte(reg_address, write);
        }

        void
        write_dwords(std::uint8_t const reg_address, DWord* write_data, std::size_t const write_size) const noexcept
        {}

        void write_words(std::uint8_t const reg_address, Word* write_data, std::size_t const write_size) const noexcept
        {}

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

        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position,
                        std::size_t const write_size) const noexcept
        {
            std::uint8_t write = this->read_byte(reg_address);
            uint8_t mask = ((1 << write_size) - 1) << (write_position - write_size + 1);
            std::uint8_t temp = write_data;
            temp <<= (write_position - write_size + 1);
            temp &= mask;
            write &= ~(mask);
            write |= temp;

            write_byte(reg_address, write);
        }

        void read_dwords(std::uint8_t const reg_address, DWord* read_data, std::size_t const read_size) const noexcept
        {}

        void read_words(std::uint8_t const reg_address, Word* read_data, std::size_t const read_size) const noexcept
        {}

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

        Byte read_bits(std::uint8_t const reg_address,
                       std::uint8_t const read_position,
                       std::size_t const read_size) const noexcept
        {
            std::uint8_t read = this->read_byte(reg_address);
            std::uint8_t mask = ((1 << read_size) - 1) << (read_position - read_size + 1);
            read &= mask;

            return read >> (read_position - read_size + 1);
        }

        I2CBusHandle i2c_bus{nullptr};
        std::uint16_t device_address{};

    private:
        static constexpr auto I2C_TIMEOUT{100};

    }; // namespace I2CDriverSigma

}; // namespace InvertedSway

#endif // I2C_DRIVER_SIGMA_HPP