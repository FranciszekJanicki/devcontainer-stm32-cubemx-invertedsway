#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "utility.hpp"
#include <array>
#include <bitset>
#include <cstdint>
#include <utility>

using namespace Utility;

namespace InvertedSway {

    struct I2CDevice {
    public:
        I2CDevice() noexcept = default;
        I2CDevice(I2CBusHandle const i2c_bus, std::uint16_t const device_address) noexcept;

        I2CDevice(I2CDevice const& other) noexcept = delete;
        I2CDevice(I2CDevice&& other) noexcept = default;

        I2CDevice& operator=(I2CDevice const& other) noexcept = delete;
        I2CDevice& operator=(I2CDevice&& other) noexcept = default;

        ~I2CDevice() noexcept = default;

        template <std::size_t READ_SIZE>
        DWords<READ_SIZE> read_dwords(std::uint8_t const reg_address) const noexcept
        {
            return bytes_to_dwords(this->read_bytes<4 * READ_SIZE>(reg_address));
        }

        DWord read_dword(std::uint8_t const reg_address) const noexcept;

        template <std::size_t READ_SIZE>
        Words<READ_SIZE> read_words(std::uint8_t const reg_address) const noexcept
        {
            return bytes_to_words(this->read_bytes<2 * READ_SIZE>(reg_address));
        }

        Word read_word(std::uint8_t const reg_address) const noexcept;

        template <std::size_t READ_SIZE>
        Bytes<READ_SIZE> read_bytes(std::uint8_t const reg_address) const noexcept
        {
            if (this->initialized_) {
                Bytes<READ_SIZE> read{};
                HAL_I2C_Mem_Read(this->i2c_bus_,
                                 this->device_address_ << 1,
                                 reg_address,
                                 sizeof(reg_address),
                                 read.data(),
                                 read.size(),
                                 I2C_TIMEOUT);
                return read;
            }
            std::unreachable();
        }

        void
        read_bytes(std::uint8_t const reg_address, Byte* const read_data, std::size_t const read_size) const noexcept;

        Byte read_byte(std::uint8_t const reg_address) const noexcept;

        Bit read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept;

        template <std::size_t READ_SIZE>
        Bits<READ_SIZE> read_bits(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            Byte read = this->read_byte(reg_address);
            Byte mask = ((1 << READ_SIZE) - 1) << (read_position - READ_SIZE + 1);
            read &= mask;
            read >>= (read_position - READ_SIZE + 1);
            return read;
        }

        Byte read_bits(std::uint8_t const reg_address,
                       std::uint8_t const read_position,
                       std::size_t const read_size) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
        {
            this->write_bytes(reg_address, dwords_to_bytes(write_data));
        }

        void write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept
        {
            this->write_bytes(reg_address, words_to_bytes(write_data));
        }

        void write_word(std::uint8_t const reg_address, Word const write_data) const noexcept;

        template <std::size_t WRITE_SIZE>
        void write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept
        {
            if (!this->initialized_) {
                Bytes<WRITE_SIZE> write{write_data};
                HAL_I2C_Mem_Write(this->i2c_bus_,
                                  this->device_address_ << 1,
                                  reg_address,
                                  sizeof(reg_address),
                                  write.data(),
                                  write.size(),
                                  I2C_TIMEOUT);
            }
        }

        void write_bytes(std::uint8_t const reg_address,
                         Byte* const write_data,
                         std::size_t const write_size) const noexcept;

        void write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept;

        void write_bit(std::uint8_t const reg_address,
                       Bit const write_data,
                       std::uint8_t const write_position) const noexcept;

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
                        std::size_t const write_size) const noexcept;

        std::uint16_t device_address() const noexcept
        {
            return this->device_address_;
        }

    private:
        static constexpr std::uint32_t I2C_TIMEOUT{100U};
        static constexpr std::uint32_t I2C_SCAN_RETRIES{10U};

        void initialize() noexcept;

        bool initialized_{false};

        I2CBusHandle i2c_bus_{nullptr};
        std::uint16_t device_address_{};
    };

}; // namespace InvertedSway

#endif // I2C_DEVICE_HPP