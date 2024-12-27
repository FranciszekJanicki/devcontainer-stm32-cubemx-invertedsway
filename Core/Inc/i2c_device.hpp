#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <array>
#include <bitset>
#include <cstdint>

namespace InvertedSway {

    struct I2CDevice {
        using Bit = bool;
        using Byte = std::uint8_t;
        using Word = std::uint16_t;
        using DWord = std::uint32_t;

        template <std::size_t SIZE>
        using Bits = std::bitset<SIZE>;
        template <std::size_t SIZE>
        using Bytes = std::array<Byte, SIZE>;
        template <std::size_t SIZE>
        using Words = std::array<Word, SIZE>;
        template <std::size_t SIZE>
        using DWords = std::array<DWord, SIZE>;

        template <std::size_t NUM_BITS>
        static Bytes<NUM_BITS / 8> bits_to_bytes(Bits<NUM_BITS> const& bits) noexcept
        {
            static_assert(NUM_BITS % 8 == 0);

            Bytes<NUM_BITS / 8> bytes{};
            for (auto i{0}; i < bytes.size(); ++i) {
                for (auto j{0}; j < 8; ++j) {
                    bytes[i] |= (1 << bits[i * j]);
                }
            }
            return bytes;
        }

        template <std::size_t NUM_BYTES>
        static Bits<8 * NUM_BYTES> bytes_to_bits(Bytes<NUM_BYTES> const& bytes) noexcept
        {
            Bits<8 * NUM_BYTES> bits{};
            for (auto i{0}; i < bits.size(); ++i) {
                for (auto j{0}; j < 8; ++j) {
                    bits[i * j] = bytes[i] & (1 << j);
                }
            }
            return bits;
        }

        template <std::size_t NUM_BYTES>
        static Words<NUM_BYTES / 2> bytes_to_words(Bytes<NUM_BYTES> const& bytes) noexcept
        {
            static_assert(NUM_BYTES % 2 == 0);

            Words<NUM_BYTES / 2> words{};
            for (auto i{0}; i < words.size(); ++i) {
                words[i] = static_cast<Word>(bytes[i] << 8) | static_cast<Word>(bytes[i + 1]);
            }
            return words;
        }

        template <std::size_t NUM_WORDS>
        static Bytes<2 * NUM_WORDS> words_to_bytes(Words<NUM_WORDS> const& words) noexcept
        {
            Bytes<2 * NUM_WORDS> bytes{};
            for (auto i{0}; i < bytes.size(); ++i) {
                bytes[i] = static_cast<Byte>(words[i] >> 8);
                bytes[i + 1] = static_cast<Byte>(words[i]);
            }
            return bytes;
        }

        template <std::size_t NUM_WORDS>
        static DWords<NUM_WORDS / 2> words_to_dwords(Words<NUM_WORDS> const& words) noexcept
        {
            static_assert(NUM_WORDS % 2 == 0);

            DWords<NUM_WORDS / 2> dwords{};
            for (auto i{0}; i < dwords.size(); ++i) {
                dwords[i] = static_cast<DWord>(words[i] << 16) | static_cast<DWord>(words[i + 1]);
            }
            return dwords;
        }

        template <std::size_t NUM_DWORDS>
        static Words<2 * NUM_DWORDS> dwords_to_words(DWords<NUM_DWORDS> const& dwords) noexcept
        {
            Words<2 * NUM_DWORDS> words{};
            for (auto i{0}; i < words.size(); ++i) {
                words[i] = static_cast<Word>(words[i] >> 16);
                words[i + 1] = static_cast<Word>(words[i]);
            }
            return words;
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] DWords<READ_SIZE> read_dwords(std::uint8_t const reg_address) const noexcept
        {
            return words_to_dwords(bytes_to_words(this->read_bytes<2 * 2 * READ_SIZE>(reg_address)));
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
            return this->read_bytes<1>(reg_address).front();
        }

        [[nodiscard]] Bit read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            return this->read_byte(reg_address) & (1 << read_position);
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] Byte read_bits(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            Byte read = this->read_byte(reg_address);
            Byte mask = ((1 << READ_SIZE) - 1) << (read_position - READ_SIZE + 1);
            read &= mask;
            return read >> (read_position - READ_SIZE + 1);
        }

        [[nodiscard]] Byte read_bits(std::uint8_t const reg_address,
                                     std::uint8_t const read_position,
                                     std::size_t const read_size) const noexcept
        {
            Byte read = this->read_byte(reg_address);
            Byte mask = ((1 << read_size) - 1) << (read_position - read_size + 1);
            read &= mask;
            return read >> (read_position - read_size + 1);
        }

        template <std::size_t WRITE_SIZE>
        void write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
        {
            this->write_bytes(reg_address, words_to_bytes(dwords_to_words(write_data)));
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
            write_byte(reg_address, write);
        }

        template <std::size_t WRITE_SIZE>
        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
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

        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position,
                        std::size_t const write_size) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            Byte mask = ((1 << write_size) - 1) << (write_position - write_size + 1);
            Byte temp = write_data;
            temp <<= (write_position - write_size + 1);
            temp &= mask;
            write &= ~(mask);
            write |= temp;
            write_byte(reg_address, write);
        }

        I2CBusHandle i2c_bus{nullptr};
        std::uint16_t device_address{};

        static constexpr auto I2C_TIMEOUT{100};
    };

}; // namespace InvertedSway

#endif // I2C_DEVICE_HPP