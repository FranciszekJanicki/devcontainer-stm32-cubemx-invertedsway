#ifndef UTILITY_HPP
#define UTILITY_HPP

#include "arithmetic.hpp"
#include "common.hpp"
#include <array>
#include <bitset>
#include <cstdint>

namespace Utility {

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
    [[nodiscard]] inline Bytes<NUM_BITS / 8> bits_to_bytes(Bits<NUM_BITS> const& bits) noexcept
    {
        static_assert(NUM_BITS % 8 == 0);
        Bytes<NUM_BITS / 8> bytes{};
        for (std::size_t i{}; i < bytes.size(); ++i) {
            for (std::size_t j{}; j < 8; ++j) {
                if (bits[i * 8 + j]) {
                    bytes[i] |= (1 << j);
                }
            }
        }
        return bytes;
    }

    template <std::size_t NUM_BYTES>
    [[nodiscard]] inline Bits<8 * NUM_BYTES> bytes_to_bits(Bytes<NUM_BYTES> const& bytes) noexcept
    {
        Bits<8 * NUM_BYTES> bits{};
        for (std::size_t i{}; i < bytes.size(); ++i) {
            for (std::size_t j{}; j < 8; ++j) {
                bits[i * 8 + j] = (bytes[i] & (1 << j)) != 0;
            }
        }
        return bits;
    }

    template <std::size_t NUM_BYTES>
    [[nodiscard]] inline Words<NUM_BYTES / 2> bytes_to_words(Bytes<NUM_BYTES> const& bytes) noexcept
    {
        static_assert(NUM_BYTES % 2 == 0);
        Words<NUM_BYTES / 2> words{};
        for (std::size_t i{}; i < words.size(); ++i) {
            words[i] = static_cast<Word>(bytes[2 * i] << 8) | static_cast<Word>(bytes[2 * i + 1]);
        }
        return words;
    }

    template <std::size_t NUM_WORDS>
    [[nodiscard]] inline Bytes<2 * NUM_WORDS> words_to_bytes(Words<NUM_WORDS> const& words) noexcept
    {
        Bytes<2 * NUM_WORDS> bytes{};
        for (std::size_t i{}; i < words.size(); ++i) {
            bytes[2 * i] = static_cast<Byte>(words[i] >> 8);
            bytes[2 * i + 1] = static_cast<Byte>(words[i]);
        }
        return bytes;
    }

    template <std::size_t NUM_WORDS>
    [[nodiscard]] inline DWords<NUM_WORDS / 2> words_to_dwords(Words<NUM_WORDS> const& words) noexcept
    {
        static_assert(NUM_WORDS % 2 == 0);

        DWords<NUM_WORDS / 2> dwords{};
        for (std::size_t i{}; i < dwords.size(); ++i) {
            dwords[i] = static_cast<DWord>(words[2 * i] << 16) | static_cast<DWord>(words[2 * i + 1]);
        }
        return dwords;
    }

    template <std::size_t NUM_DWORDS>
    [[nodiscard]] inline Words<2 * NUM_DWORDS> dwords_to_words(DWords<NUM_DWORDS> const& dwords) noexcept
    {
        Words<2 * NUM_DWORDS> words{};
        for (std::size_t i{}; i < words.size(); ++i) {
            words[2 * i] = static_cast<Word>(dwords[i] >> 16);
            words[2 * i + 1] = static_cast<Word>(dwords[i]);
        }
        return words;
    }

    template <std::size_t NUM_WORDS>
    [[nodiscard]] inline Bits<16 * NUM_WORDS> words_to_bits(Words<NUM_WORDS> const& words) noexcept
    {
        return bytes_to_bits(words_to_bytes(words));
    }

    template <std::size_t NUM_DWORDS>
    [[nodiscard]] inline Bits<32 * NUM_DWORDS> dwords_to_bits(DWords<NUM_DWORDS> const& dwords) noexcept
    {
        return words_to_bits(dwords_to_words(dwords));
    }

    template <std::size_t NUM_DWORDS>
    [[nodiscard]] inline Bytes<4 * NUM_DWORDS> dwords_to_bytes(DWords<NUM_DWORDS> const& dwords) noexcept
    {
        return words_to_bytes(dwords_to_words(dwords));
    }

    template <std::size_t NUM_BITS>
    [[nodiscard]] inline Words<NUM_BITS / 16> bits_to_words(Bits<NUM_BITS> const& bits) noexcept
    {
        return bytes_to_words(bits_to_bytes(bits));
    }

    template <std::size_t NUM_BITS>
    [[nodiscard]] inline DWords<NUM_BITS / 32> bits_to_dwords(Bits<NUM_BITS> const& bits) noexcept
    {
        return words_to_dwords(bits_to_words(bits));
    }

    template <std::size_t NUM_BYTES>
    [[nodiscard]] inline DWords<NUM_BYTES / 4> bytes_to_dwords(Bytes<NUM_BYTES> const& bytes) noexcept
    {
        return words_to_dwords(bytes_to_words(bytes));
    }

    template <Linalg::Arithmetic Value>
    [[nodiscard]] inline Value degrees_to_radians(Value const degrees) noexcept
    {
        return degrees * 3.1416F / 180.0F;
    }

    template <Linalg::Arithmetic Value>
    [[nodiscard]] inline Value radians_to_degrees(Value const radians) noexcept
    {
        return radians * 180.0F / 3.1416F;
    }

}; // namespace Utility

#endif // UTILITY_HPP