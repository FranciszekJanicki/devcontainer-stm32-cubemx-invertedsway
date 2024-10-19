#ifndef ARITHMETIC_HPP
#define ARITHMETIC_HPP

#include <concepts>
#include <type_traits>

namespace linalg {

template <typename type>
concept arithmetic = std::is_arithmetic_v<type>;

}; // namespace linalg

#endif // ARITHMETIC_HPP