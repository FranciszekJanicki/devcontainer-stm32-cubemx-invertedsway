#ifndef QUATERNION3D_HPP
#define QUATERNION3D_HPP

#include "arithmetic.hpp"
#include <cmath>
#include <compare>
#include <cstdlib>
#include <tuple>
#include <utility>

namespace Linalg {

    template <Arithmetic Value>
    struct Quaternion3D {
        [[nodiscard]] constexpr Quaternion3D conjugated() const noexcept
        {
            return Quaternion3D{w, -x, -y, -z};
        }

        constexpr void conjugate() noexcept
        {
            x = -x;
            y = -y;
            z = -z;
        }

        [[nodiscard]] constexpr auto magnitude() const noexcept
        {
            return std::sqrt(std::pow(w, 2) + std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
        }

        [[nodiscard]] constexpr Quaternion3D normalized() const noexcept
        {
            const auto im{static_cast<Value>(1) / magnitude()};
            return Quaternion3D{w * im, x * im, y * im, z * im};
        }

        constexpr void normalize() noexcept
        {
            const auto im{static_cast<Value>(1) / magnitude()};
            w *= im;
            x *= im;
            y *= im;
            z *= im;
        }

        constexpr Quaternion3D& operator+=(Quaternion3D const& other) noexcept
        {
            this->w += other.w;
            this->x += other.x;
            this->y += other.y;
            this->z += other.z;
            return *this;
        }

        constexpr Quaternion3D& operator-=(Quaternion3D const& other) noexcept
        {
            this->w -= other.w;
            this->x -= other.x;
            this->y -= other.y;
            this->z -= other.z;
            return *this;
        }

        constexpr Quaternion3D& operator*=(Quaternion3D const& other) noexcept
        {
            this->w = this->w * other.w - this->x * other.x - this->y * other.y - this->z * other.z;
            this->x = this->w * other.x + this->x * other.w + this->y * other.z - this->z * other.y;
            this->y = this->w * other.y - this->x * other.z + this->y * other.w + this->z * other.x;
            this->z = this->w * other.z + this->x * other.y - this->y * other.x + this->z * other.w;
            return *this;
        }

        template <Arithmetic Converted>
        [[nodiscard]] explicit constexpr operator Quaternion3D<Converted>() const noexcept
        {
            return Quaternion3D<Converted>{static_cast<Converted>(this->w),
                                           static_cast<Converted>(this->x),
                                           static_cast<Converted>(this->y),
                                           static_cast<Converted>(this->z)};
        }

        [[nodiscard]] constexpr bool operator<=>(Quaternion3D const& other) const noexcept = default;

        Value w{};
        Value x{};
        Value y{};
        Value z{};
    };

    template <Arithmetic Value>
    constexpr auto operator+(Quaternion3D<Value> const& left, Quaternion3D<Value> const& right) noexcept
    {
        return Quaternion3D<Value>{left.w + right.w, left.x + right.x, left.y + right.y, left.z + right.z};
    }

    template <Arithmetic Value>
    constexpr auto operator-(Quaternion3D<Value> const& left, Quaternion3D<Value> const& right) noexcept
    {
        return Quaternion3D<Value>{left.w - right.w, left.x - right.x, left.y - right.y, left.z + right.z};
    }

    template <Arithmetic Value>
    constexpr auto operator*(Quaternion3D<Value> const& left, Quaternion3D<Value> const& right) noexcept
    {
        return Quaternion3D<Value>{left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
                                   left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
                                   left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
                                   left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w};
    }

}; // namespace Linalg

#endif // QUATERNION3D_HPP