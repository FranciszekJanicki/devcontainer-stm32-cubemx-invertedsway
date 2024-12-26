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
            this->w = this->w * right.w - this->x * right.x - this->y * right.y - this->z * right.z;
            this->x = this->w * right.x + this->x * right.w + this->y * right.z - this->z * right.y;
            this->y = this->w * right.y - this->x * right.z + this->y * right.w + this->z * right.x;
            this->z = this->w * right.z + this->x * right.y - this->y * right.x + this->z * right.w;
            return *this;
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