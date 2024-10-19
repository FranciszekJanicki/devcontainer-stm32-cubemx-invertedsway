#ifndef VECTOR3D_HPP
#define VECTOR3D_HPP

#include "arithmetic.hpp"
#include <cmath>
#include <compare>
// #include <fmt/core.h>
#include <quaternion3d.hpp>
#include <utility>

namespace linalg {

    template <arithmetic value_type>
    struct vector3D {
        [[nodiscard]] constexpr auto distance(const vector3D& other) const noexcept
        {
            return std::sqrt(std::pow(this->x - other.x, 2) + std::pow(this->y - other.y, 2) +
                             std::pow(this->z - other.z, 2));
        }

        [[nodiscard]] constexpr auto magnitude() const noexcept
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));
        }

        [[nodiscard]] constexpr vector3D rotated(const quaternion3D<value_type>& q) const noexcept
        {
            quaternion3D p(0, x, y, z);
            p *= q;
            p *= q.conjugated();
            return vector3D{p.x, p.y, p.z};
        }

        constexpr void rotate(const quaternion3D<value_type>& q) noexcept
        {
            quaternion3D p(0, x, y, z);
            p *= q;
            p *= q.conjugated();
            x = p.x;
            y = p.y;
            z = p.z;
        }

        [[nodiscard]] constexpr vector3D normalized() const noexcept
        {
            const auto im{static_cast<value_type>(1) / magnitude()};
            return vector3D{x * im, y * im, z * im};
        }

        constexpr void normalize() noexcept
        {
            const auto im{static_cast<value_type>(1) / magnitude()};
            x *= im;
            y *= im;
            z *= im;
        }

        // constexpr void print() const noexcept
        //{
        //     fmt::print("a: {}, b: {}, c: {}, w: {}\n", x, y, z);
        // }

        constexpr vector3D& operator+=(const vector3D& other) noexcept
        {
            this->x += other.x;
            this->y += other.y;
            this->z += other.z;
            return *this;
        }

        constexpr vector3D& operator-=(const vector3D& other) noexcept
        {
            this->x -= other.x;
            this->y -= other.y;
            this->z -= other.z;
            return *this;
        }

        constexpr vector3D& operator*=(const vector3D& other) noexcept
        {
            this->x *= other.x;
            this->y *= other.y;
            this->z *= other.z;
            return *this;
        }

        constexpr vector3D& operator/=(const vector3D& other) noexcept
        {
            this->x /= other.x;
            this->y /= other.y;
            this->z /= other.z;
            return *this;
        }

        constexpr vector3D& operator*=(const value_type factor) noexcept
        {
            this->x *= factor;
            this->y *= factor;
            this->z *= factor;
            return *this;
        }

        constexpr vector3D& operator/=(const value_type factor) noexcept
        {
            this->x /= factor;
            this->y /= factor;
            this->z /= factor;
            return *this;
        }

        [[nodiscard]] constexpr bool operator<=>(const vector3D& other) const noexcept = default;

        value_type x{};
        value_type y{};
        value_type z{};
    };

    template <arithmetic value_type>
    constexpr auto operator+(const vector3D<value_type>& left, const vector3D<value_type>& right) noexcept
    {
        return vector3D<value_type>{left.x += right.x, left.y += right.y, left.z += right.z};
    }

    template <arithmetic value_type>
    constexpr auto operator-(const vector3D<value_type>& left, const vector3D<value_type>& right) noexcept
    {
        return vector3D<value_type>{left.x -= right.x, left.y -= right.y, left.z -= right.z};
    }

    template <arithmetic value_type>
    constexpr auto operator*(const vector3D<value_type>& left, const vector3D<value_type>& right) noexcept
    {
        return vector3D<value_type>{left.x *= right.x, left.y *= right.y, left.z *= right.z};
    }

    template <arithmetic value_type>
    constexpr auto operator/(const vector3D<value_type>& left, const vector3D<value_type>& right) noexcept
    {
        return vector3D{left.x /= right.x, left.y /= right.y, left.z /= right.z};
    }

    template <arithmetic value_type>
    constexpr auto operator*(const value_type factor, const vector3D<value_type>& point) noexcept
    {
        return vector3D<value_type>{point.x *= factor, point.y *= factor, point.z *= factor};
    }

    template <arithmetic value_type>
    constexpr auto operator*(const vector3D<value_type>& point, const value_type factor) noexcept
    {
        return vector3D{point.x += factor, point.y += factor, point.z += factor};
    }

    template <arithmetic value_type>
    constexpr auto operator/(const vector3D<value_type>& point, const value_type factor) noexcept
    {
        return vector3D<value_type>{point.x /= factor, point.y /= factor, point.z /= factor};
    }

}; // namespace linalg

#endif // VECTOR3D_HPP