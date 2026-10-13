#pragma once

#include <cmath>
#include <iostream>

#include <numbers>

constexpr float deg_to_rad(float deg) {
    return deg * std::numbers::pi / 180.0f;
}

struct Vec3 {
    float x, y, z;

    constexpr Vec3() : x(0), y(0), z(0) {}
    constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    constexpr Vec3 operator-() const {
        return {-x, -y, -z};
    }

    constexpr Vec3 operator+(const Vec3& rhs) const {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    constexpr Vec3 operator-(const Vec3& rhs) const {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    constexpr Vec3 operator*(float s) const {
        return {x * s, y * s, z * s};
    }

    constexpr Vec3 operator/(float s) const {
        return {x / s, y / s, z / s};
    }

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    Vec3& operator/=(float s) {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    constexpr bool operator==(const Vec3& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    constexpr bool operator!=(const Vec3& rhs) const {
        return !(*this == rhs);
    }

    float length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    float lengthSquared() const {
        return x * x + y * y + z * z;
    }

    Vec3 normalized() const {
        float len = length();
        return (len > 0.0f) ? *this / len : Vec3{};
    }

    static constexpr float dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }
};

constexpr Vec3 operator*(float s, const Vec3& v) {
    return v * s;
}

std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}
template<typename T>
struct Mat3 {
    T m[3][3];

    constexpr Mat3() : m{} {}

    constexpr Mat3(
        T m00, T m01, T m02,
        T m10, T m11, T m12,
        T m20, T m21, T m22
    ) : m{
        {m00, m01, m02},
        {m10, m11, m12},
        {m20, m21, m22}
    } {}

    static constexpr Mat3 identity() {
        return {
            T(1), T(0), T(0),
            T(0), T(1), T(0),
            T(0), T(0), T(1)
        };
    }

    T* operator[](int row) {
        return m[row];
    }

    const T* operator[](int row) const {
        return m[row];
    }

    Mat3 operator+(const Mat3& rhs) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r[i][j] = m[i][j] + rhs[i][j];
        return r;
    }

    Mat3 operator-(const Mat3& rhs) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r[i][j] = m[i][j] - rhs[i][j];
        return r;
    }

    Mat3 operator*(T s) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r[i][j] = m[i][j] * s;
        return r;
    }

    Mat3 operator/(T s) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r[i][j] = m[i][j] / s;
        return r;
    }

    Mat3 operator*(const Mat3& rhs) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r[i][j] = T(0);
                for (int k = 0; k < 3; ++k)
                    r[i][j] += m[i][k] * rhs[k][j];
            }
        }
        return r;
    }

    Mat3& operator+=(const Mat3& rhs) {
        return *this = *this + rhs;
    }

    Mat3& operator-=(const Mat3& rhs) {
        return *this = *this - rhs;
    }

    Mat3& operator*=(const Mat3& rhs) {
        return *this = *this * rhs;
    }

    Mat3& operator*=(T s) {
        return *this = *this * s;
    }

    Mat3& operator/=(T s) {
        return *this = *this / s;
    }

    Vec3 operator*(const Vec3& v) const {
        return {
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        };
    }

    bool operator==(const Mat3& rhs) const {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (m[i][j] != rhs[i][j])
                    return false;
        return true;
    }

    bool operator!=(const Mat3& rhs) const {
        return !(*this == rhs);
    }

    T determinant() const {
        return
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
            m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
            m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    Mat3 transpose() const {
        return {
            m[0][0], m[1][0], m[2][0],
            m[0][1], m[1][1], m[2][1],
            m[0][2], m[1][2], m[2][2]
        };
    }

    Mat3 inverse() const {
        T det = determinant();

        return {
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det,

            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det,

            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det
        };
    }
};

template<typename T>
Mat3<T> operator*(T s, const Mat3<T>& m) {
    return m * s;
}

Mat3<float> get_rmatrix(float yaw, float pitch) {
    yaw = deg_to_rad(yaw);
    pitch = deg_to_rad(pitch);

    float cy = cosf(yaw);
    float sy = sinf(yaw);

    float cp = cosf(pitch);
    float sp = sinf(pitch);

    Mat3<float> Rx(
        1, 0, 0,
        0, cy, -sy,
        0, sy, cy
    );

    Mat3<float> Ry(
        cp, 0, -sp,
        0,  1,   0,
        sp,  0, cp
    );

    return Rx * Ry;
}