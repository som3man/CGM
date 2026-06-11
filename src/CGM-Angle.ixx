module;

#include <iostream>
#include <ostream>
#include <cmath>

export module CGM:Angle;

import :Defines;

#define nodiscard [[nodiscard]]

namespace CGM {
    export class angle {
        static constexpr float max = 6.28318548f; // 2 * pi

        float radians;

        constexpr angle(float radians) noexcept : radians(radians) {};

    public:
        nodiscard static constexpr angle Asin(float value) noexcept {
            return CGM::Asin(value);
        }

        nodiscard static constexpr angle Acos(float value) noexcept {
            return CGM::Acos(value);
        }

        nodiscard static constexpr angle Atan(float value) noexcept {
            return CGM::Atan(value);
        }

        nodiscard static constexpr angle Atan(float y, float x) noexcept {
            return CGM::Atan(y, x);
        }

        nodiscard static constexpr angle FromRadians(float radians) noexcept {
            angle result = radians;

            result.radians = NormalizeRads(result.radians);

            return result;
        }

        nodiscard static constexpr angle FromDegrees(float degrees) noexcept {
            angle result = pi<float> / 180.0f * degrees;

            result.radians = NormalizeRads(result.radians);

            return result;
        }

        constexpr angle() noexcept : radians(0.0f) {};

        constexpr angle(const angle& other) noexcept : radians(other.radians) {};

        constexpr ~angle() noexcept {};

        constexpr angle& operator =(const angle& other) noexcept {
            radians = other.radians;

            return *this;
        }

        constexpr angle& operator +=(const angle& other) noexcept {
            radians += other.radians;

            if (radians >= max)
                radians -= max;

            return *this;
        }

        constexpr angle& operator -=(const angle& other) noexcept {
            radians -= other.radians;

            if (radians < 0.0f)
                radians += max;

            return *this;
        }

        constexpr angle& operator *=(float other) noexcept {
            radians *= other;

            if (other > 1.0f || other < 0.0f)
                radians = NormalizeRads(radians);

            return *this;
        }

        constexpr angle& operator /=(float other) noexcept {
            radians /= other;

            if (other < 0.0f)
                radians = NormalizeRads(radians);

            return *this;
        }

        nodiscard constexpr angle operator +(const angle& other) const noexcept {
            angle result = *this;

            result += other;

            return result;
        }

        nodiscard constexpr angle operator -(const angle& other) const noexcept {
            angle result = *this;

            result -= other;

            return result;
        }

        nodiscard constexpr angle operator *(float other) const noexcept {
            angle result = *this;

            result *= other;

            return result;
        }

        nodiscard constexpr angle operator /(float other) const noexcept {
            angle result = *this;

            result /= other;

            return result;
        }

        nodiscard constexpr float Sin() const noexcept {
            const float hpis = radians / pi<float> * 2.0f;

            const int ihpis = static_cast<int>(hpis);

            if (EpsEqual(static_cast<float>(ihpis), hpis, 4.0f)) {
                switch (ihpis) {
                case 1:
                    return 1.0f;
                case 3:
                    return -1.0f;
                case 0:
                case 2:
                case 4:
                    return 0.0f;
                default:
                    break;
                }
            }

            return std::sin(radians);
        }

        nodiscard constexpr float Cos() const noexcept {
            const float hpis = radians / pi<float> * 2.0f;

            const int ihpis = static_cast<int>(hpis);

            if (EpsEqual(static_cast<float>(ihpis), hpis, 4.0f)) {
                switch (ihpis) {
                case 0:
                case 4:
                    return 1.0f;
                case 1:
                case 3:
                    return 0.0f;
                case 2:
                    return -1.0f;
                default:
                    break;
                }
            }

            return std::cos(radians);
        }

        nodiscard constexpr float Tan() const noexcept {
            return CGM::Tan(radians);
        }

        nodiscard constexpr float ToRadians() const noexcept {
            return radians;
        }

        nodiscard constexpr float ToDegrees() const noexcept {
            return 180.0f / pi<float> * radians;
        }

        nodiscard constexpr angle Opposite() const noexcept {
            if (radians < pi<float>)
                return radians + pi<float>;
            else
                return radians - pi<float>;
        }

        nodiscard constexpr angle operator -() const noexcept {
            return Opposite();
        }

        nodiscard constexpr bool operator ==(const angle& other) const noexcept {
            return EpsEqual(radians, other.radians, 4.0f);
        }

        nodiscard constexpr bool operator !=(const angle& other) const noexcept {
            return !operator==(other);
        }

        nodiscard constexpr bool operator <=(const angle& other) const noexcept {
            return radians <= other.radians;
        }

        nodiscard constexpr bool operator >=(const angle& other) const noexcept {
            return radians >= other.radians;
        }

        nodiscard constexpr bool operator <(const angle& other) const noexcept {
            return radians < other.radians;
        }

        nodiscard constexpr bool operator >(const angle& other) const noexcept {
            return radians > other.radians;
        }

        nodiscard constexpr bool isZero() const noexcept {
            return EpsZero(radians);
        }

        nodiscard constexpr bool notZero() const noexcept {
            return !isZero();
        }

        nodiscard constexpr operator bool() const noexcept {
            return notZero();
        }
    };

    export std::ostream& operator <<(std::ostream& ostream, const angle& angle) noexcept {
        return ostream << angle.ToDegrees() << "_degs";
    }
}

export CGM::angle operator ""_rads(long double radians) noexcept {
    return CGM::angle::FromRadians(radians);
}

export CGM::angle operator ""_degs(long double degrees) noexcept {
    return CGM::angle::FromDegrees(degrees);
}