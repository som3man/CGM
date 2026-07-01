module;

#include <iostream>
#include <ostream>

export module CGM:Rotation;

import :Defines;
import :Angle;

#define nodiscard [[nodiscard]]

namespace CGM {
    export class Rotation {
        float data[4];

    public:
        nodiscard constexpr operator auto&() noexcept {
            return data;
        }

        nodiscard constexpr operator const auto&() const noexcept {
            return data;
        }

        nodiscard static Rotation FromArray(const float* array) {
            return {
                array[0],
                array[1],
                array[2],
                array[3]
            };
        }

        nodiscard static constexpr Rotation FromAxis(angle angle, const float (&axis)[3]) noexcept {
            angle *= 0.5f;

            const float sin = angle.Sin();

            return {
                angle.Cos(),
                axis[0] * sin,
                axis[1] * sin,
                axis[2] * sin
            };
        }

        nodiscard static constexpr Rotation FromAngles(angle pitch, angle yaw, angle roll) noexcept {
            pitch *= 0.5f;
            yaw   *= 0.5f;
            roll  *= 0.5f;

            const float 
                cx = pitch.Cos(),
                cy = yaw.Cos(),
                cz = roll.Cos();

            const float 
                sx = pitch.Sin(),
                sy = yaw.Sin(),
                sz = roll.Sin();

            return {
                cx * cy * cz + sx * sy * sz,
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz - sx * sy * cz
            };
        }

        constexpr Rotation() noexcept {
            w() = 1.0f;
            x() = 0.0f;
            y() = 0.0f;
            z() = 0.0f;
        }

        constexpr Rotation(float w, float x, float y, float z) noexcept {
            this->w() = w;
            this->x() = x;
            this->y() = y;
            this->z() = z;
        }

        constexpr Rotation(const Rotation& other) noexcept {
            data[0] = other[0];
            data[1] = other[1];
            data[2] = other[2];
            data[3] = other[3];
        }

        constexpr ~Rotation() noexcept {};

        constexpr Rotation& operator =(const Rotation& other) noexcept {
            data[0] = other[0];
            data[1] = other[1];
            data[2] = other[2];
            data[3] = other[3];

            return *this;
        }

        constexpr Rotation& operator *=(const Rotation& other) noexcept {
            return *this = *this * other;
        }

        nodiscard constexpr Rotation operator *(const Rotation& other) const noexcept {
            return {
                w() * other.w() - x() * other.x() - y() * other.y() - z() * other.z(),
                w() * other.x() + x() * other.w() + y() * other.z() - z() * other.y(),
                w() * other.y() - x() * other.z() + y() * other.w() + z() * other.x(),
                w() * other.z() + x() * other.y() - y() * other.x() + z() * other.w()
            };
        }

        nodiscard constexpr angle GetPitch() const noexcept {
            const float ly = 2.0f * (y() * z() + w() * x());

            const float lx = w() * w() - x() * x() - y() * y() + z() * z();

            if (EpsZero(ly) && EpsZero(lx))
                return angle::Atan(x(), w()) * 2.0f;

            return angle::Atan(ly, lx);
        }

        nodiscard constexpr angle GetYaw() const noexcept {
            return angle::Asin(Clamp(-2.0f * (x() * z() - w() * y()), -1.0f, 1.0f));
        }

        nodiscard constexpr angle GetRoll() const noexcept {
            const float ly = 2.0f * (x() * y() + w() * z());

            const float lx = w() * w() + x() * x() - y() * y() - z() * z();

            if (EpsZero(ly) && EpsZero(lx))
                return angle();

            return angle::Atan(ly, lx);
        }

        constexpr Rotation& Rotate(const Rotation& other) noexcept {
            return *this = other * *this;
        }

        constexpr Rotation& Rotate(angle pitch, angle yaw, angle roll) noexcept {
            return Rotate(FromAngles(pitch, yaw, roll));
        }

        constexpr Rotation& Rotate(angle angle, const float (&axis)[3]) noexcept {
            return Rotate(FromAxis(angle, axis));
        }

        constexpr Rotation& RotatePitch(angle pitch) noexcept {
            return Rotate(FromAxis(pitch, { 0.0f, 1.0f, 0.0f }));
        }

        constexpr Rotation& RotateYaw(angle yaw) noexcept {
            return Rotate(FromAxis(yaw, { 1.0f, 0.0f, 0.0f }));
        }

        constexpr Rotation& RotateRoll(angle roll) noexcept {
            return Rotate(FromAxis(roll, { 0.0f, 0.0f, 1.0f }));
        }

        nodiscard constexpr float GetLength() const noexcept {
            return Sqrt(w() * w() + x() * x() + y() * y() + z() * z());
        }

        nodiscard constexpr Rotation Normalized() const noexcept {
            const float q = GetLength();

            return {
                w() / q,
                x() / q,
                y() / q,
                z() / q
            };
        }

        nodiscard constexpr Rotation Conjugate() const noexcept {
            return {
                w(),
                -x(),
                -y(),
                -z()
            };
        }

        nodiscard constexpr Rotation operator -() const noexcept {
            return Conjugate();
        }

        nodiscard constexpr bool operator ==(const Rotation& other) const noexcept {
            return EpsEqual(w(), other.w()) &&
                EpsEqual(x(), other.x()) &&
                EpsEqual(y(), other.y()) &&
                EpsEqual(z(), other.z());
        }

        nodiscard constexpr bool operator !=(const Rotation& other) const noexcept {
            return !operator==(other);
        }

        nodiscard constexpr bool isNull() const noexcept {
            return operator==(Rotation());
        }

        nodiscard constexpr bool notNull() const noexcept {
            return !isNull();
        }

        nodiscard constexpr operator bool() const noexcept {
            return notNull();
        }

        nodiscard constexpr float operator [](unsigned index) const noexcept {
            return data[index];
        }

        nodiscard constexpr float& operator [](unsigned index) noexcept {
            return data[index];
        }

        nodiscard constexpr float* begin() noexcept {
            return data;
        }

        nodiscard constexpr float* end() noexcept {
            return begin() + 4;
        }

        nodiscard constexpr const float* begin() const noexcept {
            return data;
        }

        nodiscard constexpr const float* end() const noexcept {
            return begin() + 4;
        }

        nodiscard constexpr float w() const noexcept {
            return operator[](0);
        }

        nodiscard constexpr float x() const noexcept {
            return operator[](1);
        }

        nodiscard constexpr float y() const noexcept {
            return operator[](2);
        }

        nodiscard constexpr float z() const noexcept {
            return operator[](3);
        }

        nodiscard constexpr float& w() noexcept {
            return operator[](0);
        }

        nodiscard constexpr float& x() noexcept {
            return operator[](1);
        }

        nodiscard constexpr float& y() noexcept {
            return operator[](2);
        }

        nodiscard constexpr float& z() noexcept {
            return operator[](3);
        }
    };

    export std::ostream& operator <<(std::ostream& ostream, const Rotation& rotation) noexcept {
        //return ostream << "(w:" << rotation.w() << "; x:" << rotation.x() << "; y:" << rotation.y() << "; z:" << rotation.z() << ")";

        return ostream << "(pitch:" << rotation.GetPitch() << "; yaw:" << rotation.GetYaw() << "; roll:" << rotation.GetRoll() << ")";
    }
}