module;

#include <ostream>
#include <type_traits>

export module CGM:Vector;

import :Defines;
import :Angle;
import :Rotation;

#define nodiscard [[nodiscard]]

namespace CGM {
    export template <unsigned size, typename Type>
    class Vector {
        Type data[size];

    public:
        static_assert(size >= 2 && size <= 4);

        static constexpr Vector FromArray(const Type* array) noexcept {
            Vector result;

            result[0] = array[0];
            result[1] = array[1];

            if constexpr (size > 2)
                result[2] = array[2];

            if constexpr (size > 3)
                result[3] = array[3];

            return result;
        }

        constexpr Vector() noexcept {
            data[0] = Type(0);
            data[1] = Type(0);
            
            if constexpr (size > 2)
                data[2] = Type(0);

            if constexpr (size > 3)
                data[3] = Type(0);
        }

        constexpr Vector(const Vector& other) noexcept {
            operator=(other);
        }

        template <unsigned size2, typename Type2>
        constexpr Vector(const Vector<size2, Type2>& other) noexcept {
            data[0] = static_cast<Type>(other[0]);
            data[1] = static_cast<Type>(other[1]);

            if constexpr (size > 2) {
                if constexpr (size2 > 2)
                    data[2] = static_cast<Type>(other[2]);
                else
                    data[2] = Type();
            }

            if constexpr (size > 3) {
                if constexpr (size2 > 3)
                    data[3] = static_cast<Type>(other[3]);
                else
                    data[3] = Type();
            }

            return *this;
        }

        constexpr Vector(Type value) noexcept {
            data[0] = value;
            data[1] = value;
            
            if constexpr (size > 2)
                data[2] = value;

            if constexpr (size > 3)
                data[3] = value;
        }

        template <typename... Values> requires(sizeof...(Values) == size)
        constexpr Vector(Values... values) noexcept : data(values...) {};

        constexpr ~Vector() noexcept {};

    #define ASSIGN(OPERATOR) \
        constexpr Vector& operator OPERATOR(const Vector& other) noexcept { \
            data[0] OPERATOR other[0]; \
            data[1] OPERATOR other[1]; \
            if constexpr (size > 2) \
                data[2] OPERATOR other[2]; \
            if constexpr (size > 3) \
                data[3] OPERATOR other[3]; \
            return *this; \
        } \
        constexpr Vector& operator OPERATOR(Type value) noexcept { \
            data[0] OPERATOR value; \
            data[1] OPERATOR value; \
            if constexpr (size > 2) \
                data[2] OPERATOR value; \
            if constexpr (size > 3) \
                data[3] OPERATOR value; \
            return *this; \
        }

        ASSIGN(=);

        ASSIGN(+=);

        ASSIGN(-=);

        ASSIGN(*=);

        ASSIGN(/=);

    #undef ASSIGN

    #define OPRT(OPERATOR) \
        nodiscard constexpr Vector operator OPERATOR(const Vector& other) const noexcept { \
            Vector result = *this; \
            result OPERATOR##= other; \
            return result; \
        } \
        nodiscard constexpr Vector operator OPERATOR(const Type value) const noexcept { \
            Vector result = *this; \
            result OPERATOR##= value; \
            return result; \
        }
        
        OPRT(+);

        OPRT(-);

        OPRT(*);

        OPRT(/);

    #undef OPRT

        template <typename Type2 = Type>
        nodiscard constexpr Type2 GetLength() const noexcept {
            static_assert(std::is_floating_point_v<Type2>);

            Type2 sum = x() * x() + y() * y();

            if constexpr (size > 2)
                sum += z() * z();

            if constexpr (size > 3)
                sum += w() * w();

            return CGM::Sqrt(sum);
        }

        template <typename Type2 = Type>
        nodiscard constexpr Vector<size, Type2> Normalized() const noexcept {
            static_assert(std::is_floating_point_v<Type2>);

            const Type2 length = GetLength<Type2>();

            Vector<size, Type2> result = *this;

            result /= length;

            return result;
        }

        nodiscard constexpr Vector Opposite() const noexcept {
            static_assert(requires (Type value) {
                -value;
            });

            Vector result = *this;

            result.x() = -result.x();
            result.y() = -result.y();

            if constexpr (size > 2)
                result.z() = -result.z();

            if constexpr (size > 3)
                result.w() = -result.w();

            return result;
        }

        nodiscard constexpr Vector operator -() const noexcept {
            return Opposite();
        }

        nodiscard constexpr Vector Cross(const Vector& other) const noexcept {
            static_assert(size == 3);

            return {
                y() * other.z() - z() * other.y(),
                z() * other.x() - x() * other.z(),
                x() * other.y() - y() * other.x()
            };
        }

        nodiscard constexpr Type Dot(const Vector& other) const noexcept {
            Type result = x() * other.x() + y() * other.y();

            if constexpr (size > 2)
                result += z() * other.z();

            if constexpr (size > 3)
                result += w() * other.w();

            return result;
        }

        constexpr Vector& Rotate(const Rotation& rotation) noexcept {
            static_assert(size == 3 && std::is_signed_v<Type>);

            const Vector qvec = {
                static_cast<Type>(rotation.x()),
                static_cast<Type>(rotation.y()),
                static_cast<Type>(rotation.z())
            };

            const Vector c1 = qvec.Cross(*this);
            const Vector c2 = qvec.Cross(c1);

            return operator=(*this + (Vector(static_cast<Type>(rotation.w())) * c1 + c2) * Vector(Type(2)));
        }

        constexpr Vector& operator *=(const Rotation& rotation) noexcept {
            return Rotate(rotation.Conjugate());
        }

        constexpr Vector operator *(const Rotation& rotation) const noexcept {
            Vector result = *this;

            result *= rotation;

            return result;
        }

        constexpr Vector& Rotate(angle pitch, angle yaw, angle roll) noexcept {
            return Rotate(Rotation::FromAngles(pitch, yaw, roll));
        }

        constexpr Vector& Rotate(angle angle, const float (&axis)[3]) noexcept {
            return Rotate(Rotation::FromAxis(angle, axis));
        }

        constexpr Vector& RotatePitch(angle pitch) noexcept {
            return Rotate(pitch, { 1.0f, 0.0f, 0.0f });
        }

        constexpr Vector& RotateYaw(angle yaw) noexcept {
            return Rotate(yaw, { 0.0f, 1.0f, 0.0f });
        }

        constexpr Vector& RotateRoll(angle roll) noexcept {
            return Rotate(roll, { 0.0f, 0.0f, 1.0f });
        }

        nodiscard constexpr Type x() const noexcept {
            return data[0];
        }

        nodiscard constexpr Type y() const noexcept {
            return data[1];
        }

        nodiscard constexpr Type z() const noexcept {
            static_assert(size > 2);

            return data[2];
        }

        nodiscard constexpr Type w() const noexcept {
            static_assert(size > 3);

            return data[3];
        }

        nodiscard constexpr Type& x() noexcept {
            return data[0];
        }

        nodiscard constexpr Type& y() noexcept {
            return data[1];
        }

        nodiscard constexpr Type& z() noexcept {
            static_assert(size > 2);

            return data[2];
        }

        nodiscard constexpr Type& w() noexcept {
            static_assert(size > 3);

            return data[3];
        }

        nodiscard constexpr Type operator [](unsigned index) const noexcept {
            return data[index];
        }

        nodiscard constexpr Type& operator [](unsigned index) noexcept {
            return data[index];
        }

        nodiscard constexpr Type* begin() noexcept {
            return data;
        }

        nodiscard constexpr const Type* begin() const noexcept {
            return data;
        }

        nodiscard constexpr Type* end() noexcept {
            return begin() + size;
        }

        nodiscard constexpr const Type* end() const noexcept {
            return begin() + size;
        }

        nodiscard constexpr operator auto& () noexcept {
            return data;
        }

        nodiscard constexpr operator const auto& () const noexcept {
            return data;
        }

        nodiscard constexpr bool isZero() const noexcept {
            if (!EpsZero(data[0]))
                return false;

            if (!EpsZero(data[1]))
                return false;

            if constexpr (size > 2)
                if (!EpsZero(data[2]))
                    return false;

            if constexpr (size > 3)
                if (!EpsZero(data[3]))
                    return false;

            return true;
        }

        nodiscard constexpr bool notZero() const noexcept {
            return !isZero();
        }

        nodiscard constexpr operator bool() const noexcept {
            return notZero();
        }

        nodiscard constexpr bool operator ==(const Vector& other) const noexcept {
            if (!EpsEqual(x(), other.x()))
                return false;

            if (!EpsEqual(y(), other.y()))
                return false;

            if constexpr (size > 2)
                if (!EpsEqual(z(), other.z()))
                    return false;

            if constexpr (size > 3)
                if (!EpsEqual(w(), other.w()))
                    return false;

            return true;
        }

        nodiscard constexpr bool operator !=(const Vector& other) const noexcept {
            return !operator==(other);
        }

        nodiscard constexpr bool operator <=(const Vector& other) const noexcept {
            if (x() > other.x() || y() > other.y())
                return false;

            if constexpr (size > 2)
                if (z() > other.z())
                    return false;

            if constexpr (size > 3)
                if (w() > other.w())
                    return false;

            return true;
        }

        nodiscard constexpr bool operator >=(const Vector& other) const noexcept {
            if (x() < other.x() || y() < other.y())
                return false;

            if constexpr (size > 2)
                if (z() < other.z())
                    return false;

            if constexpr (size > 3)
                if (w() < other.w())
                    return false;

            return true;
        }

        nodiscard constexpr bool operator <(const Vector& other) const noexcept {
            if (x() >= other.x() || y() >= other.y())
                return false;

            if constexpr (size > 2)
                if (z() >= other.z())
                    return false;

            if constexpr (size > 3)
                if (w() >= other.w())
                    return false;

            return true;
        }

        nodiscard constexpr bool operator >(const Vector& other) const noexcept {
            if (x() <= other.x() || y() <= other.y())
                return false;

            if constexpr (size > 2)
                if (z() <= other.z())
                    return false;

            if constexpr (size > 3)
                if (w() <= other.w())
                    return false;

            return true;
        }
    };

    export template <typename Type>
    using Vector2 = Vector<2, Type>;

    export template <typename Type>
    using Vector3 = Vector<3, Type>;

    export template <typename Type>
    using Vector4 = Vector<4, Type>;

    export template <typename Type>
    nodiscard constexpr Vector<3, Type> operator *(const Rotation& rotation, const Vector<3, Type>& vector) noexcept {
        Vector<3, Type> result = vector;

        result.Rotate(rotation);

        return result;
    }

    export template <unsigned size, typename Type>
    constexpr std::ostream& operator <<(std::ostream& ostream, const Vector<size, Type>& vector) noexcept {
        ostream << "(x:" << vector.x() << "; y:" << vector.y();

        if constexpr (size > 2)
            ostream << "; z:" << vector.z();

        if constexpr (size > 3)
            ostream << "; w:" << vector.w();

        return ostream << ")";
    }
}