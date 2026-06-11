module;

#include <cstdlib>
#include <numbers>
#include <type_traits>
#include <cmath>
#include <algorithm>

export module CGM:Defines;

#define nodiscard [[nodiscard]]

namespace CGM {
    namespace Internal {
        template <typename Type>
        constexpr Type pi() noexcept {
            if constexpr (std::is_same_v<Type, float>)
                return std::numbers::pi_v<float>;
            else if (std::is_same_v<Type, double>)
                return std::numbers::pi_v<double>;
            else if (std::is_same_v<Type, long double>)
                return std::numbers::pi_v<long double>;
            else
                return 3;
        }

        template <typename Type>
        constexpr Type epsilon() noexcept {
            if constexpr (std::is_same_v<Type, float>)
                return 0.00000011920928955078125f;
            else if (std::is_same_v<Type, double>)
                return 2.220446049250313e-16;
            else if (std::is_same_v<Type, long double>)
                return 1.925929944387235853e-34L;
            else 
                return 0;
        }
    };

    export template <typename Type>
    constexpr Type pi = Internal::pi<Type>();

    export template <typename Type>
    constexpr Type epsilon = Internal::epsilon<Type>();

    // Normalizes radian value to [0; 2pi) range.
    export template <typename Type>
    nodiscard constexpr Type NormalizeRads(Type radians) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        constexpr Type twoPi = pi<Type> * Type(2);

        Type result = std::fmod(radians, twoPi);

        if (result >= twoPi)
            result -= twoPi;
        else if (result < Type(0))
            result = Type(0);

        return result;
    }

    export template <typename Type>
    nodiscard constexpr Type Abs(Type value) noexcept {
        if constexpr (!std::is_signed_v<Type>)
            return value;

        if constexpr (sizeof(Type) < 4)
            return static_cast<Type>(std::abs(static_cast<int>(value)));

        return std::abs(value);
    }

    export template <typename Type>
    nodiscard constexpr bool EpsEqual(Type left, Type right) noexcept {
        if constexpr (std::is_floating_point_v<Type>) {
            const Type scale = std::max(Abs(left), Abs(right));

            return Abs(left - right) <= (scale * epsilon<Type> * 2.0f);
        }
        else
            return left == right;
    }

    export template <typename Type>
    nodiscard constexpr bool EpsEqual(Type left, Type right, Type scale) noexcept {
        if constexpr (std::is_floating_point_v<Type>)
            return Abs(left - right) <= epsilon<Type> * scale;
        else
            return left == right;
    }

    export template <typename Type>
    nodiscard constexpr bool EpsZero(Type value) noexcept {
        if constexpr (std::is_floating_point_v<Type>)
            return Abs(value) <= epsilon<Type>;
        else
            return value == Type(0);
    }

    export template <typename Type>
    nodiscard constexpr Type Sin(Type radians) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        radians = NormalizeRads(radians);

        const Type hpis = radians / pi<Type> * Type(2);

        const int ihpis = static_cast<int>(hpis);

        if (EpsEqual(static_cast<Type>(ihpis), hpis, 4.0f)) {
            switch (ihpis) {
            case 1:
                return Type(1);
            case 3:
                return Type(-1);
            case 0:
            case 2:
            case 4:
                return Type(0);
            default:
                break;
            }
        }

        return std::sin(radians);
    }

    export template <typename Type>
    nodiscard constexpr Type Cos(Type radians) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        radians = NormalizeRads(radians);

        const Type hpis = radians / pi<Type> * Type(2);

        const int ihpis = static_cast<int>(hpis);

        if (EpsEqual(static_cast<Type>(ihpis), hpis, 4.0f)) {
            switch (ihpis) {
            case 0:
            case 4:
                return Type(1);
            case 1:
            case 3:
                return Type(0);
            case 2:
                return Type(-1);
            default:
                break;
            }
        }

        return std::cos(radians);
    }

    export template <typename Type>
    nodiscard constexpr Type Tan(Type radians) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        if (EpsZero(radians))
            return Type(0);

        return std::tan(radians);
    }
    
    export template <typename Type>
    nodiscard constexpr Type Asin(Type value) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        if (EpsZero(value))
            return Type(0);

        return std::asin(value);
    }

    export template <typename Type>
    nodiscard constexpr Type Acos(Type value) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        return std::acos(value);
    }

    export template <typename Type>
    nodiscard constexpr Type Atan(Type value) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        if (EpsZero(value))
            return Type(0);

        return std::atan(value);
    }

    export template <typename Type>
    nodiscard constexpr Type Atan(Type y, Type x) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        return std::atan2(y, x);
    }

    export template <typename Type>
    nodiscard constexpr Type Sqrt(Type value) noexcept {
        static_assert(std::is_floating_point_v<Type>);

        return std::sqrt(value);
    }

    export template <typename Type>
    nodiscard constexpr Type Clamp(Type value, Type low, Type high) noexcept {
        return std::clamp(value, low, high);
    }
}