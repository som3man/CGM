module;

#include <type_traits>

export module CGM:Matrix;

import :Defines;
import :Vector;
import :Rotation;
import :Angle;

#define nodiscard [[nodiscard]]

namespace CGM {
    export template <unsigned size, typename Type>
    class Matrix {
    public:
        static_assert(size >= 2 && size <= 4);

        using Row = Vector<size, Type>;

        nodiscard static constexpr Matrix LookAt(const Vector3<Type>& position, const Vector3<Type>& target, const Vector3<Type>& up = { Type(0), Type(1), Type(0) }) noexcept {
            static_assert(size == 4 && std::is_signed_v<Type>);

            const auto f = Vector3<float>(target - position).Normalized();
            const auto s = f.Cross(up).Normalized();
            const auto u = s.Cross(f);

            Matrix result;

            result[0] = {
                s.x(),
                u.x(),
                -f.x(),
                Type(0)
            };

            result[1] = {
                s.y(),
                u.y(),
                -f.y(),
                Type(0)
            };

            result[2] = {
                s.z(),
                u.z(),
                -f.z(),
                Type(0)
            };

            result[3] = {
                -s.Dot(position),
                -u.Dot(position),
                f.Dot(position),
                Type(1)
            };

            return result;
        }

        nodiscard static constexpr Matrix Model(const Vector3<Type>& position, const Rotation& rotation, const Vector3<Type>& scale) noexcept {
            static_assert(size == 4 && std::is_signed_v<Type>);

            return Matrix()
                .Translate(position)
                .Rotate(rotation)
                .Scale(scale);
        }

        nodiscard static constexpr Matrix View(const Vector3<Type>& position, const Rotation& rotation) noexcept {
            static_assert(size == 4 && std::is_signed_v<Type>);

            return Matrix()
                .Rotate(-rotation)
                .Translate(-position);
        }

        nodiscard static constexpr Matrix Orthogonal(const Vector2<Type>& min, const Vector2<Type>& max, Type near, Type far) noexcept {
            static_assert(size == 4 && std::is_signed_v<Type>);

            Matrix result = {};

            result[0][0] = Type(2) / (max.x() - min.x());
            result[1][1] = Type(2) / (max.y() - min.y());
            result[2][2] = -Type(1) / (far - near);
            
            result[3] = {
                -(max.x() + min.x()) / (max.x() - min.x()),
                -(max.y() + min.y()) / (max.y() - min.y()),
                -near / (far - near),
                Type(1)
            };

            return result;
        }

        nodiscard static constexpr Matrix Orthogonal(const Vector2<Type>& lsize, Type near, Type far) noexcept {
            return Orthogonal(-lsize / Type(2), lsize / Type(2), near, far);
        }

        nodiscard static constexpr Matrix Perspective(angle fov, Type aspect, Type near, Type far) noexcept {
            static_assert(size == 4 && std::is_signed_v<Type>);

            if (EpsZero(aspect))
                aspect = Type(1);

            const Type tan = static_cast<Type>((fov * 0.5f).Tan());

            Matrix result = {};

            result[0][0] = Type(1) / (aspect * tan);
            result[1][1] = Type(1) / tan;
            result[2][2] = -(far + near) / (far - near);
            result[2][3] = -Type(1);
            result[3][2] = -(Type(2) * far * near) / (far - near);
            result[3][3] = Type(0);

            return result;
        }

        nodiscard static constexpr Matrix DepthBias(Type value) noexcept {
            static_assert(size == 4);

            return {
                Row(value, 0.0f, 0.0f, 0.0f),
                Row(0.0f, value, 0.0f, 0.0f),
                Row(0.0f, 0.0f, value, 0.0f),
                Row(value, value, value, 1.0f)
            };
        }

        nodiscard static constexpr Matrix FromRotation(const Rotation& rotation) noexcept {
            static_assert(size > 2 && std::is_signed_v<Type>);

            Matrix result;

            const Type  
                xx = static_cast<Type>(rotation.x() * rotation.x()),
                yy = static_cast<Type>(rotation.y() * rotation.y()),
                zz = static_cast<Type>(rotation.z() * rotation.z());

            const Type  
                xz = static_cast<Type>(rotation.x() * rotation.z()),
                xy = static_cast<Type>(rotation.x() * rotation.y()),
                yz = static_cast<Type>(rotation.y() * rotation.z());

            const Type  
                wx = static_cast<Type>(rotation.w() * rotation.x()),
                wy = static_cast<Type>(rotation.w() * rotation.y()),
                wz = static_cast<Type>(rotation.w() * rotation.z());

            result[0][0] = Type(1) - Type(2) * (yy + zz);
            result[0][1] = Type(2) * (xy + wz);
            result[0][2] = Type(2) * (xz - wy);

            result[1][0] = Type(2) * (xy - wz);
		    result[1][1] = Type(1) - Type(2) * (xx + zz);
		    result[1][2] = Type(2) * (yz + wx);

            result[2][0] = Type(2) * (xz + wy);
		    result[2][1] = Type(2) * (yz - wx);
		    result[2][2] = Type(1) - Type(2) * (xx + yy);

            if constexpr (size > 3) {
                result[0][3] = Type(0);
                result[1][3] = Type(0);
                result[2][3] = Type(0);

                result[3] = {
                    Type(0), Type(0), Type(0), Type(1)
                };
            }

            return result;
        }

        constexpr Matrix() noexcept {
            rows[0] = 0.0f;
            rows[0][0] = 1.0f;

            rows[1] = 0.0f;
            rows[1][1] = 1.0f;

            if constexpr (size > 2) {
                rows[2] = 0.0f;
                rows[2][2] = 1.0f;
            }

            if constexpr (size > 3) {
                rows[3] = 0.0f;
                rows[3][3] = 1.0f;
            }
        }

        constexpr Matrix(const Matrix& other) noexcept {
            operator=(other);
        }

        template <typename... Rows> requires(sizeof...(Rows) == size)
        constexpr Matrix(Rows... rows) noexcept : rows(rows...) {};

        template <unsigned size2, typename Type2>
        constexpr Matrix(const Matrix<size2, Type2>& other) noexcept {
            rows[0] = Row(other[0]);
            rows[1] = Row(other[1]);

            if constexpr (size > 2) {
                if constexpr (size2 > 2)
                    rows[2] = Row(other[2]);
                else
                    rows[2] = Row();
            }

            if constexpr (size > 3) {
                if constexpr (size2 > 3)
                    rows[3] = Row(other[3]);
                else
                    rows[3] = Row();
            }
        }

        constexpr ~Matrix() noexcept {};

        constexpr Matrix& operator =(const Matrix& other) noexcept {
            rows[0] = other[0];
            rows[1] = other[1];

            if constexpr (size > 2)
                rows[2] = other[2];

            if constexpr (size > 3)
                rows[3] = other[3];

            return *this;
        }

        constexpr Matrix& operator +=(const Matrix& other) noexcept {
            rows[0] += other[0];
            rows[1] += other[1];

            if constexpr (size > 2)
                rows[2] += other[2];

            if constexpr (size > 3)
                rows[3] += other[3];

            return *this;
        }

        constexpr Matrix& operator -=(const Matrix& other) noexcept {
            rows[0] -= other[0];
            rows[1] -= other[1];

            if constexpr (size > 2)
                rows[2] -= other[2];

            if constexpr (size > 3)
                rows[3] -= other[3];

            return *this;
        }

        constexpr Matrix& operator *=(const Matrix& other) noexcept {
            return operator=(*this * other);
        }

        constexpr Matrix& operator *=(const Rotation& rotation) noexcept {
            static_assert(size > 2);

            return operator*=(Matrix::FromRotation(rotation));
        }

        nodiscard constexpr Matrix operator +(const Matrix& other) const noexcept {
            Matrix result = *this;

            result += other;

            return result;
        }

        nodiscard constexpr Matrix operator -(const Matrix& other) const noexcept {
            Matrix result = *this;

            result -= other;

            return result;
        }

        nodiscard constexpr Matrix operator *(const Matrix& other) const noexcept {
            Matrix result;

            result[0]  = rows[0] * other[0][0];
            result[0] += rows[1] * other[0][1];

            result[1]  = rows[0] * other[1][0];
            result[1] += rows[1] * other[1][1];

            if constexpr (size > 2) {
                result[0] += rows[2] * other[0][2];

                result[1] += rows[2] * other[1][2];

                result[2]  = rows[0] * other[2][0];
                result[2] += rows[1] * other[2][1];
                result[2] += rows[2] * other[2][2];
            }

            if constexpr (size > 3) {
                result[0] += rows[3] * other[0][3];

                result[1] += rows[3] * other[1][3];

                result[2] += rows[3] * other[2][3];

                result[3]  = rows[0] * other[3][0];
                result[3] += rows[1] * other[3][1];
                result[3] += rows[2] * other[3][2];
                result[3] += rows[3] * other[3][3];
            }

            return result;
        }

        nodiscard constexpr Matrix operator *(const Rotation& rotation) const noexcept {
            static_assert(size > 2);

            return operator*(Matrix::FromRotation(rotation));
        }

        nodiscard constexpr Vector<size, Type> operator *(const Vector<size, Type>& vector) const noexcept {
            Vector<size, Type> result;

            result[0] = rows[0][0] * vector[0] + rows[1][0] * vector[1];
            result[1] = rows[0][1] * vector[0] + rows[1][1] * vector[1];

            if constexpr (size > 2) {
                result[0] += rows[2][0] * vector[2];
                result[1] += rows[2][1] * vector[2];

                result[2] = rows[0][2] * vector[0] + rows[1][2] * vector[1] + rows[2][2] * vector[2];
            }

            if constexpr (size > 3) {
                result[0] += rows[3][0] * vector[3];
                result[1] += rows[3][1] * vector[3];
                result[2] += rows[3][2] * vector[3];

                result[3] = rows[0][3] * vector[0] + rows[1][3] * vector[1] + rows[2][3] * vector[2] + rows[3][3] * vector[3];
            }

            return result;
        }

        nodiscard constexpr Row& operator [](unsigned index) noexcept {
            return rows[index];
        }

        nodiscard constexpr const Row& operator [](unsigned index) const noexcept {
            return rows[index];
        }

        nodiscard constexpr Row* begin() noexcept {
            return rows;
        }

        nodiscard constexpr Row* end() noexcept {
            return begin() + size;
        }

        nodiscard constexpr const Row* begin() const noexcept {
            return rows;
        }

        nodiscard constexpr const Row* end() const noexcept {
            return begin() + size;
        }

        constexpr Matrix& Translate(const Vector3<Type>& vector) noexcept {
            static_assert(size == 4);

            rows[3] += rows[0] * vector[0] + rows[1] * vector[1] + rows[2] * vector[2];

            return *this;
        }

        constexpr Matrix& Scale(const Vector3<Type>& vector) noexcept {
            static_assert(size == 4);

            rows[0] *= vector[0];
            rows[1] *= vector[1];
            rows[2] *= vector[2];

            return *this;
        }

        constexpr Matrix& Rotate(const Rotation& rotation) noexcept {
            static_assert(size > 2);

            return operator*=(rotation);
        }

        constexpr Matrix& Rotate(angle pitch, angle yaw, angle roll) noexcept {
            return Rotate(Rotation::FromAngles(pitch, yaw, roll));
        }

        constexpr Matrix& Rotate(angle angle, const float (&axis)[3]) noexcept {
            return Rotate(Rotation::FromAxis(angle, axis));
        }

        constexpr Matrix& RotatePitch(angle pitch) noexcept {
            return Rotate(pitch, { 0.0f, 1.0f, 0.0f });
        }

        constexpr Matrix& RotateYaw(angle yaw) noexcept {
            return Rotate(yaw, { 1.0f, 0.0f, 0.0f });
        }

        constexpr Matrix& RotateRoll(angle roll) noexcept {
            return Rotate(roll, { 0.0f, 0.0f, 1.0f });
        }

        nodiscard constexpr Vector4<Type> GetProjectionPlaneLeft() const noexcept {
            static_assert(size == 4);

            Row result = rows[3] + rows[0];

            const auto l = Vector3<float>(result.x(), result.y(), result.z()).GetLength();

            return result / static_cast<Type>(l);
        }

        nodiscard constexpr Vector4<Type> GetProjectionPlaneRight() const noexcept {
            static_assert(size == 4);

            Row result = rows[3] - rows[0];

            const auto l = Vector3<float>(result.x(), result.y(), result.z()).GetLength();

            return result / static_cast<Type>(l);
        }

        nodiscard constexpr Vector4<Type> GetProjectionPlaneBottom() const noexcept {
            static_assert(size == 4);

            Row result = rows[3] + rows[1];

            const auto l = Vector3<float>(result.x(), result.y(), result.z()).GetLength();

            return result / static_cast<Type>(l);
        }

        nodiscard constexpr Vector4<Type> GetProjectionPlaneTop() const noexcept {
            static_assert(size == 4);

            Row result = rows[3] - rows[1];

            const auto l = Vector3<float>(result.x(), result.y(), result.z()).GetLength();

            return result / static_cast<Type>(l);
        }

        nodiscard constexpr Vector4<Type> GetProjectionPlaneFar() const noexcept {
            static_assert(size == 4);

            Row result = rows[3] - rows[2];

            const auto l = Vector3<float>(result.x(), result.y(), result.z()).GetLength();

            return result / static_cast<Type>(l);
        }

        nodiscard constexpr Vector4<Type> GetProjectionPlaneNear() const noexcept {
            static_assert(size == 4);

            Row result = rows[2];

            const auto l = Vector3<float>(result.x(), result.y(), result.z()).GetLength();

            return result / static_cast<Type>(l);
        }

    private:
        Row rows[4];
    };

    export using Matrix4 = Matrix<4, float>;

    export template <unsigned size, typename Type>
    nodiscard constexpr Matrix<size, Type> operator *(const Rotation& rotation, const Matrix<size, Type>& matrix) noexcept {
        static_assert(size > 2&& std::is_signed_v<Type>);

        return Matrix<size, Type>::FromRotation(rotation) * matrix;
    }
}