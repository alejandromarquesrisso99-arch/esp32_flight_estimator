// Librería de cabecera única (header-only) para álgebra matricial estática orientada al EKF
#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <array>
#include <cstddef>
#include <cmath>
#include <algorithm>

// Las dimensiones se definen en tiempo de compilación para optimizar el tamaño en memoria y vectorización
template <size_t Rows, size_t Cols>
struct Matrix {
    std::array<float, Rows * Cols> data{};  ///< Vector unidimensional de tamaño Rows * Cols inicializado a cero

    // Acceso a elementos por fila y columna (lectura/escritura)
    float& operator()(size_t r, size_t c) {
        return data[r * Cols + c];
    }

    const float& operator()(size_t r, size_t c) const {
        return data[r * Cols + c];
    }

    // Acceso directo indexado para vectores columna (N x 1)
    float& operator()(size_t i) {
        return data[i];
    }

    const float& operator()(size_t i) const {
        return data[i];
    }

    // Construcción de la matriz identidad
    static Matrix<Rows, Cols> identity() {
        static_assert(Rows == Cols, "La matriz identidad debe ser cuadrada");
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i < Rows; ++i) {
            res(i, i) = 1.0f;
        }
        return res;
    }

    // --- OPERACIONES ALGEBRAICAS MATRICIALES ---

    // Suma matricial
    Matrix<Rows, Cols> operator+(const Matrix<Rows, Cols>& other) const {
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i < Rows * Cols; ++i) {
            res.data[i] = this->data[i] + other.data[i];
        }
        return res;
    }

    Matrix<Rows, Cols>& operator+=(const Matrix<Rows, Cols>& other) {
        for (size_t i = 0; i < Rows * Cols; ++i) {
            this->data[i] += other.data[i];
        }
        return *this;
    }

    // Resta matricial
    Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols>& other) const {
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i < Rows * Cols; ++i) {
            res.data[i] = this->data[i] - other.data[i];
        }
        return res;
    }

    Matrix<Rows, Cols>& operator-=(const Matrix<Rows, Cols>& other) {
        for (size_t i = 0; i < Rows * Cols; ++i) {
            this->data[i] -= other.data[i];
        }
        return *this;
    }

    // Multiplicación por escalar
    Matrix<Rows, Cols> operator*(float scalar) const {
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i < Rows * Cols; ++i) {
            res.data[i] = this->data[i] * scalar;
        }
        return res;
    }

    // Multiplicación matricial (Rows x Cols) * (Cols x OtherCols)
    template <size_t OtherCols>
    Matrix<Rows, OtherCols> operator*(const Matrix<Cols, OtherCols>& other) const {
        Matrix<Rows, OtherCols> res{};
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t k = 0; k < Cols; ++k) {
                float val = (*this)(i, k);
                for (size_t j = 0; j < OtherCols; ++j) {
                    res(i, j) += val * other(k, j);
                }
            }
        }
        return res;
    }

    // Transposición matricial
    Matrix<Cols, Rows> transpose() const {
        Matrix<Cols, Rows> res{};
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t j = 0; j < Cols; ++j) {
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }

    // Norma euclídea para vectores columna
    float norm() const {
        static_assert(Cols == 1, "La norma euclídea solo es aplicable a vectores columna");
        float sum = 0.0f;
        for (size_t i = 0; i < Rows; ++i) {
            sum += data[i] * data[i];
        }
        return std::sqrt(sum);
    }

    // Extracción de sub-bloque matricial
    template <size_t BlockRows, size_t BlockCols>
    Matrix<BlockRows, BlockCols> block(size_t startRow, size_t startCol) const {
        Matrix<BlockRows, BlockCols> res{};
        for (size_t i = 0; i < BlockRows; ++i) {
            for (size_t j = 0; j < BlockCols; ++j) {
                res(i, j) = (*this)(startRow + i, startCol + j);
            }
        }
        return res;
    }

    // Asignación de sub-bloque matricial
    template <size_t BlockRows, size_t BlockCols>
    void setBlock(size_t startRow, size_t startCol, const Matrix<BlockRows, BlockCols>& blockMat) {
        for (size_t i = 0; i < BlockRows; ++i) {
            for (size_t j = 0; j < BlockCols; ++j) {
                (*this)(startRow + i, startCol + j) = blockMat(i, j);
            }
        }
    }

    // Inversión analítica exacta para matrices 3x3 mediante matriz adjunta
    Matrix<3, 3> inverse3x3() const {
        static_assert(Rows == 3 && Cols == 3, "inverse3x3 solo aplicable a matrices 3x3");
        float det = (*this)(0, 0) * ((*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1)) -
                    (*this)(0, 1) * ((*this)(1, 0) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 0)) +
                    (*this)(0, 2) * ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0));

        if (std::fabs(det) < 1e-7f || !std::isfinite(det)) {
            return Matrix<3, 3>{}; // Devuelve matriz nula si det = 0 o es singular
        }

        float invDet = 1.0f / det;
        Matrix<3, 3> inv{};

        inv(0, 0) = ((*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1)) * invDet;
        inv(0, 1) = ((*this)(0, 2) * (*this)(2, 1) - (*this)(0, 1) * (*this)(2, 2)) * invDet;
        inv(0, 2) = ((*this)(0, 1) * (*this)(1, 2) - (*this)(0, 2) * (*this)(1, 1)) * invDet;

        inv(1, 0) = ((*this)(1, 2) * (*this)(2, 0) - (*this)(1, 0) * (*this)(2, 2)) * invDet;
        inv(1, 1) = ((*this)(0, 0) * (*this)(2, 2) - (*this)(0, 2) * (*this)(2, 0)) * invDet;
        inv(1, 2) = ((*this)(0, 2) * (*this)(1, 0) - (*this)(0, 0) * (*this)(1, 2)) * invDet;

        inv(2, 0) = ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0)) * invDet;
        inv(2, 1) = ((*this)(0, 1) * (*this)(2, 0) - (*this)(0, 0) * (*this)(2, 1)) * invDet;
        inv(2, 2) = ((*this)(0, 0) * (*this)(1, 1) - (*this)(0, 1) * (*this)(1, 0)) * invDet;

        return inv;
    }
};

// Sobrecarga escalar * matriz
template <size_t Rows, size_t Cols>
inline Matrix<Rows, Cols> operator*(float scalar, const Matrix<Rows, Cols>& mat) {
    return mat * scalar;
}

// Inversión analítica exacta para matrices 3x3 (función libre para compatibilidad)
inline Matrix<3, 3> inverse3x3(const Matrix<3, 3>& m) {
    return m.inverse3x3();
}

// Alias de tipos fijos utilizados en el Filtro de Kalman Extendido (EKF)
using Vector3f  = Matrix<3, 1>;
using Vector4f  = Matrix<4, 1>;
using Vector7f  = Matrix<7, 1>;
using Matrix3f  = Matrix<3, 3>;
using Matrix4f  = Matrix<4, 4>;
using Matrix7f  = Matrix<7, 7>;
using Matrix37f = Matrix<3, 7>;
using Matrix73f = Matrix<7, 3>;

#endif // MATRIX_HPP



