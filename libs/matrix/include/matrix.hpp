//libreria header-only motor matricial para calculo con matrices que usará la libreria de EKF (extended Kalman Filter)
#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <array>
#include <cstddef>
#include <cmath>
#include <algorithm>

//las dimensiones se definen al compilar, de este modo el compilador sabe de antemano 
//el tamaño exacto en bytes de cada matriz para optimizar bucles antes de ejecutar el programa
template <size_t Rows, size_t Cols> 

struct Matrix {
    std::array<float, Rows * Cols> data{};  //matriz vector unidimensional rows x cols inicializados a 0.0f por ejem matriz 3x3 es una array de 9 posiciones

    //acceso a elementos fila/columna por ejemplo en una matriz 3x3, para acceder a la segunda fila (R1), tercera columna (C2) M(2,3) return -> 1 * 3 + 2 = 5
    //R/C C0 C1 C2
    //R0  11 12 13
    //R1  21 22 23
    //R2  31 32 33                    v
    //array[9] = {11, 12, 13, 21, 22, 23, 31, 32, 33}
    //array[9] = [0,  1,  2,  3,  4,  5,  6,  7,  8 ]
    //                                ^                    
    //escritura
    float& operator()(size_t r, size_t c){
        return data[r * Cols + c];
    }

    //lectura
    const float& operator()(size_t r, size_t c) const {
        return data[r * Cols + c];
    }

    //acceso a vectores (matrices N filas x 1 columna)
    //escritura
    float& operator()(size_t i){
        return data[i];
    }

    //lectura
    const float& operator()(size_t i) const{
        return data[i];
    }

    //construccion matriz identidad
    static Matrix<Rows, Cols> identity() {
        static_assert(Rows == Cols, "Matriz I debe ser cuadrada"); //si se intenta Matrix<3, 7>::identity() el codigo no compilará
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i < Rows; ++i) {
            res(i, i) = 1.0f;
        }
        return res;
    }

    //OPERACIONES MATRICIALES

    //suma de matrices
    //se crea nuevo objeto
    Matrix<Rows, Cols> operator+(const Matrix<Rows, Cols>& other) const {
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i < Rows * Cols; ++i){
            res.data[i] = this->data[i] + other.data[i];
        }
        return res;
    }

    //suma en el propio objeto
    Matrix<Rows, Cols>& operator+=(const Matrix<Rows,Cols>& other){
        for (size_t i = 0; i < Rows * Cols; ++i){
            this->data[i] += other.data[i];
        }
        return *this;
    }

    //resta de matrices
    Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols>& other) const {
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i< Rows * Cols; ++i){
            res.data[i] = this->data[i] - other.data[i];
        }
        return res;
    }

    Matrix<Rows, Cols>& operator-=(const Matrix<Rows, Cols>& other){
        for(size_t i = 0; i< Rows*Cols; ++i){
            this-> data[i] -= other.data[i];
        }
        return *this;
    }

    //producto por escalar
    Matrix<Rows, Cols> operator*(float scalar) const{
        Matrix<Rows, Cols> res{};
        for (size_t i = 0; i < Rows * Cols; ++i){
            res.data[i] = this->data[i] * scalar;
        }
        return res;
    }

    //multipiclacion de matrices (Rows x Cols)*(Cols x OtherCols)
    template <size_t OtherCols>
    Matrix<Rows, OtherCols> operator*(const Matrix<Cols, OtherCols>& other) const {
        Matrix<Rows, OtherCols> res{};
        for (size_t i = 0; i < Rows; ++i){
            for (size_t k = 0; k < Cols; k++){
                float val = (*this)(i, k);
                for (size_t j = 0; j < OtherCols; j++){
                    res(i, j) += val * other(k, j);
                }
            }
        }
        return res;
    }
    //transpuesta
    Matrix<Cols, Rows> transpose() const{
        Matrix<Cols, Rows> res{};
        for (size_t i = 0; i <Rows; ++i){
            for (size_t j = 0; j <Cols; ++j){
                res(j, i) = (*this)(i, j);
            }
        }
        return res;
    }
    

    //norma euclidea utilizada para normalizar lecturas del acelerometro y el cuaternion
    float norm() const{
        static_assert(Cols == 1, "Norma euclidea solo con vectores columna");
        float sum = 0.0f;
        for (size_t i = 0; i < Rows; ++i) {
            sum += data[i] * data[i];
        }    
        return std::sqrt(sum);
    }

    //extraccion bloque de matriz
    template <size_t BlockRows, size_t BlockCols>
    Matrix<BlockRows, BlockCols> block(size_t startRow, size_t startCol) const{
        Matrix<BlockRows, BlockCols> res{};
        for (size_t i = 0; i < BlockRows; ++i) {
            for (size_t j = 0; j < BlockCols; ++j) {
                res(i, j) = (*this)(startRow + i, startCol + j);
            }
        }
        return res;
    }

    //asignacion de sub-bloque
    template <size_t BlockRows, size_t BlockCols>
    void setBlock(size_t startRow, size_t startCol, const Matrix<BlockRows, BlockCols>& blockMat){
        for (size_t i = 0; i < BlockRows; ++i) {
            for (size_t j = 0; j < BlockCols; ++j) {
                (*this)(startRow + i, startCol + j) = blockMat(i, j);
            }
        }
    }

    // Inversion analitica para matrices 3x3
    Matrix<3, 3> inverse3x3() const {
        static_assert(Rows == 3 && Cols == 3, "inverse3x3 solo aplicable a matrices 3x3");
        float det = (*this)(0, 0) * ((*this)(1, 1) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 1)) -
                    (*this)(0, 1) * ((*this)(1, 0) * (*this)(2, 2) - (*this)(1, 2) * (*this)(2, 0)) +
                    (*this)(0, 2) * ((*this)(1, 0) * (*this)(2, 1) - (*this)(1, 1) * (*this)(2, 0));
        if (std::fabs(det) < 1e-7f || !std::isfinite(det)) {
            return Matrix<3, 3>{}; // Devuelve matriz nula si det = 0 o no numerico
        }

        float invDet = 1.0f / det; // Determinante inverso

        Matrix<3, 3> inv{}; // Inicializamos matriz inversa

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

// Inversion analitica exacta para matrices 3x3 (funcion libre para compatibilidad)
inline Matrix<3, 3> inverse3x3(const Matrix<3, 3>& m){
    return m.inverse3x3();
}

//alias de tipos fijos usado en el filtro de kalman extendido (EKF)
using Vector3f  = Matrix<3, 1>;
using Vector4f  = Matrix<4, 1>;
using Vector7f  = Matrix<7, 1>;
using Matrix3f  = Matrix<3, 3>;
using Matrix4f  = Matrix<4, 4>;
using Matrix7f  = Matrix<7, 7>;
using Matrix37f = Matrix<3, 7>;
using Matrix73f = Matrix<7, 3>;

#endif// MATRIX_HPP


