#include <iostream>
#include <cassert>
#include <cmath>
#include "matrix.hpp"

void test_matrix_basics() {
    std::cout << "[PRUEBA MATRIZ] Construccion basica e indexacion..." << std::endl;

    Matrix<3, 3> m{};
    for (size_t r = 0; r < 3; ++r) {
        for (size_t c = 0; c < 3; ++c) {
            assert(m(r, c) == 0.0f);
        }
    }

    m(0, 0) = 1.0f;
    m(1, 2) = 5.5f;
    m(2, 1) = -3.2f;

    assert(m(0, 0) == 1.0f);
    assert(m(1, 2) == 5.5f);
    assert(m(2, 1) == -3.2f);
    assert(m.data[1 * 3 + 2] == 5.5f);

    // Indexacion unidimensional para vectores columna
    Vector3f v{};
    v(0) = 10.0f;
    v(1) = 20.0f;
    v(2) = 30.0f;
    assert(v(0) == 10.0f && v(1) == 20.0f && v(2) == 30.0f);

    std::cout << "  -> Construccion basica superada con exito." << std::endl;
}

void test_matrix_identity() {
    std::cout << "[PRUEBA MATRIZ] Generacion de matriz identidad..." << std::endl;

    auto I3 = Matrix3f::identity();
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            if (i == j) {
                assert(std::fabs(I3(i, j) - 1.0f) < 1e-6f);
            } else {
                assert(std::fabs(I3(i, j)) < 1e-6f);
            }
        }
    }

    auto I7 = Matrix7f::identity();
    for (size_t i = 0; i < 7; ++i) {
        assert(std::fabs(I7(i, i) - 1.0f) < 1e-6f);
    }

    std::cout << "  -> Generacion de matriz identidad superada con exito." << std::endl;
}

void test_matrix_arithmetic() {
    std::cout << "[PRUEBA MATRIZ] Suma, resta y escalado escalar..." << std::endl;

    Matrix<2, 2> a{};
    a(0, 0) = 1.0f; a(0, 1) = 2.0f;
    a(1, 0) = 3.0f; a(1, 1) = 4.0f;

    Matrix<2, 2> b{};
    b(0, 0) = 5.0f; b(0, 1) = 6.0f;
    b(1, 0) = 7.0f; b(1, 1) = 8.0f;

    auto c = a + b;
    assert(c(0, 0) == 6.0f && c(0, 1) == 8.0f);
    assert(c(1, 0) == 10.0f && c(1, 1) == 12.0f);

    auto d = b - a;
    assert(d(0, 0) == 4.0f && d(0, 1) == 4.0f);
    assert(d(1, 0) == 4.0f && d(1, 1) == 4.0f);

    auto s1 = a * 2.5f;
    assert(s1(0, 0) == 2.5f && s1(0, 1) == 5.0f);
    assert(s1(1, 0) == 7.5f && s1(1, 1) == 10.0f);

    auto s2 = 2.0f * a;
    assert(s2(0, 0) == 2.0f && s2(0, 1) == 4.0f);

    a += b;
    assert(a(0, 0) == 6.0f && a(1, 1) == 12.0f);

    a -= b;
    assert(a(0, 0) == 1.0f && a(1, 1) == 4.0f);

    std::cout << "  -> Aritmetica matricial superada con exito." << std::endl;
}

void test_matrix_multiplication_and_transpose() {
    std::cout << "[PRUEBA MATRIZ] Multiplicacion matricial y transposicion..." << std::endl;

    Matrix<2, 3> A{};
    A(0, 0) = 1.0f; A(0, 1) = 2.0f; A(0, 2) = 3.0f;
    A(1, 0) = 4.0f; A(1, 1) = 5.0f; A(1, 2) = 6.0f;

    Matrix<3, 2> B{};
    B(0, 0) = 7.0f;  B(0, 1) = 8.0f;
    B(1, 0) = 9.0f;  B(1, 1) = 1.0f;
    B(2, 0) = 2.0f;  B(2, 1) = 3.0f;

    // C = A * B:
    // C(0,0) = 1*7 + 2*9 + 3*2 = 7 + 18 + 6 = 31
    // C(0,1) = 1*8 + 2*1 + 3*3 = 8 + 2 + 9  = 19
    // C(1,0) = 4*7 + 5*9 + 6*2 = 28 + 45 + 12 = 85
    // C(1,1) = 4*8 + 5*1 + 6*3 = 32 + 5 + 18 = 55
    Matrix<2, 2> C = A * B;
    assert(C(0, 0) == 31.0f);
    assert(C(0, 1) == 19.0f);
    assert(C(1, 0) == 85.0f);
    assert(C(1, 1) == 55.0f);

    // Transpuesta de A (3x2)
    Matrix<3, 2> At = A.transpose();
    assert(At(0, 0) == 1.0f && At(0, 1) == 4.0f);
    assert(At(1, 0) == 2.0f && At(1, 1) == 5.0f);
    assert(At(2, 0) == 3.0f && At(2, 1) == 6.0f);

    // Propiedad algebraica: (A*B)^T == B^T * A^T
    Matrix<2, 2> Ct = C.transpose();
    Matrix<2, 2> BtAt = B.transpose() * A.transpose();
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 2; ++j) {
            assert(std::fabs(Ct(i, j) - BtAt(i, j)) < 1e-5f);
        }
    }

    std::cout << "  -> Multiplicacion y transposicion superadas con exito." << std::endl;
}

void test_matrix_norm_and_blocks() {
    std::cout << "[PRUEBA MATRIZ] Norma euclidea y manipulacion de sub-bloques..." << std::endl;

    Vector3f v{};
    v(0) = 3.0f; v(1) = 4.0f; v(2) = 0.0f;
    assert(std::fabs(v.norm() - 5.0f) < 1e-6f);

    Vector4f q{};
    q(0) = 0.5f; q(1) = 0.5f; q(2) = 0.5f; q(3) = 0.5f;
    assert(std::fabs(q.norm() - 1.0f) < 1e-6f);

    // Operaciones con sub-bloques
    Matrix7f large = Matrix7f::identity();
    Matrix3f sub3{};
    sub3(0, 0) = 10.0f; sub3(0, 1) = 11.0f; sub3(0, 2) = 12.0f;
    sub3(1, 0) = 13.0f; sub3(1, 1) = 14.0f; sub3(1, 2) = 15.0f;
    sub3(2, 0) = 16.0f; sub3(2, 1) = 17.0f; sub3(2, 2) = 18.0f;

    large.setBlock<3, 3>(4, 4, sub3);
    assert(large(4, 4) == 10.0f && large(4, 5) == 11.0f && large(4, 6) == 12.0f);
    assert(large(6, 6) == 18.0f);

    Matrix<3, 3> extracted = large.block<3, 3>(4, 4);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            assert(extracted(i, j) == sub3(i, j));
        }
    }

    std::cout << "  -> Norma y sub-bloques superados con exito." << std::endl;
}

void test_matrix_inverse3x3() {
    std::cout << "[PRUEBA MATRIZ] Inversion analitica de matrices 3x3..." << std::endl;

    // 1. Inversa de matriz identidad
    Matrix3f I = Matrix3f::identity();
    Matrix3f I_inv = I.inverse3x3();
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            assert(std::fabs(I_inv(i, j) - expected) < 1e-6f);
        }
    }

    // 2. Inversa de matriz diagonal
    Matrix3f D{};
    D(0, 0) = 2.0f; D(1, 1) = 4.0f; D(2, 2) = 5.0f;
    Matrix3f D_inv = D.inverse3x3();
    assert(std::fabs(D_inv(0, 0) - 0.5f) < 1e-6f);
    assert(std::fabs(D_inv(1, 1) - 0.25f) < 1e-6f);
    assert(std::fabs(D_inv(2, 2) - 0.2f) < 1e-6f);

    // 3. Matriz general no singular
    Matrix3f M{};
    M(0, 0) = 1.0f; M(0, 1) = 2.0f; M(0, 2) = 3.0f;
    M(1, 0) = 0.0f; M(1, 1) = 1.0f; M(1, 2) = 4.0f;
    M(2, 0) = 5.0f; M(2, 1) = 6.0f; M(2, 2) = 0.0f;

    Matrix3f M_inv = M.inverse3x3();
    Matrix3f product = M * M_inv;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            assert(std::fabs(product(i, j) - expected) < 1e-4f);
        }
    }

    // 4. Matriz singular (determinante nulo det == 0)
    Matrix3f Sing{};
    Sing(0, 0) = 1.0f; Sing(0, 1) = 2.0f; Sing(0, 2) = 3.0f;
    Sing(1, 0) = 2.0f; Sing(1, 1) = 4.0f; Sing(1, 2) = 6.0f; // Fila 2 = 2 * Fila 1
    Sing(2, 0) = 7.0f; Sing(2, 1) = 8.0f; Sing(2, 2) = 9.0f;

    Matrix3f Sing_inv = Sing.inverse3x3();
    for (size_t i = 0; i < 9; ++i) {
        assert(Sing_inv.data[i] == 0.0f); // Se devuelve matriz nula de forma segura
    }

    std::cout << "  -> Inversion 3x3 superada con exito." << std::endl;
}

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "   Pruebas Unitarias del Motor Matricial    " << std::endl;
    std::cout << "============================================" << std::endl;

    test_matrix_basics();
    test_matrix_identity();
    test_matrix_arithmetic();
    test_matrix_multiplication_and_transpose();
    test_matrix_norm_and_blocks();
    test_matrix_inverse3x3();

    std::cout << "\n>>> TODAS LAS PRUEBAS MATRICIALES PASARON CON EXITO! <<<\n" << std::endl;
    return 0;
}