#include "ekf.hpp"
#include <cmath>

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    // Estado inicial por defecto: cuaternion unitario sin rotacion y sesgo cero
    x(0) = 1.0f; // q0
    x(1) = 0.0f; // q1
    x(2) = 0.0f; // q2
    x(3) = 0.0f; // q3
    x(4) = 0.0f; // bx
    x(5) = 0.0f; // by
    x(6) = 0.0f; // bz

    // Inicializacion de covarianza de estado inicial
    P = Matrix7f::identity() * 0.1f;

    // Configurar perfil por defecto (DRONE_HOVER)
    setProfile(flight::FlightProfileId::DRONE_HOVER);
}

void ExtendedKalmanFilter::setProfile(flight::FlightProfileId profile) {
    float q_gyro = 0.001f;
    float q_bias = 0.00001f;
    float r_accel = 0.05f;

    switch (profile) {
        case flight::FlightProfileId::DRONE_HOVER:
            q_gyro  = 0.001f;
            q_bias  = 0.000001f;
            r_accel = 0.01f;
            adaptiveAlpha = 15.0f;
            break;
        case flight::FlightProfileId::DRONE_ACRO:
            q_gyro  = 0.005f;
            q_bias  = 0.0001f;
            r_accel = 0.50f;
            adaptiveAlpha = 25.0f;
            break;
        case flight::FlightProfileId::ROCKET_LAUNCH:
            q_gyro  = 0.001f;
            q_bias  = 0.00001f;
            r_accel = 5.00f;
            adaptiveAlpha = 50.0f;
            break;
        case flight::FlightProfileId::MISSILE_HIGH_G:
            q_gyro  = 0.010f;
            q_bias  = 0.0005f;
            r_accel = 2.00f;
            adaptiveAlpha = 40.0f;
            break;
        default:
            q_gyro  = 0.001f;
            q_bias  = 0.00001f;
            r_accel = 0.05f;
            adaptiveAlpha = 15.0f;
            break;
    }

    // Configurar matriz de ruido de proceso Q (7x7)
    Q = Matrix7f{};
    Q(0, 0) = q_gyro;
    Q(1, 1) = q_gyro;
    Q(2, 2) = q_gyro;
    Q(3, 3) = q_gyro;
    Q(4, 4) = q_bias;
    Q(5, 5) = q_bias;
    Q(6, 6) = 0.0f; // Inobservable por gravedad (se mantiene calibrado en reposo por hardware)

    // Configurar matrices de ruido de medicion R (3x3)
    R_base = Matrix3f::identity() * r_accel;
    R = R_base;
}

void ExtendedKalmanFilter::setAdaptiveR(bool enable, float alpha) {
    useAdaptiveR = enable;
    adaptiveAlpha = alpha;
}

void ExtendedKalmanFilter::predict(const Vector3f& gyro, float dt) {
    // Guarda de dominio: dt debe ser estrictamente positivo y acotado (DO-178C Robustness)
    if (dt <= 0.0f || dt > 1.0f || !std::isfinite(dt)) {
        return;
    }

    // 1. Extraer estado actual
    float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);
    float bx = x(4), by = x(5), bz = x(6);

    // 2. Corregir velocidades angulares restando el sesgo estimado
    float wx = gyro(0) - bx;
    float wy = gyro(1) - by;
    float wz = gyro(2) - bz;

    // Filtro Zero-Motion Noise Gate: eliminar ruido residual si está en reposo o por debajo del umbral de ruido
    float gyro_norm = std::sqrt(wx * wx + wy * wy + wz * wz);
    constexpr float NOISE_THRESHOLD_RADS = 0.0018f; // ~0.10 deg/s (umbral de piso de ruido térmico)

    if (isStationary || gyro_norm < NOISE_THRESHOLD_RADS) {
        // En reposo estático, suprimir integración de ruido para eliminar deriva residual de Yaw
        wx = 0.0f;
        wy = 0.0f;
        wz = 0.0f;

        // Seguimiento suave del sesgo residual de Yaw en reposo
        if (isStationary) {
            x(6) += 0.001f * (gyro(2) - x(6));
        }
    }

    float halfDt = 0.5f * dt;

    // 3. Integración no lineal del cuaternión (cinemática de cuaterniones)
    x(0) += halfDt * (-q1 * wx - q2 * wy - q3 * wz);
    x(1) += halfDt * ( q0 * wx + q2 * wz - q3 * wy);
    x(2) += halfDt * ( q0 * wy - q1 * wz + q3 * wx);
    x(3) += halfDt * ( q0 * wz + q1 * wy - q2 * wx);
    // Los sesgos x(4), x(5), x(6) se modelan como caminatas aleatorias (constantes en predicción)

    // 4. Construcción de la matriz Jacobiana de proceso F (7x7)
    Matrix7f F_mat = Matrix7f::identity();

    // Bloque F_qq (4x4)
    F_mat(0, 1) = -halfDt * wx;  F_mat(0, 2) = -halfDt * wy;  F_mat(0, 3) = -halfDt * wz;
    F_mat(1, 0) =  halfDt * wx;  F_mat(1, 2) =  halfDt * wz;  F_mat(1, 3) = -halfDt * wy;
    F_mat(2, 0) =  halfDt * wy;  F_mat(2, 1) = -halfDt * wz;  F_mat(2, 3) =  halfDt * wx;
    F_mat(3, 0) =  halfDt * wz;  F_mat(3, 1) =  halfDt * wy;  F_mat(3, 2) = -halfDt * wx;

    // Bloque F_qb (4x3): derivadas con respecto al sesgo (d_omega/d_bias = -1)
    F_mat(0, 4) =  halfDt * q1;  F_mat(0, 5) =  halfDt * q2;  F_mat(0, 6) =  halfDt * q3;
    F_mat(1, 4) = -halfDt * q0;  F_mat(1, 5) =  halfDt * q3;  F_mat(1, 6) = -halfDt * q2;
    F_mat(2, 4) = -halfDt * q3;  F_mat(2, 5) = -halfDt * q0;  F_mat(2, 6) =  halfDt * q1;
    F_mat(3, 4) =  halfDt * q2;  F_mat(3, 5) = -halfDt * q1;  F_mat(3, 6) = -halfDt * q0;

    // 5. Propagación incondicional de la covarianza de error: P = F * P * F^T + Q * dt
    // En reposo (wx=wy=wz=0), F_mat = I y P = P + Q * dt, manteniendo la consistencia independiente de la frecuencia
    P = F_mat * P * F_mat.transpose() + (Q * dt);

    // Normalizar cuaternión tras predicción
    normalizeQuaternion();
}

void ExtendedKalmanFilter::update(const Vector3f& accel) {
    // 1. Normalizar la lectura del acelerometro a vector unitario de gravedad
    float aNorm = accel.norm();
    if (aNorm < 1e-4f) {
        return; // Lectura nula o caida libre
    }
    Vector3f z = accel * (1.0f / aNorm);

    // 2. Calculo de R dinamica adaptativa ante aceleraciones no gravitatorias (G-Gating)
    if (useAdaptiveR) {
        // En el sistema bare-metal, las lecturas del acelerómetro siempre se expresan en m/s^2 (SI)
        constexpr float gRef = flight::PhysicsConstants::GRAVITY_MSS;
        float gError = std::fabs(aNorm - gRef) / gRef; // Error relativo respecto a 1g
        float scaleFactor = 1.0f + adaptiveAlpha * (gError * gError);
        R = R_base * scaleFactor;
    } else {
        R = R_base;
    }

    // 3. Extraer cuaternion actual
    float q0 = x(0), q1 = x(1), q2 = x(2), q3 = x(3);

    // 4. Proyeccion no lineal de la gravedad esperada: h(x)
    Vector3f h{};
    h(0) = 2.0f * (q1 * q3 - q0 * q2);
    h(1) = 2.0f * (q0 * q1 + q2 * q3);
    h(2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 5. Residuo / Innovacion: y = z - h(x)
    Vector3f y = z - h;

    // 6. Jacobiano de medicion H (3x7)
    Matrix37f H{};
    H(0, 0) = -2.0f * q2;  H(0, 1) =  2.0f * q3;  H(0, 2) = -2.0f * q0;  H(0, 3) = 2.0f * q1;
    H(1, 0) =  2.0f * q1;  H(1, 1) =  2.0f * q0;  H(1, 2) =  2.0f * q3;  H(1, 3) = 2.0f * q2;
    H(2, 0) =  2.0f * q0;  H(2, 1) = -2.0f * q1;  H(2, 2) = -2.0f * q2;  H(2, 3) = 2.0f * q3;

    // 7. Covarianza de la innovacion: S = H * P * H^T + R (3x3)
    Matrix3f S = H * P * H.transpose() + R;

    // 7. Inversa exacta de S (3x3)
    Matrix3f S_inv = inverse3x3(S);

    // 8. Ganancia de Kalman: K = P * H^T * S_inv (7x3)
    Matrix73f K = P * H.transpose() * S_inv;

    // Desacoplar la dirección no observable de rotación alrededor de la vertical (Yaw)
    // Se proyecta ortogonalmente la corrección del cuaternión respecto al vector tangente de gravedad estimado h(x).
    // Esto generaliza el desacoplo de Yaw a cualquier actitud tridimensional (0 a 360 grados).
    const float gx = h(0), gy = h(1), gz = h(2);
    Vector4f nq{};
    nq(0) = -0.5f * (q1 * gx + q2 * gy + q3 * gz);
    nq(1) =  0.5f * (q0 * gx + q2 * gz - q3 * gy);
    nq(2) =  0.5f * (q0 * gy + q3 * gx - q1 * gz);
    nq(3) =  0.5f * (q0 * gz + q1 * gy - q2 * gx);

    float nqNorm = nq.norm();
    if (nqNorm > 1e-6f) {
        float invNq = 1.0f / nqNorm;
        nq(0) *= invNq;
        nq(1) *= invNq;
        nq(2) *= invNq;
        nq(3) *= invNq;

        for (size_t col = 0; col < 3; ++col) {
            float dot = nq(0) * K(0, col) + nq(1) * K(1, col) + nq(2) * K(2, col) + nq(3) * K(3, col);
            K(0, col) -= dot * nq(0);
            K(1, col) -= dot * nq(1);
            K(2, col) -= dot * nq(2);
            K(3, col) -= dot * nq(3);
        }
    }

    // Anular ganancia para el sesgo bz (inobservable en vuelo sin magnetómetro)
    K(6, 0) = 0.0f;
    K(6, 1) = 0.0f;
    K(6, 2) = 0.0f;

    // 9. Actualizacion del vector de estado estimado: x = x + K * y
    x += K * y;
    x(6) = 0.0f; // bz bloqueado a cero (compensado por calibración estática de reposo)

    // Limitar sesgos bx y by a rango fisico plausible (-0.05 a +0.05 rad/s)
    if (x(4) > 0.05f) x(4) = 0.05f;
    if (x(4) < -0.05f) x(4) = -0.05f;
    if (x(5) > 0.05f) x(5) = 0.05f;
    if (x(5) < -0.05f) x(5) = -0.05f;

    // Normalizar cuaternion inmediatamente tras la correccion
    normalizeQuaternion();

    // 10. Actualizacion de la covarianza de error mediante la forma estabilizada de Joseph:
    // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
    // Garantiza simetría y definición positiva incondicional para cualquier ganancia K (óptima o modificada).
    Matrix7f I7 = Matrix7f::identity();
    Matrix7f I_KH = I7 - (K * H);
    P = (I_KH * P * I_KH.transpose()) + (K * R * K.transpose());

    // Simetrizar y regularizar P para prevenir singularidades numericas
    for (size_t i = 0; i < 7; ++i) {
        for (size_t j = i + 1; j < 7; ++j) {
            float avg = 0.5f * (P(i, j) + P(j, i));
            P(i, j) = avg;
            P(j, i) = avg;
        }
        if (P(i, i) < 1e-6f) P(i, i) = 1e-6f;
        if (P(i, i) > 5.0f) P(i, i) = 5.0f;
    }
}

void ExtendedKalmanFilter::normalizeQuaternion() {
    float normSq = x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3);
    if (normSq > 1e-8f && std::isfinite(normSq)) {
        float invNorm = 1.0f / std::sqrt(normSq);
        x(0) *= invNorm;
        x(1) *= invNorm;
        x(2) *= invNorm;
        x(3) *= invNorm;
    } else {
        // Recuperacion anti-NaN: reajuste a cuaternion identidad si colapsa o es invalido
        x(0) = 1.0f;
        x(1) = 0.0f;
        x(2) = 0.0f;
        x(3) = 0.0f;
    }
}
