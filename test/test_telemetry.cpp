#include <iostream>
#include <cassert>
#include <cstring>
#include "../include/telemetry_protocol.hpp"
#include "../include/safety_types.hpp"
#include "../include/flight_profiles.hpp"
#include "../include/flight_fsm.hpp"

using namespace flight;
using namespace flight::protocol;

void test_structure_sizes() {
    std::cout << "[PRUEBA TELEMETRIA] Verificacion de tamanios de estructura y empaquetado binario..." << std::endl;

    static_assert(sizeof(FrameHeader) == 4, "FrameHeader debe ser exactamente de 4 bytes");
    static_assert(sizeof(PayloadHeartbeat) == 9, "PayloadHeartbeat debe ser exactamente de 9 bytes");
    static_assert(sizeof(PayloadCmdSetProfile) == 1, "PayloadCmdSetProfile debe ser exactamente de 1 byte");
    static_assert(sizeof(PayloadBistReport) == 26, "PayloadBistReport debe ser exactamente de 26 bytes");
    static_assert(sizeof(PayloadTelemetry) == 74, "PayloadTelemetry debe ser exactamente de 74 bytes");
    static_assert(sizeof(PayloadAckNack) == 2, "PayloadAckNack debe ser exactamente de 2 bytes");

    assert(sizeof(FrameHeader) == 4);
    assert(sizeof(PayloadHeartbeat) == 9);
    assert(sizeof(PayloadCmdSetProfile) == 1);
    assert(sizeof(PayloadBistReport) == 26);
    assert(sizeof(PayloadTelemetry) == 74);
    assert(sizeof(PayloadAckNack) == 2);

    assert(sizeof(Packet<PayloadHeartbeat>) == 4 + 9 + 2);
    assert(sizeof(Packet<PayloadCmdSetProfile>) == 4 + 1 + 2);
    assert(sizeof(Packet<PayloadTelemetry>) == 4 + 74 + 2);

    std::cout << "  -> Tamanios de estructura y empaquetado verificados con exito." << std::endl;
}

void test_fletcher16_and_packet_integrity() {
    std::cout << "[PRUEBA TELEMETRIA] Verificacion del calculo y validacion de suma Fletcher-16..." << std::endl;

    // Probar con trama de latido (Heartbeat)
    Packet<PayloadHeartbeat> hb_pkt{};
    init_packet(hb_pkt, MsgId::HEARTBEAT_AWAIT_PROFILE);
    hb_pkt.payload.uptime_ms = 123456;
    hb_pkt.payload.system_state = static_cast<uint8_t>(SystemState::AWAITING_PROFILE);
    hb_pkt.payload.health_flags = HEALTH_FLAG_BIST_PASSED | HEALTH_FLAG_TELEMETRY_STREAMING;
    finalize_packet(hb_pkt);

    // Validar suma de control correcta
    bool is_valid = verify_checksum(
        hb_pkt.header.msg_id,
        hb_pkt.header.payload_len,
        reinterpret_cast<const uint8_t*>(&hb_pkt.payload),
        hb_pkt.checksum
    );
    assert(is_valid == true);

    // Corromper un byte y comprobar deteccion de error
    hb_pkt.payload.uptime_ms ^= 0x01;
    bool is_corrupt_detected = !verify_checksum(
        hb_pkt.header.msg_id,
        hb_pkt.header.payload_len,
        reinterpret_cast<const uint8_t*>(&hb_pkt.payload),
        hb_pkt.checksum
    );
    assert(is_corrupt_detected == true);

    std::cout << "  -> Validacion Fletcher-16 y deteccion de corrupcion superadas con exito." << std::endl;
}

void test_telemetry_packet_roundtrip() {
    std::cout << "[PRUEBA TELEMETRIA] Serializacion de trama completa de telemetria EKF..." << std::endl;

    Packet<PayloadTelemetry> telem_pkt{};
    init_packet(telem_pkt, MsgId::ESTIMATOR_TELEMETRY);
    telem_pkt.payload.timestamp_us = 998877;
    telem_pkt.payload.q[0] = 1.0f;
    telem_pkt.payload.q[1] = 0.0f;
    telem_pkt.payload.q[2] = 0.0f;
    telem_pkt.payload.q[3] = 0.0f;
    telem_pkt.payload.euler_deg[0] = 12.5f;
    telem_pkt.payload.euler_deg[1] = -4.2f;
    telem_pkt.payload.euler_deg[2] = 180.0f;
    telem_pkt.payload.gyro_dps[0] = 0.1f;
    telem_pkt.payload.gyro_dps[1] = -0.2f;
    telem_pkt.payload.gyro_dps[2] = 0.05f;
    telem_pkt.payload.accel_g[0] = 0.01f;
    telem_pkt.payload.accel_g[1] = 0.02f;
    telem_pkt.payload.accel_g[2] = 0.98f;
    telem_pkt.payload.wcet_cycles = 14400; // ~60us a 240MHz
    telem_pkt.payload.wcet_us = 60.0f;
    telem_pkt.payload.loop_freq_hz = 500.0f;
    telem_pkt.payload.health_flags = HEALTH_FLAG_IMU_OK | HEALTH_FLAG_EKF_CONVERGED;
    telem_pkt.payload.system_state = static_cast<uint8_t>(SystemState::RUNNING_ESTIMATOR);
    telem_pkt.payload.active_profile_id = static_cast<uint8_t>(FlightProfileId::DRONE_ACRO);
    finalize_packet(telem_pkt);

    assert(telem_pkt.header.preamble[0] == PREAMBLE_0);
    assert(telem_pkt.header.preamble[1] == PREAMBLE_1);
    assert(telem_pkt.header.msg_id == static_cast<uint8_t>(MsgId::ESTIMATOR_TELEMETRY));
    assert(telem_pkt.header.payload_len == sizeof(PayloadTelemetry));

    bool valid = verify_checksum(
        telem_pkt.header.msg_id,
        telem_pkt.header.payload_len,
        reinterpret_cast<const uint8_t*>(&telem_pkt.payload),
        telem_pkt.checksum
    );
    assert(valid);

    std::cout << "  -> Serializacion de telemetria y suma de control verificadas con exito." << std::endl;
}

void test_fsm_transitions() {
    std::cout << "[PRUEBA TELEMETRIA] Comprobacion de transiciones de estado de la FSM e inmutabilidad en vuelo..." << std::endl;

    // Flujo nominal de arranque
    assert(FlightFSM::is_transition_valid(SystemState::UNINITIALIZED, SystemState::AWAITING_PROFILE));
    assert(FlightFSM::is_transition_valid(SystemState::AWAITING_PROFILE, SystemState::BIST_AND_CALIBRATION));
    assert(FlightFSM::is_transition_valid(SystemState::BIST_AND_CALIBRATION, SystemState::RUNNING_ESTIMATOR));

    // Regla de Inmutabilidad en vuelo:
    // Una vez en RUNNING_ESTIMATOR, las transiciones hacia atras estan estrictamente prohibidas
    assert(!FlightFSM::is_transition_valid(SystemState::RUNNING_ESTIMATOR, SystemState::AWAITING_PROFILE));
    assert(!FlightFSM::is_transition_valid(SystemState::RUNNING_ESTIMATOR, SystemState::BIST_AND_CALIBRATION));

    // Transiciones hacia estado de fallo critico siempre permitidas
    assert(FlightFSM::is_transition_valid(SystemState::RUNNING_ESTIMATOR, SystemState::HARD_FAULT_LOCK));
    assert(FlightFSM::is_transition_valid(SystemState::AWAITING_PROFILE, SystemState::HARD_FAULT_LOCK));

    // El estado de bloqueo HARD_FAULT_LOCK no permite transicionar a ningun otro estado
    assert(!FlightFSM::is_transition_valid(SystemState::HARD_FAULT_LOCK, SystemState::RUNNING_ESTIMATOR));

    std::cout << "  -> Transiciones de la FSM y reglas de inmutabilidad validadas con exito." << std::endl;
}

void test_flight_profiles() {
    std::cout << "[PRUEBA TELEMETRIA] Comprobacion de la tabla estatica de perfiles de vuelo..." << std::endl;

    const auto* p1 = get_profile_config(FlightProfileId::DRONE_HOVER);
    assert(p1 != nullptr && p1->rate_hz == 200 && p1->gyro_fs_dps == 250);

    const auto* p2 = get_profile_config(FlightProfileId::DRONE_ACRO);
    assert(p2 != nullptr && p2->rate_hz == 500 && p2->gyro_fs_dps == 1000);

    const auto* p3 = get_profile_config(FlightProfileId::ROCKET_LAUNCH);
    assert(p3 != nullptr && p3->rate_hz == 500 && p3->accel_fs_g == 16);

    const auto* p4 = get_profile_config(FlightProfileId::MISSILE_HIGH_G);
    assert(p4 != nullptr && p4->rate_hz == 1000 && p4->gyro_fs_dps == 2000);

    const auto* p_invalid = get_profile_config(static_cast<FlightProfileId>(99));
    assert(p_invalid == nullptr);

    std::cout << "  -> Parametros estaticos de perfiles de vuelo validados con exito." << std::endl;
}

void test_drdy_and_timer_watchdog() {
    std::cout << "[PRUEBA TELEMETRIA] Comprobacion de temporizaciones DRDY y Watchdog GPTimer..." << std::endl;

    // Verificar formulas de timeout de 1.5 * T_muestreo para cada perfil de vuelo
    const auto* p1 = get_profile_config(FlightProfileId::DRONE_HOVER);
    uint32_t t1_sample = 1000000UL / p1->rate_hz;
    uint32_t t1_timeout = (t1_sample * 3) / 2;
    assert(p1->rate_hz == 200);
    assert(t1_sample == 5000);
    assert(t1_timeout == 7500); // 7.5ms de timeout para 200Hz

    const auto* p2 = get_profile_config(FlightProfileId::DRONE_ACRO);
    uint32_t t2_sample = 1000000UL / p2->rate_hz;
    uint32_t t2_timeout = (t2_sample * 3) / 2;
    assert(p2->rate_hz == 500);
    assert(t2_sample == 2000);
    assert(t2_timeout == 3000); // 3.0ms de timeout para 500Hz

    const auto* p4 = get_profile_config(FlightProfileId::MISSILE_HIGH_G);
    uint32_t t4_sample = 1000000UL / p4->rate_hz;
    uint32_t t4_timeout = (t4_sample * 3) / 2;
    assert(p4->rate_hz == 1000);
    assert(t4_sample == 1000);
    assert(t4_timeout == 1500); // 1.5ms de timeout para 1000Hz (Bucle estricto de 1ms)

    // Verificar mascaras de bits de banderas de salud FDIR
    uint32_t flags = HEALTH_FLAG_NONE;
    assert((flags & HEALTH_FLAG_TIMER_FALLBACK_ACTIVE) == 0);
    flags |= HEALTH_FLAG_TIMER_FALLBACK_ACTIVE;
    assert((flags & HEALTH_FLAG_TIMER_FALLBACK_ACTIVE) != 0);

    std::cout << "  -> Temporizaciones DRDY/GPTimer y banderas FDIR validadas con exito." << std::endl;
}

#include "fdir_manager.hpp"

void test_fdir_manager() {
    std::cout << "[PRUEBA TELEMETRIA] Validando Gestor FDIR (G-gating, limites dinamicos, aislamiento I2C)..." << std::endl;

    FDIRManager::init(FlightProfileId::DRONE_HOVER);
    std::atomic<uint32_t> health_flags{HEALTH_FLAG_NONE};

    // 1. Probar envolvente gravitatoria nominal (1.0g)
    drivers::InertialScaledData nominal_data{};
    nominal_data.accel_g[0] = 0.0f;
    nominal_data.accel_g[1] = 0.0f;
    nominal_data.accel_g[2] = 1.0f; // 1g vertical
    nominal_data.gyro_dps[0] = 10.0f;
    nominal_data.gyro_dps[1] = 5.0f;
    nominal_data.gyro_dps[2] = 0.0f;

    bool can_fuse = FDIRManager::should_fuse_accelerometer(nominal_data, health_flags);
    assert(can_fuse == true);
    assert((health_flags.load() & HEALTH_FLAG_HIGH_G_REJECTION) == 0);

    // 2. Probar rechazo de choques y aceleraciones High-G (1.5g en HOVER con umbral 0.35g -> error = 0.5g > 0.35g)
    drivers::InertialScaledData shock_data{};
    shock_data.accel_g[0] = 0.0f;
    shock_data.accel_g[1] = 0.0f;
    shock_data.accel_g[2] = 1.5f; // Magnitud 1.5g -> |1.5 - 1.0| = 0.5g > 0.35g

    can_fuse = FDIRManager::should_fuse_accelerometer(shock_data, health_flags);
    assert(can_fuse == false);
    assert((health_flags.load() & HEALTH_FLAG_HIGH_G_REJECTION) != 0);
    assert(FDIRManager::get_high_g_event_count() == 1);

    // Recuperacion tras evento High-G
    can_fuse = FDIRManager::should_fuse_accelerometer(nominal_data, health_flags);
    assert(can_fuse == true);
    assert((health_flags.load() & HEALTH_FLAG_HIGH_G_REJECTION) == 0);

    // 3. Probar limites de velocidad angular y deteccion de anomalias
    drivers::InertialScaledData extreme_rate_data{};
    extreme_rate_data.gyro_dps[0] = 500.0f; // Supera la escala completa de 250 dps de DRONE_HOVER
    bool sample_safe = FDIRManager::process_sample(extreme_rate_data, 0.005f, health_flags);
    assert(sample_safe == false);
    assert((health_flags.load() & HEALTH_FLAG_ANOMALY_DETECTED) != 0);
    assert(FDIRManager::get_anomaly_count() == 1);

    // 4. Probar aislamiento de fallos del bus I2C (5 errores consecutivos -> HARD_FAULT)
    health_flags.store(HEALTH_FLAG_IMU_OK);
    FDIRManager::reset_diagnostics();

    for (int i = 1; i <= 4; ++i) {
        bool declare_hard_fault = FDIRManager::register_i2c_error(health_flags);
        assert(declare_hard_fault == false);
        assert(FDIRManager::get_consecutive_i2c_errors() == static_cast<uint32_t>(i));
        assert((health_flags.load() & HEALTH_FLAG_HARD_FAULT) == 0);
    }

    // El 5to error consecutivo activa la declaracion de fallo de hardware (HARD_FAULT)
    bool declare_hard_fault = FDIRManager::register_i2c_error(health_flags);
    assert(declare_hard_fault == true);
    assert((health_flags.load() & HEALTH_FLAG_HARD_FAULT) != 0);

    // Recuperacion ante comunicacion exitosa
    FDIRManager::register_i2c_success();
    assert(FDIRManager::get_consecutive_i2c_errors() == 0);

    std::cout << "  -> Deteccion de anomalias y aislamiento FDIR verificados con exito." << std::endl;
}

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "  Pruebas Unitarias de Telemetria (PC)      " << std::endl;
    std::cout << "============================================" << std::endl;

    test_structure_sizes();
    test_fletcher16_and_packet_integrity();
    test_telemetry_packet_roundtrip();
    test_fsm_transitions();
    test_flight_profiles();
    test_drdy_and_timer_watchdog();
    test_fdir_manager();

    std::cout << "\n>>> TODAS LAS PRUEBAS DE TELEMETRIA PASARON CON EXITO! <<<\n" << std::endl;
    return 0;
}


