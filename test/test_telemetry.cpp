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
    std::cout << "[TEST] Checking packet structure sizes & zero-padding..." << std::endl;

    static_assert(sizeof(FrameHeader) == 4, "FrameHeader must be exactly 4 bytes");
    static_assert(sizeof(PayloadHeartbeat) == 9, "PayloadHeartbeat must be exactly 9 bytes");
    static_assert(sizeof(PayloadCmdSetProfile) == 1, "PayloadCmdSetProfile must be exactly 1 byte");
    static_assert(sizeof(PayloadBistReport) == 26, "PayloadBistReport must be exactly 26 bytes");
    static_assert(sizeof(PayloadTelemetry) == 74, "PayloadTelemetry must be exactly 74 bytes");
    static_assert(sizeof(PayloadAckNack) == 2, "PayloadAckNack must be exactly 2 bytes");

    assert(sizeof(FrameHeader) == 4);
    assert(sizeof(PayloadHeartbeat) == 9);
    assert(sizeof(PayloadCmdSetProfile) == 1);
    assert(sizeof(PayloadBistReport) == 26);
    assert(sizeof(PayloadTelemetry) == 74);
    assert(sizeof(PayloadAckNack) == 2);

    assert(sizeof(Packet<PayloadHeartbeat>) == 4 + 9 + 2);
    assert(sizeof(Packet<PayloadCmdSetProfile>) == 4 + 1 + 2);
    assert(sizeof(Packet<PayloadTelemetry>) == 4 + 74 + 2);

    std::cout << "  -> Structure sizes verified successfully." << std::endl;
}

void test_fletcher16_and_packet_integrity() {
    std::cout << "[TEST] Verifying Fletcher-16 checksum computation and validation..." << std::endl;

    // Test with Heartbeat packet
    Packet<PayloadHeartbeat> hb_pkt{};
    init_packet(hb_pkt, MsgId::HEARTBEAT_AWAIT_PROFILE);
    hb_pkt.payload.uptime_ms = 123456;
    hb_pkt.payload.system_state = static_cast<uint8_t>(SystemState::AWAITING_PROFILE);
    hb_pkt.payload.health_flags = HEALTH_FLAG_BIST_PASSED | HEALTH_FLAG_TELEMETRY_STREAMING;
    finalize_packet(hb_pkt);

    // Validate correct checksum
    bool is_valid = verify_checksum(
        hb_pkt.header.msg_id,
        hb_pkt.header.payload_len,
        reinterpret_cast<const uint8_t*>(&hb_pkt.payload),
        hb_pkt.checksum
    );
    assert(is_valid == true);

    // Corrupt one byte and verify checksum failure
    hb_pkt.payload.uptime_ms ^= 0x01;
    bool is_corrupt_detected = !verify_checksum(
        hb_pkt.header.msg_id,
        hb_pkt.header.payload_len,
        reinterpret_cast<const uint8_t*>(&hb_pkt.payload),
        hb_pkt.checksum
    );
    assert(is_corrupt_detected == true);

    std::cout << "  -> Fletcher-16 verification and corruption detection passed." << std::endl;
}

void test_telemetry_packet_roundtrip() {
    std::cout << "[TEST] Testing high-integrity telemetry packet serialization..." << std::endl;

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
    telem_pkt.payload.wcet_cycles = 14400; // ~60us at 240MHz
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

    std::cout << "  -> Telemetry serialization and checksum verified." << std::endl;
}

void test_fsm_transitions() {
    std::cout << "[TEST] Checking FSM state transitions and in-flight immutability..." << std::endl;

    // Normal startup flow
    assert(FlightFSM::is_transition_valid(SystemState::UNINITIALIZED, SystemState::AWAITING_PROFILE));
    assert(FlightFSM::is_transition_valid(SystemState::AWAITING_PROFILE, SystemState::BIST_AND_CALIBRATION));
    assert(FlightFSM::is_transition_valid(SystemState::BIST_AND_CALIBRATION, SystemState::RUNNING_ESTIMATOR));

    // Rule of In-flight Immutability:
    // Once in RUNNING_ESTIMATOR, transitions back to AWAITING_PROFILE or BIST are strictly forbidden
    assert(!FlightFSM::is_transition_valid(SystemState::RUNNING_ESTIMATOR, SystemState::AWAITING_PROFILE));
    assert(!FlightFSM::is_transition_valid(SystemState::RUNNING_ESTIMATOR, SystemState::BIST_AND_CALIBRATION));

    // Critical fault transitions are always allowed
    assert(FlightFSM::is_transition_valid(SystemState::RUNNING_ESTIMATOR, SystemState::HARD_FAULT_LOCK));
    assert(FlightFSM::is_transition_valid(SystemState::AWAITING_PROFILE, SystemState::HARD_FAULT_LOCK));

    // Lock state cannot transition anywhere
    assert(!FlightFSM::is_transition_valid(SystemState::HARD_FAULT_LOCK, SystemState::RUNNING_ESTIMATOR));

    std::cout << "  -> FSM transitions and immutability rules validated." << std::endl;
}

void test_flight_profiles() {
    std::cout << "[TEST] Checking static flight profiles table..." << std::endl;

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

    std::cout << "  -> Flight profiles static parameters validated." << std::endl;
}

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "  Running Flight Telemetry Unit Tests (PC)  " << std::endl;
    std::cout << "============================================" << std::endl;

    test_structure_sizes();
    test_fletcher16_and_packet_integrity();
    test_telemetry_packet_roundtrip();
    test_fsm_transitions();
    test_flight_profiles();

    std::cout << "\n>>> ALL TELEMETRY TESTS PASSED SUCCESSFULLY! <<<\n" << std::endl;
    return 0;
}
