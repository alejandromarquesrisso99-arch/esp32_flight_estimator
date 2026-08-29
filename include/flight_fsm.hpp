#pragma once

#include <cstdint>
#include "flight_profiles.hpp"

namespace flight {

/**
 * @brief Estados de ejecución del sistema de alta integridad
 */
enum class SystemState : uint8_t {
    UNINITIALIZED        = 0x00,
    AWAITING_PROFILE     = 0x01,
    BIST_AND_CALIBRATION = 0x02,
    RUNNING_ESTIMATOR    = 0x03,
    HARD_FAULT_LOCK      = 0x04
};

/**
 * @brief Convierte SystemState a cadena de texto para registro y depuración
 */
constexpr const char* state_to_string(SystemState state) {
    switch (state) {
        case SystemState::UNINITIALIZED:        return "UNINITIALIZED";
        case SystemState::AWAITING_PROFILE:     return "AWAITING_PROFILE";
        case SystemState::BIST_AND_CALIBRATION: return "BIST_AND_CALIBRATION";
        case SystemState::RUNNING_ESTIMATOR:    return "RUNNING_ESTIMATOR";
        case SystemState::HARD_FAULT_LOCK:      return "HARD_FAULT_LOCK";
        default:                                return "UNKNOWN";
    }
}

/**
 * @brief Controlador de máquina de estados finito (determinista y estático)
 */
class FlightFSM {
public:
    static constexpr bool is_transition_valid(SystemState current, SystemState next) {
        switch (current) {
            case SystemState::UNINITIALIZED:
                return (next == SystemState::AWAITING_PROFILE || next == SystemState::HARD_FAULT_LOCK);

            case SystemState::AWAITING_PROFILE:
                return (next == SystemState::BIST_AND_CALIBRATION || next == SystemState::HARD_FAULT_LOCK);

            case SystemState::BIST_AND_CALIBRATION:
                return (next == SystemState::RUNNING_ESTIMATOR || next == SystemState::HARD_FAULT_LOCK);

            case SystemState::RUNNING_ESTIMATOR:
                // Regla de Inmutabilidad: ¡Las transiciones de perfil en vuelo están prohibidas!
                // Solo se permite la transición a HARD_FAULT_LOCK o el reinicio global del sistema.
                return (next == SystemState::HARD_FAULT_LOCK);

            case SystemState::HARD_FAULT_LOCK:
                // Estado pozo de bloqueo permanente hasta reinicio físico o por software
                return false;

            default:
                return false;
        }
    }
};

} // namespace flight

