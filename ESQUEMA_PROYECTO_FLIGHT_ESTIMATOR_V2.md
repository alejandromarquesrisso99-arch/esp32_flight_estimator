# BluePrint Arquitectónico: High-Integrity Attitude Estimator (ESP32 / FreeRTOS)

---

## 1. Filosofía de Diseño y Principios de Tiempo Real Duro

1. **Cero Asignación Dinámica de Memoria (*Zero-Heap Policy*):**
   * Queda estrictamente prohibido el uso de `malloc`, `calloc`, `realloc`, `free`, `new`, `delete`, así como cualquier contenedor o abstracción de la STL con reserva en heap (`std::vector`, `std::string`, `std::function`).
   * Todas las tareas, colas, semáforos y bloques de control de FreeRTOS (`StaticTask_t`, `StaticQueue_t`, `StackType_t`) deben declararse e instanciarse en tiempo de compilación (`BSS` / `DATA` estático).
   * La biblioteca de matrices (`Matrix<Rows, Cols>`) e instanciación del EKF deben residir enteramente en memoria estática o en la pila asignada a la tarea GNC.
   * El driver I2C no debe utilizar llamadas a funciones con reserva dinámica por iteración (evitar la API obsoleta `i2c_cmd_link_create()` en bucle rápido; usar buffers estáticos preasignados o el driver `esp_driver_i2c` con descriptores fijos).

2. **Aislamiento Multinúcleo Estricto (Asymmetric Dual-Core Architecture):**
   * **Núcleo 1 (Dominio Crítico / Hard Real-Time):** Exclusivo para la lectura del bus I2C, la FDIR inercial, el cómputo matricial del EKF y el control de temporización. No ejecuta tareas de red, telemetría pesada ni I/O bloqueante.
   * **Núcleo 0 (Dominio de Servicio / Telemetría y Comunicaciones):** Exclusivo para el enlace UART/serie, empaquetado binario, checksum Fletcher-16, handshake de arranque con Processing y tareas de mantenimiento del sistema.
   * **Comunicación Inter-Núcleo:** Cola estática de un único elemento con política de sobrescritura (`xQueueOverwriteStatic`), garantizando que la tarea GNC jamás se bloquee por la velocidad del enlace de telemetría.

3. **Determinismo Temporal y Acotamiento de WCET:**
   * La ejecución del EKF y la adquisición inercial no se gestionan por sondeo (*polling*) con jitter, sino mediante la señal de interrupción hardware `DRDY` del sensor, respaldada por un temporizador del **Hardware Timer Group (GPTimer)**.
   * El tiempo de ejecución en el peor caso (**WCET**) se medirá ciclo a ciclo mediante el registro `CCOUNT` de la CPU Xtensa LX6 (240 MHz), verificando que $WCET < T_s$ en cualquier condición operativa.

---

## 2. Estructura de Directorios del Nuevo Proyecto

```text
Flight-Estimator-V2/
├── CMakeLists.txt                  # Build system CMake (soporte dual: ESP-IDF y PC Tests)
├── sdkconfig.defaults              # Configuración base fijada (FreeRTOS 1000Hz, CPU 240MHz, etc.)
├── include/
│   ├── flight_fsm.hpp              # Definición de estados formales y transiciones
│   ├── flight_profiles.hpp         # Configuraciones estáticas de los perfiles de vuelo
│   ├── safety_types.hpp            # Flags de salud (HealthFlags) y códigos BIST
│   └── telemetry_protocol.hpp      # Estructuras empaquetadas y comandos de comunicación
├── drivers/
│   └── mpu6050/
│       ├── CMakeLists.txt
│       ├── include/mpu6050_driver.hpp   # Driver baremetal con inyección de HAL estático
│       └── src/mpu6050_driver.cpp
├── libs/
│   ├── matrix/
│   │   ├── CMakeLists.txt
│   │   └── include/matrix.hpp           # Motor algebraico compile-time template (std::array)
│   └── ekf/
│       ├── CMakeLists.txt
│       ├── include/ekf.hpp              # Filtro de Kalman Extendido (Float32, Joseph Form)
│       └── src/ekf.cpp
├── src/
│   ├── CMakeLists.txt
│   ├── main.cpp                    # Punto de entrada `app_main`, creación de tareas estáticas
│   ├── gnc_task.cpp                # Bucle de alta prioridad en Core 1
│   ├── telemetry_task.cpp          # Bucle de telemetría y recepción en Core 0
│   ├── timer_watchdog.cpp          # Hardware Timer Group para fallback y sincronización
│   └── fdir_manager.cpp            # Lógica de detección y aislamiento de fallos (anti-NaN, stuck)
├── test/
│   ├── CMakeLists.txt
│   ├── test_matrix.cpp             # Tests unitarios del motor matricial en PC
│   ├── test_ekf.cpp                # Validación numérica de convergencia y covarianza
│   └── test_telemetry.cpp          # Validación de tramas binarias y checksums
└── tools/
    └── processing_visualizer/
        ├── FlightVisualizerV2.pde  # Interfaz gráfica: Selector de perfil + Visor 3D + Telemetría
        └── sketch.properties
```

---

## 3. Arquitectura de Sincronización e Interrupciones Hardware

```text
 +-----------------------------------------------------------------------------------+
 |                                   HARDWARE ESP32                                  |
 |                                                                                   |
 |  [MPU6050 Pin INT] ──> GPIO19 (Flanco Subida) ──> GPIO ISR (IRAM, Nivel 3)       |
 |                                                           |                       |
 |                                                           | vTaskNotifyGiveFromISR|
 |  [Hardware GPTimer] ──> Timeout Backup (1.2 * Ts) ────────+                       |
 +-----------------------------------------------------------|-----------------------+
                                                             |
                                                             v
 +-----------------------------------------------------------------------------------+
 | CORE 1: TAREA GNC (Prioridad Máxima: configMAX_PRIORITIES - 1, Pila Estática)     |
 |                                                                                   |
 | 1. ulTaskNotifyTake(pdTRUE, timeoutTicks)  [Despertar por DRDY o Timer Backup]   |
 | 2. Registro CCOUNT_start = esp_cpu_get_cycle_count()                              |
 | 3. Lectura I2C en ráfaga (14 bytes con buffer estático)                          |
 | 4. FDIR Inercial: Sanity check de acelerómetro y giroscopio                       |
 | 5. EKF Predict: Propagación de cinemática Y propagación obligatoria de P          |
 | 6. EKF Update: Corrección de gravedad con formulación simétrica/Joseph            |
 | 7. Verificación Anti-NaN (Reseteo atómico de X y reinflado de P ante corrupción)  |
 | 8. Registro CCOUNT_end = esp_cpu_get_cycle_count() -> Calcular WCET = (end-start)|
 | 9. xQueueOverwriteStatic(&telemetryQueue, &currentPacket)                         |
 +-----------------------------------------------------------|-----------------------+
                                                             | (Lock-Free)
                                                             v
 +-----------------------------------------------------------------------------------+
 | CORE 0: TAREA TELEMETRÍA Y COMUNICACIÓN (Prioridad 3, Pila Estática)             |
 |                                                                                   |
 | 1. Recepción periódica no bloqueante desde `telemetryQueue`                       |
 | 2. Serialización binaria empaquetada (#pragma pack(push, 1))                      |
 | 3. Cálculo de Checksum Fletcher-16                                                |
 | 4. Transmisión UART hacia Processing a 30-50 Hz                                   |
 | 5. Monitorización de comandos entrantes de Processing (Handshake de perfil)       |
 +-----------------------------------------------------------------------------------+
```

---

## 4. Protocolo de Arranque y Handshake Bidireccional con Processing

Para evitar que el ESP32 inicie con un perfil arbitrario sin control del operador, se implementa una máquina de estados de arranque determinista:

### Flujo de Selección de Perfil
```text
[ESP32 Power-On / Reset]
           │
           ▼
[Estado: AWAITING_PROFILE] ──(Emite 'HEARTBEAT_AWAIT_PROFILE' por UART a 10 Hz)──> [Processing]
           │                                                                             │
           │                                                                    (Muestra Menú UI)
           │                                                                    [1] Drone Hover
           │                                                                    [2] Drone Acro
           │                                                                    [3] Rocket Launch
           │                                                                    [4] Missile High-G
           │                                                                             │
           │ <───[Trama de Configuración: CMD_SET_PROFILE + ID + Checksum] ─── (Pulsación Tecla)
           ▼
[Validación de Trama]
 ├── Checksum Inválido ──> Permanece en AWAITING_PROFILE
 └── Checksum Válido   ──> Bloquea perfil en tiempo de ejecución (Inmutable)
           │
           ▼
[Estado: BIST_AND_CALIBRATION] (Ejecuta test ALU, ruido de sensor, calibración de sesgos)
           │
           ├── Fallo Crítico ──> [Estado: HARD_FAULT_LOCK] (Bloqueo de seguridad)
           └── Éxito         ──> [Estado: RUNNING_ESTIMATOR]
                                       │
                                       └── (Comienza emisión continua de telemetría 3D)
```

* **Regla de Inmutabilidad:** Una vez seleccionado el perfil y alcanzado el estado `RUNNING_ESTIMATOR`, **no se permite el cambio dinámico de perfil en vuelo**. Si se requiere otro perfil, el operador debe pulsar el botón de Reset físico del ESP32 o disparar un comando de Software Reset global, reiniciando la secuencia desde cero.

---

## 5. Especificaciones para las Librerías Matemáticas y EKF

Las librerías `libs/matrix` y `libs/ekf` se conservan en su núcleo conceptual, pero con los siguientes ajustes obligatorios:

1. **`libs/matrix`:**
   * Almacenamiento 100 % interno mediante `std::array<float, Rows * Cols>`.
   * Métodos `constexpr` e inline para operaciones fundamentales (suma, resta, multiplicación con bucle `i-k-j`, transposición).
   * Inversión analítica $3 \times 3$ protegida con comprobación estricta de determinante mínimo $|\det| > 10^{-7}$ para evitar división por cero o singularidades numéricas.

2. **`libs/ekf`:**
   * **Propagación Continua de Covarianza:** Eliminar cualquier retorno anticipado en `predict()`. Si la velocidad angular cae dentro de una banda muerta, se integra $\omega = 0$, pero la propagación de la covarianza $P_{k|k-1} = F P_{k-1|k-1} F^T + Q$ **debe ejecutarse obligatoriamente en cada ciclo** para evitar que el filtro quede ciego ante cambios de actitud tras reposo.
   * **Simetría y Definición Positiva:** Actualización de $P$ utilizando simetrización explícita $P = \frac{1}{2}(P + P^T)$ y suelo mínimo en la diagonal principal ($P_{ii} \ge 10^{-6}$).
   * **Manejo Anti-NaN Integral:** Si se detecta un NaN en cualquier componente del cuaternión o de la matriz de covarianza, el mecanismo FDIR reinicializa el estado a $q = [1, 0, 0, 0]^T$ y re-infla la matriz $P$ con su valor inicial $P_0$.

---

## 6. Módulo Processing (Estación Terrena)

El nuevo sketch de Processing se divide en tres vistas claramente diferenciadas:

1. **Pantalla 1: Menú de Selección de Perfil (Pre-Flight):**
   * Muestra el estado del enlace serie (esperando señal de heartbeat del ESP32).
   * Presenta una tabla con las características de cada perfil:
     * **[1] DRONE_HOVER:** $\pm 250^\circ/\text{s}, \pm 2\text{g}, 200\text{ Hz}$ (Alta resolución, baja dinámica).
     * **[2] DRONE_ACRO:** $\pm 1000^\circ/\text{s}, \pm 8\text{g}, 500\text{ Hz}$ (Maniobra rápida).
     * **[3] ROCKET_LAUNCH:** $\pm 1000^\circ/\text{s}, \pm 16\text{g}, 500\text{ Hz}$ (Rechazo de aceleración axial).
     * **[4] MISSILE_HIGH_G:** $\pm 2000^\circ/\text{s}, \pm 16\text{g}, 1000\text{ Hz}$ (Dinámica extrema).
   * Captura la pulsación de tecla ('1', '2', '3', '4') y envía la trama binaria correspondiente.

2. **Pantalla 2: Vista de Inicialización y BIST:**
   * Renderiza el progreso de calibración de sesgos del MPU6050 y el resultado de los autodiagnósticos (ALU test, ruido inercial, coherencia de interrupciones).

3. **Pantalla 3: Visualizador 3D y Dashboard Operativo:**
   * Renderizado en 3D del vehículo (utilizando el cuaternión normalizado recibido).
   * Visualización de ángulos de Euler (Roll, Pitch, Yaw).
   * Monitores en tiempo real de:
     * Frecuencia real del lazo GNC ($f_s$).
     * Tiempo de ejecución medido (**WCET** en microsegundos y ciclos de CPU).
     * Banderas de salud (`HealthFlags`): `IMU_OK`, `EKF_CONVERGED`, `TIMER_FALLBACK_ACTIVE`, `HIGH_G_REJECTION`, `ANOMALY_DETECTED`.

---

## 7. Hoja de Ruta de Implementación (Paso a Paso)

Para garantizar que el nuevo proyecto no sufra problemas de comunicación o bloqueos, el desarrollo debe seguir esta secuencia rigurosa:

### **Fase 1: Configuración Base y Comunicación Serie Robusta**
1. Crear el nuevo directorio y configurar `CMakeLists.txt` y `sdkconfig.defaults` (fijando tick de FreeRTOS a 1000 Hz, frecuencia de CPU a 240 MHz).
2. Implementar `include/telemetry_protocol.hpp` con el empaquetado binario estricto (`#pragma pack(push, 1)`) y checksum Fletcher-16.
3. Crear `telemetry_task.cpp` en el **Núcleo 0** e implementar el handshake con Processing.
4. Escribir el sketch de Processing para verificar el menú de selección de perfil y el handshake bidireccional estable.

### **Fase 2: HAL de I2C Estático y Driver MPU6050**
1. Implementar la capa I2C sobre ESP-IDF sin reserva dinámica de memoria en runtime.
2. Adaptar el driver `mpu6050_driver.cpp` para configurar dinámicamente rangos de acelerómetro, giroscopio y DLPF según el perfil seleccionado.
3. Validar la lectura en ráfaga de 14 bytes y verificar que la tasa de errores de bus sea cero.

### **Fase 3: Sincronización Hardware (DRDY + GPTimer Backup)**
1. Configurar la interrupción GPIO para el pin DRDY del MPU6050 conectada a una ISR en IRAM.
2. Configurar un temporizador por hardware (Hardware Timer Group) como respaldo *watchdog*. Si no llega la interrupción DRDY en $1.2 \times T_s$, el temporizador dispara la ejecución de la tarea GNC marcando la bandera `TIMER_FALLBACK_ACTIVE`.
3. Validar la sincronización en osciloscopio o mediante GPIO de depuración.

### **Fase 4: Integración del Motor EKF y FDIR en Núcleo 1**
1. Integrar `libs/matrix` y `libs/ekf` en `gnc_task.cpp`.
2. Asignar la tarea GNC de forma estática (`xTaskCreateStaticPinnedToCore`) al **Núcleo 1** con prioridad `configMAX_PRIORITIES - 1`.
3. Implementar el control de ciclos con `esp_cpu_get_cycle_count()` para calcular el WCET real del lazo.
4. Conectar la salida del EKF con la cola estática `telemetryQueue` hacia el Núcleo 0.

### **Fase 5: FDIR y Pruebas de Esfuerzo (Stress Testing)**
1. Validar la detección de sensor desconectado, sensor congelado e inyección de ruido.
2. Comprobar que en reposo el filtro no colapse su matriz de covarianza.
3. Ejecutar los perfiles a 200 Hz, 500 Hz y 1000 Hz, registrando que en todos los casos el WCET se mantenga por debajo del límite de tiempo asignado.