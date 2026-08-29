/**
 * FlightVisualizerV2.pde
 * Estacion Terrena para Estimador de Actitud de Alta Integridad (Processing 4 / P3D)
 * 
 * Caracteristicas:
 * - Negociacion y seleccion de perfil FSM (Pantalla 1)
 * - Calibracion y monitorizacion BIST (Pantalla 2)
 * - Visualizador 3D de actitud con rejilla de referencia espacial (Pantalla 3)
 * - Validacion estricta de suma de control Fletcher-16 en todas las tramas binarias
 * - Modo DEMO / Simulacion interactiva (Pulsar 'D') para pruebas fuera de linea
 */

import processing.serial.*;

// Constantes de protocolo coincidentes con telemetry_protocol.hpp
final int PREAMBLE_0 = 0xAA;
final int PREAMBLE_1 = 0x55;

final int MSG_HEARTBEAT_AWAIT_PROFILE = 0x01;
final int MSG_CMD_SET_PROFILE         = 0x02;
final int MSG_BIST_REPORT             = 0x03;
final int MSG_ESTIMATOR_TELEMETRY     = 0x04;
final int MSG_ACK_NACK                = 0x05;
final int MSG_CMD_SYSTEM_RESET        = 0x06;

// Mascara de bits para banderas de salud
final int FLAG_IMU_OK                = (1 << 0);
final int FLAG_EKF_CONVERGED         = (1 << 1);
final int FLAG_TIMER_FALLBACK_ACTIVE = (1 << 2);
final int FLAG_HIGH_G_REJECTION      = (1 << 3);
final int FLAG_ANOMALY_DETECTED      = (1 << 4);
final int FLAG_BIST_PASSED           = (1 << 5);
final int FLAG_HARD_FAULT            = (1 << 6);
final int FLAG_TELEMETRY_STREAMING   = (1 << 7);

// Estados del sistema en la interfaz de usuario
final int UI_STATE_PORT_SELECT       = 0;
final int UI_STATE_AWAITING_PROFILE  = 1;
final int UI_STATE_BIST_CALIBRATION  = 2;
final int UI_STATE_RUNNING_ESTIMATOR = 3;
final int UI_STATE_HARD_FAULT        = 4;

int currentUiState = UI_STATE_PORT_SELECT;
Serial serialPort = null;
String selectedPortName = "SIN PUERTO SERIE (DEMO)";
int baudRate = 115200;
boolean isDemoMode = false;
float demoSimTime = 0.0f;

// Variables de estado de telemetria
long lastPacketTimeMs = 0;
int rxPacketCount = 0;
int checksumErrors = 0;

// Latido (Heartbeat) y BIST
long espUptimeMs = 0;
int espSystemState = 0;
int healthFlags = FLAG_IMU_OK | FLAG_EKF_CONVERGED | FLAG_BIST_PASSED | FLAG_TELEMETRY_STREAMING;
int bistCode = 0;
int bistProgressPct = 0;

// Datos de telemetria del estimador
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;
float gyroDpsX = 0.0f, gyroDpsY = 0.0f, gyroDpsZ = 0.0f;
float accelGX = 0.0f, accelGY = 0.0f, accelGZ = 0.0f;
long wcetCycles = 14280;
float wcetUs = 59.5f;
float loopFreqHz = 500.0f;
int activeProfileId = 1;

// Maquina de estados para recepcion serie
int rxState = 0;
int rxMsgId = 0;
int rxLen = 0;
byte[] rxBuffer = new byte[256];
int rxBufIdx = 0;
int rxChkLow = 0;

void settings() {
  size(1120, 740, P3D);
}

void setup() {
  surface.setTitle("Estimador de Vuelo de Alta Integridad V2 - Estacion Terrena");
  textFont(createFont("Consolas", 14));
  frameRate(60);
}

void draw() {
  background(15, 20, 28);
  
  // Leer y procesar flujo serie si esta conectado
  processSerial();

  // Ejecutar simulacion fisica en modo demo
  if (isDemoMode) {
    updateDemoSimulation();
  }

  switch (currentUiState) {
    case UI_STATE_PORT_SELECT:
      drawPortSelectionScreen();
      break;
    case UI_STATE_AWAITING_PROFILE:
      drawProfileSelectionScreen();
      break;
    case UI_STATE_BIST_CALIBRATION:
      drawBistScreen();
      break;
    case UI_STATE_RUNNING_ESTIMATOR:
      drawDashboardScreen();
      break;
    case UI_STATE_HARD_FAULT:
      drawHardFaultScreen();
      break;
  }
}

// -------------------------------------------------------------
// RENDERIZADO DE PANTALLAS DE INTERFAZ DE USUARIO
// -------------------------------------------------------------

void drawPortSelectionScreen() {
  hint(DISABLE_DEPTH_TEST);
  
  fill(0, 200, 255);
  textSize(24);
  textAlign(CENTER, TOP);
  text("ESTIMADOR DE VUELO V2 - ESTACION TERRENA", width/2, 40);

  textSize(14);
  fill(180, 200, 220);
  text("Seleccione el puerto serie COM conectado al ESP32 (Baudios: 115200)", width/2, 80);

  String[] ports = Serial.list();
  float startY = 130;
  
  if (ports != null && ports.length > 0) {
    for (int i = 0; i < ports.length; i++) {
      float y = startY + i * 50;
      boolean hover = (mouseX > width/2 - 160 && mouseX < width/2 + 160 && mouseY > y && mouseY < y + 38);
      
      fill(hover ? color(30, 80, 120) : color(25, 40, 55));
      stroke(hover ? color(0, 220, 255) : color(50, 80, 110));
      rect(width/2 - 160, y, 320, 38, 6);

      fill(255);
      textAlign(CENTER, CENTER);
      text("[" + (i + 1) + "]  CONECTAR A " + ports[i], width/2, y + 19);
    }
  } else {
    fill(255, 120, 120);
    textAlign(CENTER, CENTER);
    text("No hay puertos serie fisicos disponibles.", width/2, 160);
  }

  // Tarjeta de simulacion en modo demo
  float demoY = (ports != null && ports.length > 0) ? startY + ports.length * 50 + 30 : 220;
  boolean hoverDemo = (mouseX > width/2 - 160 && mouseX < width/2 + 160 && mouseY > demoY && mouseY < demoY + 45);

  fill(hoverDemo ? color(20, 120, 80) : color(18, 70, 50));
  stroke(hoverDemo ? color(0, 255, 180) : color(40, 150, 100));
  strokeWeight(2);
  rect(width/2 - 160, demoY, 320, 45, 6);

  fill(0, 255, 200);
  textAlign(CENTER, CENTER);
  textSize(14);
  text("[D]  MODO SIMULACION 3D (SIN ESP32)", width/2, demoY + 22);

  fill(140, 170, 190);
  textSize(12);
  textAlign(CENTER, BOTTOM);
  text("Puede pulsar 'D' en cualquier momento para ver la actitud 3D y telemetria fuera de linea.", width/2, height - 25);
}

void drawProfileSelectionScreen() {
  hint(DISABLE_DEPTH_TEST);

  fill(0, 220, 255);
  textSize(22);
  textAlign(CENTER, TOP);
  text("CONFIGURACION PRE-VUELO - SELECCION DE PERFIL", width/2, 30);

  fill(160, 190, 210);
  textSize(13);
  text("Puerto: " + selectedPortName + " | Estado ESP32: AWAITING_PROFILE", width/2, 65);

  // Tarjetas de perfiles de vuelo
  drawProfileCard(1, "DRONE_HOVER", "200 Hz", "+/- 250 dps", "+/- 2 g", "DLPF ~98Hz | Vuelo estacionario suave de alta resolucion", 80, 110, 440, 240);
  drawProfileCard(2, "DRONE_ACRO", "500 Hz", "+/- 1000 dps", "+/- 8 g", "DLPF ~188Hz | Dinamica rapida y maniobras acrobaticas", 580, 110, 440, 240);
  drawProfileCard(3, "ROCKET_LAUNCH", "500 Hz", "+/- 1000 dps", "+/- 16 g", "DLPF ~188Hz | Rechazo a alta aceleracion axial de lanzamiento", 80, 380, 440, 240);
  drawProfileCard(4, "MISSILE_HIGH_G", "1000 Hz", "+/- 2000 dps", "+/- 16 g", "DLPF ~256Hz | Dinamica extrema (Bucle duro de 1ms)", 580, 380, 440, 240);

  fill(255, 230, 100);
  textSize(14);
  textAlign(CENTER, BOTTOM);
  text("Pulse la tecla '1', '2', '3', '4' (o haga clic en una tarjeta) para seleccionar perfil | 'D' para Demo", width/2, height - 20);
}

void drawProfileCard(int id, String name, String freq, String gyro, String accel, String desc, float x, float y, float w, float h) {
  boolean hover = (mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h);
  fill(hover ? color(28, 52, 75) : color(22, 34, 48));
  stroke(hover ? color(0, 220, 255) : color(45, 70, 95));
  strokeWeight(hover ? 2 : 1);
  rect(x, y, w, h, 8);

  fill(0, 220, 255);
  textAlign(LEFT, TOP);
  textSize(16);
  text("[" + id + "] " + name, x + 20, y + 16);

  fill(200, 220, 240);
  textSize(13);
  text("Frecuencia Objetivo:  " + freq, x + 20, y + 55);
  text("Rango de Giroscopo:   " + gyro, x + 20, y + 80);
  text("Rango Acelerometro:   " + accel, x + 20, y + 105);

  fill(140, 170, 200);
  textSize(12);
  text(desc, x + 20, y + 145, w - 40, 60);

  fill(hover ? color(0, 255, 200) : color(80, 130, 180));
  rect(x + w - 100, y + h - 35, 85, 24, 4);
  fill(hover ? 0 : 255);
  textAlign(CENTER, CENTER);
  text("SELECCIONAR", x + w - 57, y + h - 23);
}

void drawBistScreen() {
  hint(DISABLE_DEPTH_TEST);

  fill(0, 220, 255);
  textSize(22);
  textAlign(CENTER, TOP);
  text("INICIALIZACION DE HARDWARE Y CALIBRACION ESP32", width/2, 100);

  fill(180, 200, 220);
  textSize(14);
  text("Calibrando sesgos de sensores inerciales y ejecutando autodiagnostico BIST...", width/2, 150);

  // Barra de progreso
  noFill();
  stroke(50, 100, 150);
  rect(width/2 - 200, 220, 400, 24, 6);

  fill(0, 220, 255);
  noStroke();
  float pWidth = map(bistProgressPct, 0, 100, 0, 396);
  rect(width/2 - 198, 222, pWidth, 20, 4);

  fill(255);
  textAlign(CENTER, CENTER);
  text(bistProgressPct + "%", width/2, 232);

  fill(160, 200, 220);
  textSize(13);
  textAlign(CENTER, BOTTOM);
  text("Perfil Fijado: " + activeProfileId + " | Esperando arranque del lazo del estimador...", width/2, 300);
}

void drawDashboardScreen() {
  // 1. Dibujar Visualizador de Actitud 3D (Panel Izquierdo)
  draw3DVisualizer(30, 35, 540, 670);

  // 2. Dibujar Panel de Telemetria y Salud (Panel Derecho)
  drawTelemetryPanel(600, 35, 490, 670);
}


// -------------------------------------------------------------
// VISUALIZADOR 3D DE ACTITUD (PLACA PLANA)
// -------------------------------------------------------------

void draw3DVisualizer(float x, float y, float w, float h) {
  // Paso 1: Fondo del panel en 2D
  hint(DISABLE_DEPTH_TEST);
  fill(12, 18, 26);
  stroke(35, 60, 85);
  strokeWeight(1.5);
  rect(x, y, w, h, 8);

  // Paso 2: Renderizado 3D con prueba de profundidad
  hint(ENABLE_DEPTH_TEST);
  pushMatrix();

  // Centrar espacio de trabajo 3D
  translate(x + w/2, y + h/2 + 25, 0);

  // Proyeccion isometrica espacial (vista diagonal superior-frontal)
  rotateX(-atan(1.0f / sqrt(2.0f))); // -35.264°: inclinacion superior
  rotateY(radians(135));              // 135°: orientacion hacia el frente derecho de la placa

  // Iluminacion 3D dinamica
  lights();
  ambientLight(90, 110, 140);
  directionalLight(230, 245, 255, -0.4, 0.9, -0.6);
  
  // 1. Rejilla de referencia en el suelo horizontal
  drawGroundReferenceGrid(160, 32, 110);

  // 2. Aplicar matriz de rotacion del cuaternion de actitud:
  // IMU Y (Pitch) -> Processing X (Inclinacion lateral)
  // IMU Z (Yaw)   -> Processing Y (Giro vertical)
  // IMU X (Roll)  -> Processing Z (Alabeo longitudinal)
  pushMatrix();
  applyQuaternionRotation(q0, q2, -q3, -q1);

  // 3. Renderizado del modelo plano de la placa IMU
  drawFlatBoardModel();

  popMatrix();
  popMatrix();

  // Paso 3: Superposiciones HUD 2D
  hint(DISABLE_DEPTH_TEST);

  fill(0, 220, 255);
  textSize(16);
  textAlign(LEFT, TOP);
  text("ESTIMADOR DE ACTITUD 3D (PLACA IMU)", x + 20, y + 18);

  // Cuadro HUD de Angulos de Euler
  fill(20, 35, 50, 220);
  stroke(0, 200, 255, 120);
  strokeWeight(1);
  rect(x + 15, y + 50, 220, 82, 6);

  fill(0, 220, 255);
  textSize(13);
  textAlign(LEFT, TOP);
  text("ROLL:  " + nf(rollDeg, 1, 2) + " deg", x + 25, y + 58);
  text("PITCH: " + nf(pitchDeg, 1, 2) + " deg", x + 25, y + 78);
  text("YAW:   " + nf(yawDeg, 1, 2) + " deg", x + 25, y + 98);

  if (isDemoMode) {
    fill(0, 255, 180);
    textAlign(RIGHT, TOP);
    text("[SIMULACION DEMO]", x + w - 20, y + 20);
  }
}

void drawGroundReferenceGrid(float size, float step, float yOffset) {
  pushMatrix();
  translate(0, yOffset, 0); // Posicion inferior en el espacio 3D

  // Lineas de rejilla en plano X-Z
  strokeWeight(1);
  for (float i = -size; i <= size + 0.1f; i += step) {
    // Lineas longitudinales (a lo largo de Z)
    stroke(0, 180, 255, (abs(i) < 0.1f) ? 140 : 45);
    line(i, 0, -size, i, 0, size);

    // Lineas laterales (a lo largo de X)
    stroke(0, 180, 255, (abs(i) < 0.1f) ? 140 : 45);
    line(-size, 0, i, size, 0, i);
  }

  // Borde exterior de la plataforma cuadrada
  stroke(0, 220, 255, 120);
  strokeWeight(1.5);
  line(-size, 0, -size, size, 0, -size);
  line(size, 0, -size, size, 0, size);
  line(size, 0, size, -size, 0, size);
  line(-size, 0, size, -size, 0, -size);

  // Ejes cardinales centrales en el plano inferior
  strokeWeight(2.5);
  stroke(255, 70, 70, 200); line(0, 0, 0, 0, 0, -size); // Frente (-Z)
  stroke(70, 255, 70, 200); line(0, 0, 0, size, 0, 0);  // Derecha (+X)

  popMatrix();
}

void drawFlatBoardModel() {
  // 1. Ejes de coordenadas del cuerpo 3D
  strokeWeight(3);
  stroke(255, 50, 50); line(0, 0, 0, 0, 0, -110);  // X / Frente (Rojo)
  stroke(50, 255, 50); line(0, 0, 0, 110, 0, 0);   // Y / Derecha (Verde)
  stroke(50, 100, 255); line(0, 0, 0, 0, 60, 0);   // Z / Abajo (Azul)

  // 2. Modelo de la placa PCB principal (Ancho X = 190, Alto Y = 14, Largo Z = 130)
  fill(25, 45, 75);
  stroke(0, 220, 255);
  strokeWeight(1.5);
  box(190, 14, 130);

  // 3. Chip del sensor IMU en el centro
  fill(20, 20, 25);
  stroke(100, 140, 180);
  strokeWeight(1);
  pushMatrix();
  translate(0, -8, 0);
  box(36, 4, 36);
  popMatrix();

  // 4. Franja indicadora de rumbo frontal
  fill(255, 110, 0);
  noStroke();
  pushMatrix();
  translate(0, -8, -60);
  box(60, 4, 10);
  popMatrix();
}

void drawTelemetryPanel(float x, float y, float w, float h) {
  hint(DISABLE_DEPTH_TEST);

  fill(12, 18, 26);
  stroke(35, 60, 85);
  strokeWeight(1.5);
  rect(x, y, w, h, 8);

  fill(0, 220, 255);
  textSize(18);
  textAlign(LEFT, TOP);
  text("PANEL DE TIEMPO REAL DURO", x + 20, y + 18);

  // Seccion de Rendimiento y Ejecucion
  fill(25, 40, 58);
  noStroke();
  rect(x + 20, y + 55, w - 40, 130, 6);

  fill(255);
  textSize(13);
  text("Frecuencia de Lazo GNC: " + nf(loopFreqHz, 1, 1) + " Hz", x + 35, y + 70);
  text("Tiempo de Ejecucion WCET:" + nf(wcetUs, 1, 1) + " us (" + wcetCycles + " ciclos)", x + 35, y + 95);
  text("Paquetes RX / Errores:  " + rxPacketCount + " / " + checksumErrors, x + 35, y + 120);
  text("Perfil de Vuelo Activo: Perfil " + activeProfileId, x + 35, y + 145);

  // Seccion de Datos Inerciales
  fill(25, 40, 58);
  rect(x + 20, y + 200, w - 40, 140, 6);

  fill(255);
  text("Velocidades Giroscopo (dps):", x + 35, y + 215);
  fill(200, 220, 255);
  text("Wx: " + nf(gyroDpsX, 1, 2) + "   Wy: " + nf(gyroDpsY, 1, 2) + "   Wz: " + nf(gyroDpsZ, 1, 2), x + 50, y + 240);

  fill(255);
  text("Aceleraciones (g):", x + 35, y + 275);
  fill(200, 220, 255);
  text("Ax: " + nf(accelGX, 1, 3) + "   Ay: " + nf(accelGY, 1, 3) + "   Az: " + nf(accelGZ, 1, 3), x + 50, y + 300);

  // Matriz de Banderas de Salud del Sistema (FDIR)
  fill(25, 40, 58);
  rect(x + 20, y + 355, w - 40, 180, 6);

  fill(255);
  text("BANDERAS DE SALUD DEL SISTEMA (FDIR):", x + 35, y + 370);

  drawStatusLed("IMU_OK", (healthFlags & FLAG_IMU_OK) != 0, x + 40, y + 405);
  drawStatusLed("EKF_CONVERGIDO", (healthFlags & FLAG_EKF_CONVERGED) != 0, x + 240, y + 405);
  drawStatusLed("RESPALDO_TIMER", (healthFlags & FLAG_TIMER_FALLBACK_ACTIVE) != 0, x + 40, y + 445);
  drawStatusLed("RECHAZO_HIGH_G", (healthFlags & FLAG_HIGH_G_REJECTION) != 0, x + 240, y + 445);
  drawStatusLed("ANOMALIA_DETECT", (healthFlags & FLAG_ANOMALY_DETECTED) != 0, x + 40, y + 485);
  drawStatusLed("TRANSMITIENDO", (healthFlags & FLAG_TELEMETRY_STREAMING) != 0, x + 240, y + 485);

  // Boton de Reinicio / Retorno
  boolean hoverReset = (mouseX >= x + 20 && mouseX <= x + w - 20 && mouseY >= y + h - 50 && mouseY <= y + h - 15);
  fill(hoverReset ? color(200, 40, 40) : color(140, 30, 30));
  stroke(255, 80, 80);
  rect(x + 20, y + h - 50, w - 40, 35, 6);
  fill(255);
  textAlign(CENTER, CENTER);
  text(isDemoMode ? "VOLVER AL MENU DE PERFILES (REINICIAR SIM)" : "EJECUTAR REINICIO SOFTWARE (REBOOT ESP32)", x + w/2, y + h - 33);
}

void drawStatusLed(String label, boolean active, float x, float y) {
  fill(active ? color(0, 255, 120) : color(60, 75, 85));
  stroke(active ? color(0, 200, 100) : color(40, 50, 60));
  ellipse(x + 6, y + 6, 12, 12);

  fill(active ? color(220, 240, 255) : color(120, 140, 155));
  textAlign(LEFT, CENTER);
  textSize(12);
  text(label, x + 22, y + 6);
}

void drawHardFaultScreen() {
  hint(DISABLE_DEPTH_TEST);
  fill(255, 50, 50);
  textSize(24);
  textAlign(CENTER, TOP);
  text("BLOQUEO POR FALLO CRITICO (HARD FAULT)", width/2, 150);

  fill(220, 200, 200);
  textSize(14);
  text("El ESP32 ha detectado un fallo irrecuperable de integridad de vuelo.", width/2, 200);
  text("El sistema ha bloqueado actuadores y detenido el estimador para evitar divergencia.", width/2, 230);
  text("Por favor, reinicie la alimentacion de la placa o active reinicio hardware.", width/2, 280);
}

// -------------------------------------------------------------
// MOTOR DE SIMULACION DEMO (PRUEBAS FUERA DE LINEA)
// -------------------------------------------------------------

void startDemoMode(int profileId) {
  isDemoMode = true;
  activeProfileId = profileId;
  bistProgressPct = 0;
  currentUiState = UI_STATE_BIST_CALIBRATION;
  
  if (profileId == 1) loopFreqHz = 200.0f;
  else if (profileId == 2) loopFreqHz = 500.0f;
  else if (profileId == 3) loopFreqHz = 500.0f;
  else loopFreqHz = 1000.0f;
}

void updateDemoSimulation() {
  demoSimTime += 0.016f;

  if (currentUiState == UI_STATE_BIST_CALIBRATION) {
    bistProgressPct += 2;
    if (bistProgressPct >= 100) {
      bistProgressPct = 100;
      currentUiState = UI_STATE_RUNNING_ESTIMATOR;
    }
  } else if (currentUiState == UI_STATE_RUNNING_ESTIMATOR) {
    // Generar rotaciones continuas de Euler en 3D para verificacion visual
    rollDeg  = sin(demoSimTime * 1.2f) * 25.0f;
    pitchDeg = cos(demoSimTime * 0.9f) * 20.0f;
    yawDeg   = (demoSimTime * 15.0f) % 360.0f;

    // Convertir angulos de Euler a cuaternion normalizado [qw, qx, qy, qz]
    float r = radians(rollDeg) * 0.5f;
    float p = radians(pitchDeg) * 0.5f;
    float y = radians(yawDeg) * 0.5f;

    float cr = cos(r), sr = sin(r);
    float cp = cos(p), sp = sin(p);
    float cy = cos(y), sy = sin(y);

    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;

    // Tasas inerciales simuladas
    gyroDpsX = cos(demoSimTime * 1.2f) * 25.0f * 1.2f;
    gyroDpsY = -sin(demoSimTime * 0.9f) * 20.0f * 0.9f;
    gyroDpsZ = 15.0f;

    accelGX = sin(radians(pitchDeg));
    accelGY = -sin(radians(rollDeg));
    accelGZ = cos(radians(rollDeg)) * cos(radians(pitchDeg));

    wcetCycles = 14200 + (long)(sin(demoSimTime * 5.0f) * 400);
    wcetUs = wcetCycles / 240.0f;
    rxPacketCount++;
  }
}

// -------------------------------------------------------------
// PROTOCOLO Y COMUNICACION SERIE
// -------------------------------------------------------------

void mousePressed() {
  if (currentUiState == UI_STATE_PORT_SELECT) {
    String[] ports = Serial.list();
    float startY = 130;
    
    if (ports != null) {
      for (int i = 0; i < ports.length; i++) {
        float y = startY + i * 50;
        if (mouseX > width/2 - 160 && mouseX < width/2 + 160 && mouseY > y && mouseY < y + 38) {
          connectToSerial(ports[i]);
          return;
        }
      }
    }

    // Clic en el boton de modo demo
    float demoY = (ports != null && ports.length > 0) ? startY + ports.length * 50 + 30 : 220;
    if (mouseX > width/2 - 160 && mouseX < width/2 + 160 && mouseY > demoY && mouseY < demoY + 45) {
      startDemoMode(1);
    }
  } else if (currentUiState == UI_STATE_AWAITING_PROFILE) {
    if (mouseX >= 80 && mouseX <= 520 && mouseY >= 110 && mouseY <= 350) selectProfile(1);
    if (mouseX >= 580 && mouseX <= 1020 && mouseY >= 110 && mouseY <= 350) selectProfile(2);
    if (mouseX >= 80 && mouseX <= 520 && mouseY >= 380 && mouseY <= 620) selectProfile(3);
    if (mouseX >= 580 && mouseX <= 1020 && mouseY >= 380 && mouseY <= 620) selectProfile(4);
  } else if (currentUiState == UI_STATE_RUNNING_ESTIMATOR) {
    // Comprobar boton de reinicio
    if (mouseX >= 600 && mouseX <= 1040 && mouseY >= 630 && mouseY <= 665) {
      if (isDemoMode) {
        isDemoMode = false;
        currentUiState = UI_STATE_PORT_SELECT;
      } else {
        sendSystemResetCmd();
      }
    }
  }
}

void keyPressed() {
  if (key == 'd' || key == 'D') {
    startDemoMode(1);
    return;
  }

  if (currentUiState == UI_STATE_PORT_SELECT) {
    String[] ports = Serial.list();
    if (ports != null) {
      int idx = key - '1';
      if (idx >= 0 && idx < ports.length) {
        connectToSerial(ports[idx]);
      }
    }
  } else if (currentUiState == UI_STATE_AWAITING_PROFILE) {
    if (key == '1') selectProfile(1);
    if (key == '2') selectProfile(2);
    if (key == '3') selectProfile(3);
    if (key == '4') selectProfile(4);
  }
}

void selectProfile(int id) {
  if (serialPort != null) {
    sendSetProfileCmd(id);
  } else {
    // Modo demo fuera de linea
    startDemoMode(id);
  }
}

void connectToSerial(String portName) {
  try {
    if (serialPort != null) {
      serialPort.stop();
    }
    selectedPortName = portName;
    serialPort = new Serial(this, selectedPortName, baudRate);
    currentUiState = UI_STATE_AWAITING_PROFILE;
    println("Conectado a: " + portName);
  } catch (Exception e) {
    println("Error conectando al puerto: " + e.getMessage());
    currentUiState = UI_STATE_AWAITING_PROFILE;
  }
}

void sendSetProfileCmd(int profileId) {
  if (serialPort == null) return;

  byte[] pkt = new byte[7];
  pkt[0] = (byte)PREAMBLE_0;
  pkt[1] = (byte)PREAMBLE_1;
  pkt[2] = (byte)MSG_CMD_SET_PROFILE;
  pkt[3] = (byte)1; // Longitud = 1 byte
  pkt[4] = (byte)profileId;

  // Calcular Fletcher-16 sobre bytes 2, 3, 4 (msg_id, len, payload)
  int chk = calculateFletcher16(pkt, 2, 3);
  pkt[5] = (byte)(chk & 0xFF);
  pkt[6] = (byte)((chk >> 8) & 0xFF);

  serialPort.write(pkt);
  println("Enviado CMD_SET_PROFILE: " + profileId);
}

void sendSystemResetCmd() {
  if (serialPort == null) return;

  byte[] pkt = new byte[6];
  pkt[0] = (byte)PREAMBLE_0;
  pkt[1] = (byte)PREAMBLE_1;
  pkt[2] = (byte)MSG_CMD_SYSTEM_RESET;
  pkt[3] = (byte)0; // Longitud = 0 bytes

  int chk = calculateFletcher16(pkt, 2, 2);
  pkt[4] = (byte)(chk & 0xFF);
  pkt[5] = (byte)((chk >> 8) & 0xFF);

  serialPort.write(pkt);
  println("Enviado CMD_SYSTEM_RESET");
  currentUiState = UI_STATE_AWAITING_PROFILE;
}

int calculateFletcher16(byte[] data, int offset, int len) {
  int sum1 = 0;
  int sum2 = 0;
  for (int i = 0; i < len; i++) {
    int val = data[offset + i] & 0xFF;
    sum1 = (sum1 + val) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum2 << 8) | sum1;
}

void processSerial() {
  if (serialPort == null) return;

  while (serialPort.available() > 0) {
    int b = serialPort.read() & 0xFF;

    switch (rxState) {
      case 0: // PREAMBLE_0
        if (b == PREAMBLE_0) rxState = 1;
        break;

      case 1: // PREAMBLE_1
        if (b == PREAMBLE_1) {
          rxState = 2;
        } else if (b != PREAMBLE_0) {
          rxState = 0;
        }
        break;

      case 2: // MSG_ID
        rxMsgId = b;
        rxState = 3;
        break;

      case 3: // LENGTH
        rxLen = b;
        rxBufIdx = 0;
        if (rxLen > rxBuffer.length) {
          rxState = 0;
        } else if (rxLen == 0) {
          rxState = 5;
        } else {
          rxState = 4;
        }
        break;

      case 4: // PAYLOAD
        rxBuffer[rxBufIdx++] = (byte)b;
        if (rxBufIdx >= rxLen) rxState = 5;
        break;

      case 5: // CHK_LOW
        rxChkLow = b;
        rxState = 6;
        break;

      case 6: // CHK_HIGH
        int receivedChk = (b << 8) | rxChkLow;
        byte[] chkData = new byte[2 + rxLen];
        chkData[0] = (byte)rxMsgId;
        chkData[1] = (byte)rxLen;
        System.arraycopy(rxBuffer, 0, chkData, 2, rxLen);

        int calcChk = calculateFletcher16(chkData, 0, chkData.length);
        if (calcChk == receivedChk) {
          handleReceivedPacket(rxMsgId, rxBuffer, rxLen);
          rxPacketCount++;
        } else {
          checksumErrors++;
        }
        rxState = 0;
        break;
    }
  }
}

void handleReceivedPacket(int msgId, byte[] payload, int len) {
  lastPacketTimeMs = millis();
  isDemoMode = false; // La telemetria fisica real desactiva el modo demo

  if (msgId == MSG_HEARTBEAT_AWAIT_PROFILE && len >= 9) {
    espUptimeMs = readUint32LE(payload, 0);
    espSystemState = payload[4] & 0xFF;
    healthFlags = readInt32LE(payload, 5);

    if (currentUiState != UI_STATE_AWAITING_PROFILE && currentUiState != UI_STATE_PORT_SELECT) {
      currentUiState = UI_STATE_AWAITING_PROFILE;
    }
  } else if (msgId == MSG_BIST_REPORT && len >= 26) {
    bistCode = payload[0] & 0xFF;
    bistProgressPct = payload[1] & 0xFF;
    currentUiState = UI_STATE_BIST_CALIBRATION;
  } else if (msgId == MSG_ESTIMATOR_TELEMETRY && len >= 74) {
    q0 = readFloatLE(payload, 4);
    q1 = readFloatLE(payload, 8);  // IMU X (Alabeo / Roll)
    q2 = readFloatLE(payload, 12); // IMU Y (Cabeceo / Pitch)
    q3 = readFloatLE(payload, 16); // IMU Z (Guiñada / Yaw)

    rollDeg  = readFloatLE(payload, 20); // Roll (Alabeo lateral)
    pitchDeg = readFloatLE(payload, 24); // Pitch (Cabeceo morro)
    yawDeg   = readFloatLE(payload, 28); // Yaw (Guiñada / brújula)

    gyroDpsX = readFloatLE(payload, 32);
    gyroDpsY = readFloatLE(payload, 36);
    gyroDpsZ = readFloatLE(payload, 40);

    accelGX  = readFloatLE(payload, 44);
    accelGY  = readFloatLE(payload, 48);
    accelGZ  = readFloatLE(payload, 52);

    wcetCycles = readUint32LE(payload, 56);
    wcetUs     = readFloatLE(payload, 60);
    loopFreqHz = readFloatLE(payload, 64);
    healthFlags = readInt32LE(payload, 68);
    espSystemState = payload[72] & 0xFF;
    activeProfileId = payload[73] & 0xFF;

    currentUiState = UI_STATE_RUNNING_ESTIMATOR;
  } else if (msgId == MSG_ACK_NACK && len >= 2) {
    int refMsg = payload[0] & 0xFF;
    int status = payload[1] & 0xFF;
    println("Recibido ACK_NACK para Msg 0x" + hex(refMsg, 2) + " Estado: " + status);
    if (refMsg == MSG_CMD_SET_PROFILE && status == 0) {
      currentUiState = UI_STATE_BIST_CALIBRATION;
    }
  }
}

// -------------------------------------------------------------
// FUNCIONES AUXILIARES DE DESERIALIZACION BINARIA (LITTLE ENDIAN)
// -------------------------------------------------------------

long readUint32LE(byte[] b, int offset) {
  return ((b[offset] & 0xFFL)) |
         ((b[offset + 1] & 0xFFL) << 8) |
         ((b[offset + 2] & 0xFFL) << 16) |
         ((b[offset + 3] & 0xFFL) << 24);
}

int readInt32LE(byte[] b, int offset) {
  return ((b[offset] & 0xFF)) |
         ((b[offset + 1] & 0xFF) << 8) |
         ((b[offset + 2] & 0xFF) << 16) |
         ((b[offset + 3] & 0xFF) << 24);
}

float readFloatLE(byte[] b, int offset) {
  int intBits = readInt32LE(b, offset);
  return Float.intBitsToFloat(intBits);
}

void applyQuaternionRotation(float w, float x, float y, float z) {
  // Normalizar cuaternion para renderizado seguro
  float norm = sqrt(w*w + x*x + y*y + z*z);
  if (norm < 1e-6f) return;
  w /= norm; x /= norm; y /= norm; z /= norm;

  // Convertir cuaternion a matriz de rotacion 3D estandar
  float xx = x * x, yy = y * y, zz = z * z;
  float xy = x * y, xz = x * z, yz = y * z;
  float wx = w * x, wy = w * y, wz = w * z;

  applyMatrix(
    1.0f - 2.0f * (yy + zz),  2.0f * (xy - wz),        2.0f * (xz + wy),        0.0f,
    2.0f * (xy + wz),         1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),        0.0f,
    2.0f * (xz - wy),         2.0f * (yz + wx),        1.0f - 2.0f * (xx + yy), 0.0f,
    0.0f,                     0.0f,                    0.0f,                    1.0f
  );
}

