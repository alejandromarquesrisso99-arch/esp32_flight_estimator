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
float propAngle = 0.0f;

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
  
  // Actualizar rotacion continua de helices
  propAngle += 0.45f;
  if (propAngle > TWO_PI) propAngle -= TWO_PI;
  
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
  // IMU Y (Pitch) -> Processing X (-q2 para sentido de cabeceo correcto)
  // IMU Z (Yaw)   -> Processing Y (+q3 para sentido de guiñada/rumbo correcto)
  // IMU X (Roll)  -> Processing Z (-q1 para alabeo)
  pushMatrix();
  applyQuaternionRotation(q0, -q2, q3, -q1);

  // 3. Renderizado del modelo 3D del vehiculo segun perfil de vuelo
  drawVehicleModel();

  popMatrix();
  popMatrix();

  // Paso 3: Superposiciones HUD 2D
  hint(DISABLE_DEPTH_TEST);

  fill(0, 220, 255);
  textSize(16);
  textAlign(LEFT, TOP);
  String modelTitle = "ESTIMADOR DE ACTITUD 3D";
  if (activeProfileId == 1) modelTitle = "ESTIMADOR DE ACTITUD 3D - DRON CIVIL";
  else if (activeProfileId == 2) modelTitle = "ESTIMADOR DE ACTITUD 3D - DRON ACRO";
  else if (activeProfileId == 3) modelTitle = "ESTIMADOR DE ACTITUD 3D - COHETE";
  else if (activeProfileId == 4) modelTitle = "ESTIMADOR DE ACTITUD 3D - MISIL";
  text(modelTitle, x + 20, y + 18);

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

// -------------------------------------------------------------
// SELECTOR Y MODELOS 3D DE VEHICULO
// -------------------------------------------------------------

void drawVehicleModel() {
  switch (activeProfileId) {
    case 1:
      drawCivilianDroneModel();
      break;
    case 2:
      drawFpvRacingDroneModel();
      break;
    case 3:
      drawSpaceRocketModel();
      break;
    case 4:
      drawHighGManeuverMissileModel();
      break;
    default:
      drawCivilianDroneModel();
      break;
  }
}

/**
 * Modelo 3D: Dron Civil Minimalista para Grabacion Aerea (Perfil 1: DRONE_HOVER)
 * Diseno limpio, aerodinamico, estilo cuadricoptero de filmacion con gimbal 4K
 */
void drawCivilianDroneModel() {
  // 1. Ejes de coordenadas del cuerpo (referencia discreta de navegacion)
  strokeWeight(2);
  stroke(255, 60, 60, 160); line(0, 0, 0, 0, 0, -90);  // X cuerpo / Proa (-Z Processing)
  stroke(60, 255, 60, 160); line(0, 0, 0, 90, 0, 0);   // Y cuerpo / Estribor (+X Processing)
  stroke(60, 120, 255, 160); line(0, 0, 0, 0, 50, 0);  // Z cuerpo / Abajo (+Y Processing)

  // 2. Chasis / Fuselaje Central Aerodinamico
  // Base inferior del vientre
  fill(185, 192, 202);
  stroke(130, 140, 155);
  strokeWeight(1);
  pushMatrix();
  translate(0, 5, 0);
  box(32, 8, 76);
  popMatrix();

  // Modulo de posicionamiento optico inferior (Optical Flow + Sonar)
  fill(30, 35, 42);
  noStroke();
  pushMatrix();
  translate(0, 9.5f, 10);
  box(14, 3, 18);
  // Lentes del sensor de flujo optico
  fill(0, 195, 235);
  pushMatrix(); translate(-3.5f, 1.6f, 0); drawCylinder(1.8f, 1.8f, 0.5f, 12); popMatrix();
  pushMatrix(); translate(3.5f, 1.6f, 0); drawCylinder(1.8f, 1.8f, 0.5f, 12); popMatrix();
  popMatrix();

  // Cubierta superior aerodinamica (Blanco satinado / Pearl White)
  fill(238, 242, 246);
  stroke(160, 175, 190);
  strokeWeight(1);
  pushMatrix();
  translate(0, -3, 0);
  box(34, 10, 80);
  popMatrix();

  // Bahia superior de bateria y bahia GPS (Gris grafito mate)
  fill(42, 48, 56);
  stroke(30, 35, 42);
  pushMatrix();
  translate(0, -9, 5);
  box(24, 5, 50);
  popMatrix();

  // Indicadores LED de estado de bateria (4 micro-LEDs verdes en lomo)
  noStroke();
  fill(0, 255, 140);
  for (int i = 0; i < 4; i++) {
    pushMatrix();
    translate(0, -12, 16 - i * 7);
    box(4, 1.5f, 2.5f);
    popMatrix();
  }

  // Morro frontal conico
  fill(238, 242, 246);
  stroke(160, 175, 190);
  pushMatrix();
  translate(0, -1, -47);
  box(24, 9, 16);
  popMatrix();

  // Visor frontal con sensores estereoscopicos anticolision
  fill(18, 22, 28);
  noStroke();
  pushMatrix();
  translate(0, -1, -55.5f);
  box(20, 5, 2);
  // Sensores opticos estereo
  fill(0, 200, 255);
  pushMatrix(); translate(-6, 0, -1.2f); box(2.5f, 2.5f, 0.5f); popMatrix();
  pushMatrix(); translate(6, 0, -1.2f); box(2.5f, 2.5f, 0.5f); popMatrix();
  popMatrix();

  // 3. Camara de Grabacion Gimbal Estabilizada 4K (bajo el morro frontal)
  drawGimbalCamera(0, 8, -34);

  // 4. Cuatro Brazos Diagonales en configuracion 'X' aerodinamica
  // Delantero Izquierdo y Derecho
  drawSleekArm(-12, 0, -18, -75, -2, -55, 10, 7);
  drawSleekArm(12, 0, -18, 75, -2, -55, 10, 7);
  // Trasero Izquierdo y Derecho
  drawSleekArm(-12, 0, 18, -70, -2, 60, 10, 7);
  drawSleekArm(12, 0, 18, 70, -2, 60, 10, 7);

  // 5. Motores, Helices Giratorias y Luces de Navegacion Aerea
  // Delantero Izquierdo (FL) - CW (Giro horario), LED Rojo de babor
  drawPropellerAssembly(-75, -4, -55, propAngle, true);
  drawLandingGearAndNavLed(-75, 0, -55, color(255, 40, 40));

  // Delantero Derecho (FR) - CCW (Giro antihorario), LED Verde de estribor
  drawPropellerAssembly(75, -4, -55, propAngle, false);
  drawLandingGearAndNavLed(75, 0, -55, color(40, 255, 70));

  // Trasero Izquierdo (RL) - CCW, LED Ambar de cola
  drawPropellerAssembly(-70, -4, 60, propAngle, false);
  drawLandingGearAndNavLed(-70, 0, 60, color(255, 200, 40));

  // Trasero Derecho (RR) - CW, LED Ambar de cola
  drawPropellerAssembly(70, -4, 60, propAngle, true);
  drawLandingGearAndNavLed(70, 0, 60, color(255, 200, 40));
}

/**
 * Modelo 3D: Dron Acrobatico FPV Racer (Perfil 2: DRONE_ACRO)
 * Diseno agresivo de competicion: chasis True-X de fibra de carbono vista 3K, camara micro FPV
 * con 35° de inclinacion de carrera, antena VTX pagoda trasera, motores high-KV y helices tripala ahumadas.
 */
void drawFpvRacingDroneModel() {
  // 1. Ejes de referencia discretos de proa / estribor
  strokeWeight(2);
  stroke(255, 45, 55, 180); line(0, 0, 0, 0, 0, -85);  // Proa (-Z Processing)
  stroke(45, 255, 90, 180); line(0, 0, 0, 85, 0, 0);   // Estribor (+X Processing)
  stroke(45, 120, 255, 180); line(0, 0, 0, 0, 45, 0);  // Abajo (+Y Processing)

  // 2. Chasis Central de Competicion (True-X Compact Frame)
  // Placa inferior de fibra de carbono (Bottom Plate)
  fill(20, 23, 28);
  stroke(48, 56, 66);
  strokeWeight(1);
  pushMatrix();
  translate(0, 3, 0);
  box(24, 3, 66);
  popMatrix();

  // Separadores estructurales de aluminio anodizado (Standoffs de titanio)
  fill(60, 68, 78);
  noStroke();
  pushMatrix(); translate(-9, -2, -20); drawCylinder(2, 2, 7, 10); popMatrix();
  pushMatrix(); translate(9, -2, -20);  drawCylinder(2, 2, 7, 10); popMatrix();
  pushMatrix(); translate(-9, -2, 20);  drawCylinder(2, 2, 7, 10); popMatrix();
  pushMatrix(); translate(9, -2, 20);   drawCylinder(2, 2, 7, 10); popMatrix();

  // Placa superior de carbono (Top Plate)
  fill(24, 27, 32);
  stroke(52, 60, 70);
  strokeWeight(1);
  pushMatrix();
  translate(0, -6, 0);
  box(22, 2.5f, 60);
  popMatrix();

  // Canopy aerodinamico TPU en grafito mate stealth
  fill(32, 36, 44);
  stroke(45, 52, 62);
  pushMatrix();
  translate(0, -11, -8);
  box(18, 8, 32);
  popMatrix();

  // Bateria LiPo 6S de competicion montada en el lomo trasero
  fill(16, 18, 22);
  stroke(40, 46, 54);
  pushMatrix();
  translate(0, -12, 12);
  box(18, 9, 28);
  // Correa de velcro / kevlar de sujeccion
  fill(36, 40, 48);
  noStroke();
  translate(0, 0, 0);
  box(19, 10, 8);
  // Conector de potencia XT60 amarillo discreto atras
  fill(210, 160, 20);
  translate(0, 2, 16);
  box(6, 4, 4);
  popMatrix();

  // 3. Camara Micro FPV Agresiva con inclinacion de carrera (35° Pitch Up)
  pushMatrix();
  translate(0, -8, -28);
  
  // Placas laterales de carbono protectoras de la lente
  fill(20, 22, 26);
  stroke(42, 48, 56);
  strokeWeight(1);
  pushMatrix(); translate(-9, 0, 0); box(1.5f, 12, 16); popMatrix();
  pushMatrix(); translate(9, 0, 0);  box(1.5f, 12, 16); popMatrix();

  // Modulo de camara inclinado a 35 grados hacia arriba
  pushMatrix();
  rotateX(radians(-35)); // Inclinacion agresiva de carrera

  // Cuerpo de la camara FPV
  fill(26, 30, 36);
  stroke(50, 58, 68);
  box(14, 12, 14);

  // Bisel de la lente FPV
  fill(18, 20, 24);
  pushMatrix();
  translate(0, 0, -8);
  rotateX(HALF_PI);
  drawCylinder(4.5f, 4.5f, 4, 14);
  
  // Optica gran angular con reflejo carmin oscuro
  fill(190, 30, 45);
  noStroke();
  pushMatrix();
  translate(0, -2.1f, 0);
  drawCylinder(3.5f, 3.5f, 0.5f, 14);
  popMatrix();
  popMatrix();

  popMatrix();
  popMatrix();

  // 4. Antena VTX Pagoda Trasera (Inclinada 45° hacia atras)
  pushMatrix();
  translate(0, -10, 28);
  rotateX(radians(45));
  // Vastago
  fill(20, 22, 25);
  noStroke();
  drawCylinder(1.5f, 1.5f, 16, 8);
  // Cabeza Pagoda
  fill(15, 17, 20);
  pushMatrix();
  translate(0, -9, 0);
  drawCylinder(4.5f, 4.5f, 5, 14);
  popMatrix();
  popMatrix();

  // 5. Cuatro Brazos de Carbono en True-X
  drawCarbonArm(-10, 2, -14, -62, 0, -45, 8, 6);
  drawCarbonArm(10, 2, -14, 62, 0, -45, 8, 6);
  drawCarbonArm(-10, 2, 14, -62, 0, 45, 8, 6);
  drawCarbonArm(10, 2, 14, 62, 0, 45, 8, 6);

  // 6. Tiras LED de Competicion (Bajo los brazos)
  // Delanteras: Cian discreto
  fill(0, 190, 220);
  noStroke();
  pushMatrix(); translate(-36, 4, -30); box(12, 1, 3); popMatrix();
  pushMatrix(); translate(36, 4, -30);  box(12, 1, 3); popMatrix();
  // Traseras: Rojo carmin de carrera
  fill(220, 30, 40);
  pushMatrix(); translate(-36, 4, 30); box(12, 1, 3); popMatrix();
  pushMatrix(); translate(36, 4, 30);  box(12, 1, 3); popMatrix();

  // 7. Motores Brushless Race y Helices Tripala de Alta Velocidad
  // Delantero Izquierdo (FL) - CW
  drawTriBladePropeller(-62, -2, -45, propAngle, true);
  // Delantero Derecho (FR) - CCW
  drawTriBladePropeller(62, -2, -45, propAngle, false);
  // Trasero Izquierdo (RL) - CCW
  drawTriBladePropeller(-62, -2, 45, propAngle, false);
  // Trasero Derecho (RR) - CW
  drawTriBladePropeller(62, -2, 45, propAngle, true);
}

/**
 * Modelo 3D: Cohete Espacial Multietapa (Perfil 3: ROCKET_LAUNCH)
 * Inspirado en vehiculos de lanzamiento orbital aeroespacial modernos (estilo NASA / SpaceX)
 * Geometria pura y estilizada: cofia ojival de proa, anillo interetapa de carbono, 4 aletas de rejilla (Grid Fins) de titanio,
 * conductos longitudinales de avionica (raceways), toberas de alta temperatura y penacho de propulsion con diamantes de choque supersonicos.
 */
void drawSpaceRocketModel() {
  // 1. Ejes de referencia discretos de proa / estribor / vientre
  strokeWeight(2);
  stroke(255, 45, 55, 180); line(0, 0, 0, 0, 0, -145); // Eje longitudinal / Proa (-Z Processing)
  stroke(45, 255, 90, 180); line(0, 0, 0, 60, 0, 0);    // Eje transversal / Estribor (+X Processing)
  stroke(45, 120, 255, 180); line(0, 0, 0, 0, 60, 0);   // Eje normal / Vientre (+Y Processing)

  // 2. Cofia Ojival de Carga Util (Payload Fairing en proa: Z = -135 a Z = -75)
  // Punta conica ojival de proa
  fill(242, 245, 250);
  stroke(175, 185, 200);
  strokeWeight(1);
  pushMatrix();
  translate(0, 0, -115);
  rotateX(HALF_PI);
  drawCylinder(0.8f, 13.5f, 40, 24);
  popMatrix();

  // Cuerpo cilindrico de la cofia
  fill(245, 248, 252);
  pushMatrix();
  translate(0, 0, -85);
  rotateX(HALF_PI);
  drawCylinder(13.5f, 13.5f, 20, 24);
  popMatrix();

  // Franja de separacion de cofia (Linea pirotecnica de desacople)
  fill(40, 45, 52);
  noStroke();
  pushMatrix();
  translate(0, 0, -75);
  rotateX(HALF_PI);
  drawCylinder(13.7f, 13.7f, 1.5f, 24);
  popMatrix();

  // 3. Segunda Etapa (Upper Stage: Z = -75 a Z = -40)
  fill(238, 242, 248);
  stroke(175, 185, 200);
  strokeWeight(1);
  pushMatrix();
  translate(0, 0, -57.5f);
  rotateX(HALF_PI);
  drawCylinder(13.0f, 13.0f, 35, 24);
  popMatrix();

  // 4. Anillo Interetapa de Carbono (Interstage: Z = -40 a Z = -20)
  fill(26, 29, 35);
  stroke(50, 58, 68);
  strokeWeight(1);
  pushMatrix();
  translate(0, 0, -30);
  rotateX(HALF_PI);
  drawCylinder(13.2f, 13.2f, 20, 24);
  popMatrix();

  // 4 Aletas de Guiado Triangulares (Delta Canards) en el anillo interetapa
  drawTriangularAeroFin(0, 13.2f, -30, 16.0f, 13.0f, 12.0f, 1.4f, color(45, 52, 62), color(70, 80, 95));
  drawTriangularAeroFin(HALF_PI, 13.2f, -30, 16.0f, 13.0f, 12.0f, 1.4f, color(45, 52, 62), color(70, 80, 95));
  drawTriangularAeroFin(PI, 13.2f, -30, 16.0f, 13.0f, 12.0f, 1.4f, color(45, 52, 62), color(70, 80, 95));
  drawTriangularAeroFin(PI + HALF_PI, 13.2f, -30, 16.0f, 13.0f, 12.0f, 1.4f, color(45, 52, 62), color(70, 80, 95));

  // 5. Primera Etapa / Booster Principal (Z = -20 a Z = +70)
  fill(240, 244, 250);
  stroke(180, 190, 205);
  strokeWeight(1);
  pushMatrix();
  translate(0, 0, 25);
  rotateX(HALF_PI);
  drawCylinder(13.0f, 13.0f, 90, 24);
  popMatrix();

  // Conductos longitudinales de avionica y alimentacion (Raceways)
  fill(36, 42, 50);
  noStroke();
  pushMatrix();
  translate(0, -13.2f, 15);
  box(2.2f, 1.2f, 110);
  popMatrix();
  pushMatrix();
  translate(0, 13.2f, 15);
  box(2.2f, 1.2f, 110);
  popMatrix();

  // Bandas estructurales de tanques criogenicos (LOX y Kerosene/CH4)
  fill(160, 172, 185);
  pushMatrix(); translate(0, 0, -5); rotateX(HALF_PI); drawCylinder(13.3f, 13.3f, 1.2f, 24); popMatrix();
  pushMatrix(); translate(0, 0, 45); rotateX(HALF_PI); drawCylinder(13.3f, 13.3f, 1.2f, 24); popMatrix();

  // 4 Grandes Aletas Estabilizadoras Delta Triangulares en la base
  drawTriangularAeroFin(0, 13.0f, 52, 28.0f, 18.0f, 20.0f, 2.2f, color(240, 244, 250), color(175, 185, 200));
  drawTriangularAeroFin(HALF_PI, 13.0f, 52, 28.0f, 18.0f, 20.0f, 2.2f, color(240, 244, 250), color(175, 185, 200));
  drawTriangularAeroFin(PI, 13.0f, 52, 28.0f, 18.0f, 20.0f, 2.2f, color(240, 244, 250), color(175, 185, 200));
  drawTriangularAeroFin(PI + HALF_PI, 13.0f, 52, 28.0f, 18.0f, 20.0f, 2.2f, color(240, 244, 250), color(175, 185, 200));

  // 6. Base de Empuje y Racimo de Toberas (Thrust Structure & Nozzles: Z = +70 a Z = +85)
  // Escudo termico de popa
  fill(28, 30, 34);
  stroke(15, 18, 20);
  pushMatrix();
  translate(0, 0, 70);
  rotateX(HALF_PI);
  drawCylinder(13.0f, 11.5f, 4, 24);
  popMatrix();

  // Racimo de Toberas de Motor Cohete (Aleacion Inconel/Niobio)
  // Tobera Central Principal
  drawRocketNozzle(0, 0, 72, 3.0f, 5.5f, 14);

  // 4 Toberas Perifericas
  float nozOff = 5.6f;
  drawRocketNozzle(-nozOff, 0, 72, 2.2f, 4.2f, 12);
  drawRocketNozzle(nozOff, 0, 72, 2.2f, 4.2f, 12);
  drawRocketNozzle(0, -nozOff, 72, 2.2f, 4.2f, 12);
  drawRocketNozzle(0, nozOff, 72, 2.2f, 4.2f, 12);

  // 7. Penacho de Propulsion Cohete Animado (Exhaust Plume con Diamantes de Choque)
  drawRocketExhaustPlume(0, 0, 86);
}

// -------------------------------------------------------------
// SUBCOMPONENTES DEL COHETE ESPACIAL
// -------------------------------------------------------------

void drawTriangularAeroFin(float angle, float radius, float z, float rootLen, float span, float sweepZ, float thick, color cFill, color cStroke) {
  pushMatrix();
  translate(0, 0, z);
  rotateZ(angle);
  translate(0, -radius, 0);

  fill(cFill);
  stroke(cStroke);
  strokeWeight(1);

  float halfThick = thick * 0.5f;
  float frontZ = -rootLen * 0.5f;
  float rearZ  = rootLen * 0.5f;
  float tipY   = -span;
  float tipZ   = frontZ + sweepZ;

  // Cara lateral izquierda (+X)
  beginShape();
  vertex(halfThick, 0, frontZ);
  vertex(halfThick, tipY, tipZ);
  vertex(halfThick, 0, rearZ);
  endShape(CLOSE);

  // Cara lateral derecha (-X)
  beginShape();
  vertex(-halfThick, 0, frontZ);
  vertex(-halfThick, 0, rearZ);
  vertex(-halfThick, tipY, tipZ);
  endShape(CLOSE);

  // Borde de ataque (Leading Edge con perfil afilado)
  beginShape();
  vertex(-halfThick, 0, frontZ);
  vertex(halfThick, 0, frontZ);
  vertex(halfThick, tipY, tipZ);
  vertex(-halfThick, tipY, tipZ);
  endShape(CLOSE);

  // Borde de fuga (Trailing Edge)
  beginShape();
  vertex(halfThick, tipY, tipZ);
  vertex(halfThick, 0, rearZ);
  vertex(-halfThick, 0, rearZ);
  vertex(-halfThick, tipY, tipZ);
  endShape(CLOSE);

  // Borde de encastre (Root)
  beginShape();
  vertex(-halfThick, 0, frontZ);
  vertex(-halfThick, 0, rearZ);
  vertex(halfThick, 0, rearZ);
  vertex(halfThick, 0, frontZ);
  endShape(CLOSE);

  popMatrix();
}

void drawRocketNozzle(float x, float y, float z, float rTop, float rBottom, float len) {
  pushMatrix();
  translate(x, y, z + len * 0.5f);
  rotateX(HALF_PI);

  // Exterior de la tobera de niobio/inconel
  fill(65, 60, 56);
  stroke(35, 30, 28);
  strokeWeight(0.8f);
  drawCylinder(rTop, rBottom, len, 16);

  // Interior al rojo vivo de la camara de combustion
  fill(255, 120, 30);
  noStroke();
  pushMatrix();
  translate(0, -len * 0.2f, 0);
  drawCylinder(rTop * 0.8f, rBottom * 0.6f, len * 0.5f, 16);
  popMatrix();

  popMatrix();
}

void drawRocketExhaustPlume(float x, float y, float z) {
  float flicker = sin(frameCount * 0.5f) * 4.0f + random(-2.0f, 2.0f);
  float plumeLen = 72.0f + flicker;

  pushMatrix();
  translate(x, y, z);

  // Capa exterior de llama energetica naranja/ambar translucida
  noStroke();
  fill(255, 120, 20, 140);
  pushMatrix();
  translate(0, 0, plumeLen * 0.45f);
  rotateX(HALF_PI);
  drawCylinder(8.5f, 1.0f, plumeLen, 16);
  popMatrix();

  // Nucleo interior hipersonico blanco brillante / amarillo
  fill(255, 245, 210, 220);
  pushMatrix();
  translate(0, 0, plumeLen * 0.25f);
  rotateX(HALF_PI);
  drawCylinder(5.0f, 0.5f, plumeLen * 0.55f, 14);
  popMatrix();

  // Diamantes de choque supersonicos (Shock Diamonds)
  fill(0, 230, 255, 210);
  for (int d = 1; d <= 3; d++) {
    pushMatrix();
    translate(0, 0, d * 14.0f + flicker * 0.2f);
    rotateX(HALF_PI);
    drawCylinder(3.2f - d * 0.6f, 0.4f, 7.0f, 10);
    popMatrix();
  }

  popMatrix();
}

/**
 * Modelo 3D: Misil Aire-Aire Tactico de Alta Maniobra (Perfil 4: MISSILE_HIGH_G)
 * Estilo misil aire-aire supersonico interceptor (AIM-120 AMRAAM / Meteor / AIM-9X / IRIS-T)
 * Fuselaje esbelto y alargado, colores oscuros militares discretos (gris grafito tactico RAM),
 * aletas triangulares en cruz cruciforme delante (canards) y detras (cola), cupula optica de punteria
 * y propulsion supersonica con penacho de llama azul electrico y diamantes de choque.
 */
void drawHighGManeuverMissileModel() {
  // 1. Ejes de referencia discretos de trayectoria / proa / estribor
  strokeWeight(2);
  stroke(255, 45, 55, 180); line(0, 0, 0, 0, 0, -145); // Eje longitudinal de trayectoria (-Z Processing)
  stroke(45, 255, 90, 180); line(0, 0, 0, 45, 0, 0);    // Eje transversal (+X Processing)
  stroke(45, 120, 255, 180); line(0, 0, 0, 0, 45, 0);   // Eje vertical (+Y Processing)

  // 2. Cupula Frontal y Autodirector Aire-Aire (Seeker Nose: Z = -135 a Z = -100)
  // Sensor infrarrojo / optico frontal
  fill(16, 20, 25);
  stroke(45, 52, 62);
  strokeWeight(1);
  pushMatrix();
  translate(0, 0, -125);
  rotateX(HALF_PI);
  drawCylinder(0.8f, 3.2f, 18, 20);
  popMatrix();

  // Lente de zafiro frontal con destello discreto
  fill(160, 200, 240, 220);
  noStroke();
  pushMatrix();
  translate(0, 0, -133.5f);
  sphere(1.0f);
  popMatrix();

  // Ojiva aerodinamica frontal de baja resistencia (Radomo ceramico grafito)
  fill(28, 32, 38);
  stroke(50, 58, 68);
  strokeWeight(1);
  pushMatrix();
  translate(0, 0, -108);
  rotateX(HALF_PI);
  drawCylinder(3.2f, 6.0f, 16, 22);
  popMatrix();

  // 3. Seccion Delantera de Guiado y Aletas Delanteras en Cruz (Canards: Z = -100 a Z = -65)
  fill(36, 40, 48);
  stroke(55, 62, 72);
  pushMatrix();
  translate(0, 0, -85);
  rotateX(HALF_PI);
  drawCylinder(6.0f, 6.0f, 30, 22);
  popMatrix();

  // Banda tactica amarilla de ojiva de combate (High Explosive)
  fill(175, 140, 25);
  noStroke();
  pushMatrix();
  translate(0, 0, -96);
  rotateX(HALF_PI);
  drawCylinder(6.15f, 6.15f, 1.5f, 22);
  popMatrix();

  // 4 ALETAS DELANTERAS EN CRUZ CRUCIFORME (Canards de Control de Guiado Rapido en Z = -85)
  drawTriangularAeroFin(0, 6.0f, -85, 16.0f, 12.0f, 10.0f, 1.2f, color(24, 28, 34), color(65, 75, 90));
  drawTriangularAeroFin(HALF_PI, 6.0f, -85, 16.0f, 12.0f, 10.0f, 1.2f, color(24, 28, 34), color(65, 75, 90));
  drawTriangularAeroFin(PI, 6.0f, -85, 16.0f, 12.0f, 10.0f, 1.2f, color(24, 28, 34), color(65, 75, 90));
  drawTriangularAeroFin(PI + HALF_PI, 6.0f, -85, 16.0f, 12.0f, 10.0f, 1.2f, color(24, 28, 34), color(65, 75, 90));

  // 4. Fuselaje Alargado Principal / Motor Cohete Solido (Solid Rocket Motor: Z = -65 a Z = +55)
  fill(32, 36, 42);
  stroke(50, 58, 68);
  strokeWeight(1);
  pushMatrix();
  translate(0, 0, -5);
  rotateX(HALF_PI);
  drawCylinder(6.0f, 6.0f, 120, 22);
  popMatrix();

  // Banda tactica marron de motor cohete (Rocket Motor)
  fill(125, 68, 25);
  noStroke();
  pushMatrix();
  translate(0, 0, -58);
  rotateX(HALF_PI);
  drawCylinder(6.15f, 6.15f, 1.5f, 22);
  popMatrix();

  // Conductos longitudinales de cableado tactico dorsales y ventrales
  fill(22, 26, 32);
  pushMatrix(); translate(0, -6.15f, -5); box(1.3f, 0.6f, 115); popMatrix();
  pushMatrix(); translate(0, 6.15f, -5);  box(1.3f, 0.6f, 115); popMatrix();

  // 5. Seccion de Cola y 4 ALETAS TRASERAS EN CRUZ (Aft Tail Fins: Z = +55 a Z = +78)
  fill(26, 30, 36);
  stroke(46, 54, 64);
  pushMatrix();
  translate(0, 0, 66);
  rotateX(HALF_PI);
  drawCylinder(6.0f, 5.5f, 22, 22);
  popMatrix();

  // 4 ALETAS TRASERAS EN CRUZ CRUCIFORME (Aletas de Estabilizacion y Maniobra en Z = +65)
  drawTriangularAeroFin(0, 5.8f, 65, 26.0f, 16.0f, 18.0f, 1.4f, color(22, 25, 30), color(70, 80, 95));
  drawTriangularAeroFin(HALF_PI, 5.8f, 65, 26.0f, 16.0f, 18.0f, 1.4f, color(22, 25, 30), color(70, 80, 95));
  drawTriangularAeroFin(PI, 5.8f, 65, 26.0f, 16.0f, 18.0f, 1.4f, color(22, 25, 30), color(70, 80, 95));
  drawTriangularAeroFin(PI + HALF_PI, 5.8f, 65, 26.0f, 16.0f, 18.0f, 1.4f, color(22, 25, 30), color(70, 80, 95));

  // 6. Tobera de Propulsion de Popa (Z = +77 a Z = +85)
  pushMatrix();
  translate(0, 0, 81);
  rotateX(HALF_PI);
  fill(38, 36, 35);
  stroke(22, 20, 18);
  strokeWeight(0.8f);
  drawCylinder(3.6f, 4.8f, 7.0f, 16);

  // Nucleo incandescente de plasma de la tobera
  fill(0, 200, 255);
  noStroke();
  pushMatrix();
  translate(0, -1.8f, 0);
  drawCylinder(3.0f, 2.2f, 2.5f, 14);
  popMatrix();
  popMatrix();

  // 7. Llama y Penacho de Propulsion Azul Electrico Supersonico
  drawMissileExhaustPlume(0, 0, 85);
}

void drawMissileExhaustPlume(float x, float y, float z) {
  float flicker = sin(frameCount * 0.65f) * 3.5f + random(-1.5f, 1.5f);
  float plumeLen = 60.0f + flicker;

  pushMatrix();
  translate(x, y, z);

  // Llama exterior conica azul electrico / cian translucida
  noStroke();
  fill(0, 140, 255, 160);
  pushMatrix();
  translate(0, 0, plumeLen * 0.45f);
  rotateX(HALF_PI);
  drawCylinder(4.2f, 0.4f, plumeLen, 14);
  popMatrix();

  // Nucleo hipersonico blanco-azulado de alta temperatura
  fill(215, 242, 255, 230);
  pushMatrix();
  translate(0, 0, plumeLen * 0.25f);
  rotateX(HALF_PI);
  drawCylinder(2.4f, 0.3f, plumeLen * 0.5f, 12);
  popMatrix();

  // Diamantes de choque supersonicos en cian brillante
  fill(0, 255, 240, 230);
  for (int d = 1; d <= 3; d++) {
    pushMatrix();
    translate(0, 0, d * 11.0f + flicker * 0.2f);
    rotateX(HALF_PI);
    drawCylinder(1.8f - d * 0.4f, 0.2f, 5.0f, 8);
    popMatrix();
  }

  popMatrix();
}

// -------------------------------------------------------------
// SUBCOMPONENTES DEL MODELO 3D
// -------------------------------------------------------------

void drawCarbonArm(float x1, float y1, float z1, float x2, float y2, float z2, float w, float h) {
  pushMatrix();
  float mx = (x1 + x2) * 0.5f;
  float my = (y1 + y2) * 0.5f;
  float mz = (z1 + z2) * 0.5f;
  translate(mx, my, mz);

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dz = z2 - z1;
  float len = sqrt(dx*dx + dy*dy + dz*dz);

  float yaw = atan2(dx, dz);
  float pitch = -atan2(dy, sqrt(dx*dx + dz*dz));

  rotateY(yaw);
  rotateX(pitch);

  // Brazo de carbono 3K mate
  fill(22, 25, 30);
  stroke(48, 54, 62);
  strokeWeight(1);
  box(w, h, len);

  // Chaflan superior
  fill(38, 44, 52);
  noStroke();
  pushMatrix();
  translate(0, -h * 0.4f, 0);
  box(w * 0.7f, 1.0f, len * 0.95f);
  popMatrix();

  popMatrix();
}

void drawTriBladePropeller(float x, float y, float z, float angle, boolean isCW) {
  pushMatrix();
  translate(x, y, z);

  // Carcasa del motor Brushless Race
  fill(36, 40, 48);
  stroke(20, 24, 30);
  strokeWeight(1);
  drawCylinder(7.5f, 7.5f, 9, 16);

  // Campana rotorica en titanio oscuro
  fill(65, 72, 82);
  pushMatrix();
  translate(0, -4.5f, 0);
  drawCylinder(8, 8, 4, 16);
  popMatrix();

  // Helice Tripala en giro
  pushMatrix();
  translate(0, -7.5f, 0);
  rotateY(isCW ? angle * 1.5f : -angle * 1.5f);

  // Buje central
  fill(25, 28, 34);
  noStroke();
  drawCylinder(3.2f, 3.2f, 3.5f, 12);

  // 3 Palas espaciadas a 120 grados
  for (int b = 0; b < 3; b++) {
    pushMatrix();
    rotateY(b * TWO_PI / 3.0f);

    // Cuerpo de la pala ahumada
    fill(24, 28, 34, 225);
    stroke(15, 18, 22);
    strokeWeight(0.8f);
    pushMatrix();
    translate(0, 0, 16);
    rotateX(radians(9));
    box(5.5f, 1.2f, 22);
    popMatrix();

    // Punta en carmin oscuro discreto
    fill(190, 28, 38);
    noStroke();
    pushMatrix();
    translate(0, 0, 30);
    rotateX(radians(9));
    box(4.8f, 1.1f, 7);
    popMatrix();

    popMatrix();
  }
  popMatrix();

  // Halo translucido de disco de rotor en alta velocidad
  noStroke();
  fill(30, 40, 55, 28);
  pushMatrix();
  translate(0, -7.5f, 0);
  drawCylinder(34, 34, 1, 24);
  popMatrix();

  popMatrix();
}

void drawSleekArm(float x1, float y1, float z1, float x2, float y2, float z2, float w, float h) {
  pushMatrix();
  float mx = (x1 + x2) * 0.5f;
  float my = (y1 + y2) * 0.5f;
  float mz = (z1 + z2) * 0.5f;
  translate(mx, my, mz);

  float dx = x2 - x1;
  float dy = y2 - y1;
  float dz = z2 - z1;
  float len = sqrt(dx*dx + dy*dy + dz*dz);

  float yaw = atan2(dx, dz);
  float pitch = -atan2(dy, sqrt(dx*dx + dz*dz));

  rotateY(yaw);
  rotateX(pitch);

  // Cubierta superior blanca del brazo
  fill(230, 235, 240);
  stroke(140, 155, 170);
  strokeWeight(1);
  box(w, h * 0.65f, len);

  // Insercion inferior de refuerzo de carbono
  fill(38, 44, 52);
  noStroke();
  pushMatrix();
  translate(0, h * 0.35f, 0);
  box(w * 0.8f, h * 0.4f, len * 0.95f);
  popMatrix();

  popMatrix();
}

void drawPropellerAssembly(float x, float y, float z, float angle, boolean isCW) {
  pushMatrix();
  translate(x, y, z);

  // Carcasa del estator del motor sin escobillas (Brushless)
  fill(55, 60, 68);
  stroke(30, 35, 40);
  strokeWeight(1);
  drawCylinder(8, 8, 10, 16);

  // Campana rotorica de aluminio superior
  fill(85, 92, 102);
  pushMatrix();
  translate(0, -5, 0);
  drawCylinder(8.5f, 8.5f, 4, 16);
  popMatrix();

  // Helice giratoria con palas aerodinamicas plegables
  pushMatrix();
  translate(0, -8, 0);
  rotateY(isCW ? angle : -angle);

  // Tapa central del buje
  fill(30, 34, 40);
  noStroke();
  drawCylinder(3.5f, 3.5f, 4, 12);

  // 2 Palas aerodinamicas opuestas
  for (int b = 0; b < 2; b++) {
    pushMatrix();
    rotateY(b * PI);

    // Cuerpo de la pala de carbono
    fill(35, 40, 48, 230);
    stroke(20, 24, 30);
    strokeWeight(0.8f);
    pushMatrix();
    translate(0, 0, 18);
    rotateX(radians(7)); // Angulo de ataque aerodinamico
    box(7, 1.2f, 26);
    popMatrix();

    // Puntas de pala de alta visibilidad naranja
    fill(255, 140, 20);
    noStroke();
    pushMatrix();
    translate(0, 0, 34);
    rotateX(radians(7));
    box(6, 1.2f, 8);
    popMatrix();

    popMatrix();
  }
  popMatrix();

  // Halo translucido de disco de rotor en giro (efecto desenfoque de movimiento)
  noStroke();
  fill(0, 210, 255, 20);
  pushMatrix();
  translate(0, -8, 0);
  drawCylinder(38, 38, 1, 24);
  popMatrix();

  popMatrix();
}

void drawLandingGearAndNavLed(float x, float y, float z, int ledColor) {
  pushMatrix();
  translate(x, y, z);

  // Pata de aterrizaje y amortiguador
  fill(40, 45, 52);
  stroke(25, 30, 36);
  strokeWeight(1);
  pushMatrix();
  translate(0, 10, 0);
  box(6, 14, 7);
  popMatrix();

  // Diodo LED de navegacion y emisor
  noStroke();
  fill(ledColor);
  pushMatrix();
  translate(0, 17, 0);
  box(4, 2, 4);

  // Halo difuso de luz
  fill(ledColor, 60);
  drawCylinder(6, 6, 2, 12);
  popMatrix();

  popMatrix();
}

void drawGimbalCamera(float x, float y, float z) {
  pushMatrix();
  translate(x, y, z);

  // Base del gimbal bajo el morro
  fill(45, 50, 58);
  stroke(25, 30, 36);
  strokeWeight(1);
  drawCylinder(5, 5, 4, 12);

  // Horquilla / brazo de balanceo del gimbal
  pushMatrix();
  translate(0, 5, 0);
  fill(40, 45, 52);
  box(18, 4, 8);
  popMatrix();

  // Cuerpo de camara con inclinacion cinematografica (-12 grados hacia abajo)
  pushMatrix();
  translate(0, 11, -2);
  rotateX(radians(12));

  fill(28, 32, 38);
  stroke(50, 58, 68);
  strokeWeight(1);
  box(16, 13, 18);

  // Bisel exterior de la lente de grabacion
  fill(20, 22, 26);
  pushMatrix();
  translate(0, 0, -10);
  rotateX(HALF_PI);
  drawCylinder(5.5f, 5.5f, 4, 16);

  // Elemento de cristal optico con reflejo cian
  fill(0, 195, 235);
  noStroke();
  pushMatrix();
  translate(0, -2.1f, 0);
  drawCylinder(4.2f, 4.2f, 0.5f, 16);
  popMatrix();
  popMatrix();

  // Micro-LED de grabacion en curso (Tally light rojo)
  fill(255, 30, 30);
  noStroke();
  pushMatrix();
  translate(5.5f, -4, -9.5f);
  box(1.5f, 1.5f, 1.5f);
  popMatrix();

  popMatrix();

  popMatrix();
}

void drawCylinder(float rTop, float rBottom, float h, int sides) {
  float angleStep = TWO_PI / sides;

  // Superficie lateral cilindrica
  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    float a = i * angleStep;
    float ca = cos(a);
    float sa = sin(a);
    vertex(ca * rTop, -h * 0.5f, sa * rTop);
    vertex(ca * rBottom, h * 0.5f, sa * rBottom);
  }
  endShape();

  // Tapa superior
  if (rTop > 0) {
    beginShape(TRIANGLE_FAN);
    vertex(0, -h * 0.5f, 0);
    for (int i = 0; i <= sides; i++) {
      float a = i * angleStep;
      vertex(cos(a) * rTop, -h * 0.5f, sin(a) * rTop);
    }
    endShape();
  }

  // Tapa inferior
  if (rBottom > 0) {
    beginShape(TRIANGLE_FAN);
    vertex(0, h * 0.5f, 0);
    for (int i = 0; i <= sides; i++) {
      float a = i * angleStep;
      vertex(cos(a) * rBottom, h * 0.5f, sin(a) * rBottom);
    }
    endShape();
  }
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
    // Comprobar clic en el boton de reinicio (X: 620-1070, Y: 655-690)
    if (mouseX >= 620 && mouseX <= 1070 && mouseY >= 655 && mouseY <= 690) {
      if (isDemoMode) {
        isDemoMode = false;
        currentUiState = UI_STATE_AWAITING_PROFILE;
      } else {
        sendSystemResetCmd();
      }
    }
  }
}

void keyPressed() {
  // Tecla 'R' para reinicio rapido y retorno al menu de perfiles
  if (key == 'r' || key == 'R') {
    if (isDemoMode) {
      isDemoMode = false;
      currentUiState = UI_STATE_AWAITING_PROFILE;
    } else {
      sendSystemResetCmd();
    }
    return;
  }

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
  if (serialPort != null) {
    byte[] pkt = new byte[6];
    pkt[0] = (byte)PREAMBLE_0;
    pkt[1] = (byte)PREAMBLE_1;
    pkt[2] = (byte)MSG_CMD_SYSTEM_RESET;
    pkt[3] = (byte)0; // Longitud = 0 bytes

    int chk = calculateFletcher16(pkt, 2, 2);
    pkt[4] = (byte)(chk & 0xFF);
    pkt[5] = (byte)((chk >> 8) & 0xFF);

    serialPort.write(pkt);
    println("Enviado CMD_SYSTEM_RESET al ESP32");
  }
  isDemoMode = false;
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
    float inQ0 = readFloatLE(payload, 4);
    float inQ1 = readFloatLE(payload, 8);  // IMU X (Alabeo / Roll)
    float inQ2 = readFloatLE(payload, 12); // IMU Y (Cabeceo / Pitch)
    float inQ3 = readFloatLE(payload, 16); // IMU Z (Guiñada / Yaw)

    if (!Float.isNaN(inQ0) && !Float.isNaN(inQ1) && !Float.isNaN(inQ2) && !Float.isNaN(inQ3) &&
        !Float.isInfinite(inQ0) && !Float.isInfinite(inQ1) && !Float.isInfinite(inQ2) && !Float.isInfinite(inQ3)) {
      q0 = inQ0; q1 = inQ1; q2 = inQ2; q3 = inQ3;
    }

    float inR = readFloatLE(payload, 20); // Roll (Alabeo lateral)
    float inP = readFloatLE(payload, 24); // Pitch (Cabeceo morro)
    float inY = readFloatLE(payload, 28); // Yaw (Guiñada / brújula)

    if (!Float.isNaN(inR) && !Float.isNaN(inP) && !Float.isNaN(inY) &&
        !Float.isInfinite(inR) && !Float.isInfinite(inP) && !Float.isInfinite(inY)) {
      rollDeg = inR; pitchDeg = inP; yawDeg = inY;
    }

    float inGx = readFloatLE(payload, 32);
    float inGy = readFloatLE(payload, 36);
    float inGz = readFloatLE(payload, 40);

    if (!Float.isNaN(inGx) && !Float.isNaN(inGy) && !Float.isNaN(inGz) &&
        !Float.isInfinite(inGx) && !Float.isInfinite(inGy) && !Float.isInfinite(inGz)) {
      gyroDpsX = inGx; gyroDpsY = inGy; gyroDpsZ = inGz;
    }

    float inAx = readFloatLE(payload, 44);
    float inAy = readFloatLE(payload, 48);
    float inAz = readFloatLE(payload, 52);

    if (!Float.isNaN(inAx) && !Float.isNaN(inAy) && !Float.isNaN(inAz) &&
        !Float.isInfinite(inAx) && !Float.isInfinite(inAy) && !Float.isInfinite(inAz)) {
      accelGX = inAx; accelGY = inAy; accelGZ = inAz;
    }

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
  // Guardas estrictas anti-NaN y anti-Infinito para proteger el pipeline OpenGL
  if (Float.isNaN(w) || Float.isNaN(x) || Float.isNaN(y) || Float.isNaN(z) ||
      Float.isInfinite(w) || Float.isInfinite(x) || Float.isInfinite(y) || Float.isInfinite(z)) {
    return;
  }

  // Normalizar cuaternion para renderizado seguro
  float norm = sqrt(w*w + x*x + y*y + z*z);
  if (Float.isNaN(norm) || Float.isInfinite(norm) || norm < 1e-6f) return;
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

