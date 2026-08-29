/**
 * FlightVisualizerV2.pde
 * High-Integrity Attitude Estimator Ground Station (Processing 4 / P3D)
 * 
 * Features:
 * - Handshake and profile selection FSM (Screen 1)
 * - Calibration and BIST monitoring (Screen 2)
 * - Flat 3D Board Attitude Visualizer with Reference Grid (Screen 3)
 * - Strict Fletcher-16 Checksum validation on all binary frames
 * - Interactive DEMO / Simulation Mode (Press 'D') for offline testing
 */

import processing.serial.*;

// Protocol constants matching telemetry_protocol.hpp
final int PREAMBLE_0 = 0xAA;
final int PREAMBLE_1 = 0x55;

final int MSG_HEARTBEAT_AWAIT_PROFILE = 0x01;
final int MSG_CMD_SET_PROFILE         = 0x02;
final int MSG_BIST_REPORT             = 0x03;
final int MSG_ESTIMATOR_TELEMETRY     = 0x04;
final int MSG_ACK_NACK                = 0x05;
final int MSG_CMD_SYSTEM_RESET        = 0x06;

// Health flags bitmask
final int FLAG_IMU_OK                = (1 << 0);
final int FLAG_EKF_CONVERGED         = (1 << 1);
final int FLAG_TIMER_FALLBACK_ACTIVE = (1 << 2);
final int FLAG_HIGH_G_REJECTION      = (1 << 3);
final int FLAG_ANOMALY_DETECTED      = (1 << 4);
final int FLAG_BIST_PASSED           = (1 << 5);
final int FLAG_HARD_FAULT            = (1 << 6);
final int FLAG_TELEMETRY_STREAMING   = (1 << 7);

// UI System States
final int UI_STATE_PORT_SELECT       = 0;
final int UI_STATE_AWAITING_PROFILE  = 1;
final int UI_STATE_BIST_CALIBRATION  = 2;
final int UI_STATE_RUNNING_ESTIMATOR = 3;
final int UI_STATE_HARD_FAULT        = 4;

int currentUiState = UI_STATE_PORT_SELECT;
Serial serialPort = null;
String selectedPortName = "NO SERIAL (DEMO)";
int baudRate = 115200;
boolean isDemoMode = false;
float demoSimTime = 0.0f;

// Telemetry state variables
long lastPacketTimeMs = 0;
int rxPacketCount = 0;
int checksumErrors = 0;

// Heartbeat & BIST
long espUptimeMs = 0;
int espSystemState = 0;
int healthFlags = FLAG_IMU_OK | FLAG_EKF_CONVERGED | FLAG_BIST_PASSED | FLAG_TELEMETRY_STREAMING;
int bistCode = 0;
int bistProgressPct = 0;

// Estimator Telemetry Data
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float rollDeg = 0.0f, pitchDeg = 0.0f, yawDeg = 0.0f;
float gyroDpsX = 0.0f, gyroDpsY = 0.0f, gyroDpsZ = 0.0f;
float accelGX = 0.0f, accelGY = 0.0f, accelGZ = 0.0f;
long wcetCycles = 14280;
float wcetUs = 59.5f;
float loopFreqHz = 500.0f;
int activeProfileId = 1;

// Serial RX parsing state machine
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
  surface.setTitle("High-Integrity Flight Estimator V2 - Ground Station");
  textFont(createFont("Consolas", 14));
  frameRate(60);
}

void draw() {
  background(15, 20, 28);
  
  // Read and process serial stream if connected
  processSerial();

  // Run demo physics simulation if in demo mode
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
// UI SCREEN RENDERING
// -------------------------------------------------------------

void drawPortSelectionScreen() {
  hint(DISABLE_DEPTH_TEST);
  
  fill(0, 200, 255);
  textSize(24);
  textAlign(CENTER, TOP);
  text("FLIGHT ESTIMATOR V2 - GROUND STATION", width/2, 40);

  textSize(14);
  fill(180, 200, 220);
  text("Select active serial COM port connecting ESP32 (Baud: 115200)", width/2, 80);

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
      text("[" + (i + 1) + "]  CONNECT TO " + ports[i], width/2, y + 19);
    }
  } else {
    fill(255, 120, 120);
    textAlign(CENTER, CENTER);
    text("No physical serial ports currently available.", width/2, 160);
  }

  // Demo simulation mode card
  float demoY = (ports != null && ports.length > 0) ? startY + ports.length * 50 + 30 : 220;
  boolean hoverDemo = (mouseX > width/2 - 160 && mouseX < width/2 + 160 && mouseY > demoY && mouseY < demoY + 45);

  fill(hoverDemo ? color(20, 120, 80) : color(18, 70, 50));
  stroke(hoverDemo ? color(0, 255, 180) : color(40, 150, 100));
  strokeWeight(2);
  rect(width/2 - 160, demoY, 320, 45, 6);

  fill(0, 255, 200);
  textAlign(CENTER, CENTER);
  textSize(14);
  text("[D]  RUN 3D DEMO SIMULATION (NO ESP32)", width/2, demoY + 22);

  fill(140, 170, 190);
  textSize(12);
  textAlign(CENTER, BOTTOM);
  text("You can press 'D' anytime to preview the 3D attitude visualizer & telemetry gauges offline.", width/2, height - 25);
}

void drawProfileSelectionScreen() {
  hint(DISABLE_DEPTH_TEST);

  fill(0, 220, 255);
  textSize(22);
  textAlign(CENTER, TOP);
  text("PRE-FLIGHT CONFIGURATION - SELECT FLIGHT PROFILE", width/2, 30);

  fill(160, 190, 210);
  textSize(13);
  text("Port: " + selectedPortName + " | ESP32 Status: AWAITING_PROFILE", width/2, 65);

  // Profile cards
  drawProfileCard(1, "DRONE_HOVER", "200 Hz", "+/- 250 dps", "+/- 2 g", "DLPF ~98Hz | High Resolution Smooth Hover", 80, 110, 440, 240);
  drawProfileCard(2, "DRONE_ACRO", "500 Hz", "+/- 1000 dps", "+/- 8 g", "DLPF ~188Hz | Fast Dynamics & Acrobatic Maneuvers", 580, 110, 440, 240);
  drawProfileCard(3, "ROCKET_LAUNCH", "500 Hz", "+/- 1000 dps", "+/- 16 g", "DLPF ~188Hz | High Axial Acceleration Rejection", 80, 380, 440, 240);
  drawProfileCard(4, "MISSILE_HIGH_G", "1000 Hz", "+/- 2000 dps", "+/- 16 g", "DLPF ~256Hz | Extreme Dynamics (1ms Hard Loop)", 580, 380, 440, 240);

  fill(255, 230, 100);
  textSize(14);
  textAlign(CENTER, BOTTOM);
  text("Press key '1', '2', '3', '4' (or click a card) to select profile | Press 'D' for Demo", width/2, height - 20);
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
  text("Target Frequency: " + freq, x + 20, y + 55);
  text("Gyroscope Range:  " + gyro, x + 20, y + 80);
  text("Accel Range:      " + accel, x + 20, y + 105);

  fill(140, 170, 200);
  textSize(12);
  text(desc, x + 20, y + 145, w - 40, 60);

  fill(hover ? color(0, 255, 200) : color(80, 130, 180));
  rect(x + w - 90, y + h - 35, 75, 24, 4);
  fill(hover ? 0 : 255);
  textAlign(CENTER, CENTER);
  text("SELECT", x + w - 52, y + h - 23);
}

void drawBistScreen() {
  hint(DISABLE_DEPTH_TEST);

  fill(0, 220, 255);
  textSize(22);
  textAlign(CENTER, TOP);
  text("ESP32 HARDWARE INITIALIZATION & CALIBRATION", width/2, 100);

  fill(180, 200, 220);
  textSize(14);
  text("Calibrating IMU sensor biases and performing BIST self-tests...", width/2, 150);

  // Progress bar
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
  text("Profile Locked: " + activeProfileId + " | Waiting for estimator lock-free loop start...", width/2, 300);
}

void drawDashboardScreen() {
  // 1. Draw 3D Attitude Visualizer (Left Panel)
  draw3DVisualizer(30, 35, 540, 670);

  // 2. Draw 2D Telemetry & Health Gauges (Right Panel)
  drawTelemetryPanel(600, 35, 490, 670);
}

// -------------------------------------------------------------
// 3D FLAT BOARD ATTITUDE VISUALIZER
// -------------------------------------------------------------

void draw3DVisualizer(float x, float y, float w, float h) {
  // Step 1: Draw panel background in 2D
  hint(DISABLE_DEPTH_TEST);
  fill(12, 18, 26);
  stroke(35, 60, 85);
  strokeWeight(1.5);
  rect(x, y, w, h, 8);

  // Step 2: Render 3D Scene with Depth Testing
  hint(ENABLE_DEPTH_TEST);
  pushMatrix();

  // Center 3D viewport
  translate(x + w/2, y + h/2 + 25, 0);

  // True Mathematical Isometric Spatial Projection (Viewed from FRONT-TOP diagonal)
  rotateX(-atan(1.0f / sqrt(2.0f))); // Exact -35.264°: elevated above
  rotateY(radians(135));              // 135°: facing the FRONT-RIGHT of the board

  // Dynamic 3D lighting
  lights();
  ambientLight(90, 110, 140);
  directionalLight(230, 245, 255, -0.4, 0.9, -0.6);
  // 1. Reference Ground Grid Platform (Square grid in horizontal X-Z plane below the board)
  drawGroundReferenceGrid(160, 32, 110);

  // 2. Apply Attitude Quaternion Matrix matching IMU axes to Processing 3D coordinates:
  // IMU Y (Pitch) -> Processing X (Lateral tilt)
  // IMU Z (Yaw)   -> Processing Y (Vertical turn)
  // IMU X (Roll)  -> Processing Z (Longitudinal bank)
  pushMatrix();
  applyQuaternionRotation(q0, q2, -q3, -q1);

  // 3. Render Clean Flat IMU Board / Box Model
  drawFlatBoardModel();

  popMatrix();
  popMatrix();

  // Step 3: Draw 2D HUD overlays on top
  hint(DISABLE_DEPTH_TEST);

  fill(0, 220, 255);
  textSize(16);
  textAlign(LEFT, TOP);
  text("3D ATTITUDE ESTIMATOR (IMU BOARD)", x + 20, y + 18);

  // Euler Attitude HUD Box
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
    text("[DEMO SIMULATION]", x + w - 20, y + 20);
  }
}

void drawGroundReferenceGrid(float size, float step, float yOffset) {
  pushMatrix();
  translate(0, yOffset, 0); // Position below the board in 3D

  // Draw grid lines in X-Z plane
  strokeWeight(1);
  for (float i = -size; i <= size + 0.1f; i += step) {
    // Longitudinal lines (along Z)
    stroke(0, 180, 255, (abs(i) < 0.1f) ? 140 : 45);
    line(i, 0, -size, i, 0, size);

    // Lateral lines (along X)
    stroke(0, 180, 255, (abs(i) < 0.1f) ? 140 : 45);
    line(-size, 0, i, size, 0, i);
  }

  // Outer border of square platform
  stroke(0, 220, 255, 120);
  strokeWeight(1.5);
  line(-size, 0, -size, size, 0, -size);
  line(size, 0, -size, size, 0, size);
  line(size, 0, size, -size, 0, size);
  line(-size, 0, size, -size, 0, -size);

  // Central cardinal axis lines on the floor
  strokeWeight(2.5);
  stroke(255, 70, 70, 200); line(0, 0, 0, 0, 0, -size); // Forward (-Z)
  stroke(70, 255, 70, 200); line(0, 0, 0, size, 0, 0);  // Right (+X)

  popMatrix();
}

void drawFlatBoardModel() {
  // 1. 3D Body Frame Coordinate Axes
  strokeWeight(3);
  stroke(255, 50, 50); line(0, 0, 0, 0, 0, -110);  // X / Forward (Red)
  stroke(50, 255, 50); line(0, 0, 0, 110, 0, 0);   // Y / Right (Green)
  stroke(50, 100, 255); line(0, 0, 0, 0, 60, 0);   // Z / Down (Blue)

  // 2. Main Flat PCB Board Box (Width X = 190, Height Y = 14, Length Z = 130)
  fill(25, 45, 75);
  stroke(0, 220, 255);
  strokeWeight(1.5);
  box(190, 14, 130);

  // 3. Sensor IC Chip in center (Black square with pin-1 dot)
  fill(20, 20, 25);
  stroke(100, 140, 180);
  strokeWeight(1);
  pushMatrix();
  translate(0, -8, 0);
  box(36, 4, 36);
  popMatrix();

  // 4. Front Heading Indicator (Bright Orange/Red forward strip)
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
  text("HARD REAL-TIME DASHBOARD", x + 20, y + 18);

  // Performance Section
  fill(25, 40, 58);
  noStroke();
  rect(x + 20, y + 55, w - 40, 130, 6);

  fill(255);
  textSize(13);
  text("GNC Loop Frequency:     " + nf(loopFreqHz, 1, 1) + " Hz", x + 35, y + 70);
  text("WCET Execution Time:    " + nf(wcetUs, 1, 1) + " us (" + wcetCycles + " cycles)", x + 35, y + 95);
  text("Packets RX / Errors:    " + rxPacketCount + " / " + checksumErrors, x + 35, y + 120);
  text("Active Flight Profile:  Profile " + activeProfileId, x + 35, y + 145);

  // Inertial Data Section
  fill(25, 40, 58);
  rect(x + 20, y + 200, w - 40, 140, 6);

  fill(255);
  text("Gyroscope Rates (dps):", x + 35, y + 215);
  fill(200, 220, 255);
  text("Wx: " + nf(gyroDpsX, 1, 2) + "   Wy: " + nf(gyroDpsY, 1, 2) + "   Wz: " + nf(gyroDpsZ, 1, 2), x + 50, y + 240);

  fill(255);
  text("Accelerometer (g):", x + 35, y + 275);
  fill(200, 220, 255);
  text("Ax: " + nf(accelGX, 1, 3) + "   Ay: " + nf(accelGY, 1, 3) + "   Az: " + nf(accelGZ, 1, 3), x + 50, y + 300);

  // Health Flags Matrix
  fill(25, 40, 58);
  rect(x + 20, y + 355, w - 40, 180, 6);

  fill(255);
  text("SYSTEM HEALTH FLAGS (FDIR):", x + 35, y + 370);

  drawStatusLed("IMU_OK", (healthFlags & FLAG_IMU_OK) != 0, x + 40, y + 405);
  drawStatusLed("EKF_CONVERGED", (healthFlags & FLAG_EKF_CONVERGED) != 0, x + 240, y + 405);
  drawStatusLed("TIMER_FALLBACK", (healthFlags & FLAG_TIMER_FALLBACK_ACTIVE) != 0, x + 40, y + 445);
  drawStatusLed("HIGH_G_REJECT", (healthFlags & FLAG_HIGH_G_REJECTION) != 0, x + 240, y + 445);
  drawStatusLed("ANOMALY_DETECT", (healthFlags & FLAG_ANOMALY_DETECTED) != 0, x + 40, y + 485);
  drawStatusLed("STREAMING", (healthFlags & FLAG_TELEMETRY_STREAMING) != 0, x + 240, y + 485);

  // Reset / Return Button
  boolean hoverReset = (mouseX >= x + 20 && mouseX <= x + w - 20 && mouseY >= y + h - 50 && mouseY <= y + h - 15);
  fill(hoverReset ? color(200, 40, 40) : color(140, 30, 30));
  stroke(255, 80, 80);
  rect(x + 20, y + h - 50, w - 40, 35, 6);
  fill(255);
  textAlign(CENTER, CENTER);
  text(isDemoMode ? "RETURN TO PROFILE MENU (RESET SIM)" : "TRIGGER SOFTWARE RESET (REBOOT ESP32)", x + w/2, y + h - 33);
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
  text("HARD FAULT LOCK ACTIVE", width/2, 150);

  fill(220, 200, 200);
  textSize(14);
  text("The ESP32 encountered an unrecoverable flight integrity fault.", width/2, 200);
  text("System has locked actuators and halted estimator to prevent divergence.", width/2, 230);
  text("Please power cycle the board or trigger a hardware reset.", width/2, 280);
}

// -------------------------------------------------------------
// DEMO SIMULATION ENGINE (OFFLINE TESTING)
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
    // Generate smooth continuous 3D Euler rotations for visualization test
    rollDeg  = sin(demoSimTime * 1.2f) * 25.0f;
    pitchDeg = cos(demoSimTime * 0.9f) * 20.0f;
    yawDeg   = (demoSimTime * 15.0f) % 360.0f;

    // Convert Euler to normalized quaternion [qw, qx, qy, qz]
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

    // Simulated inertial rates and accel
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
// PROTOCOL & SERIAL COMMUNICATION
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

    // Demo mode button click
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
    // Check reset button
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
    // Offline demo fallback
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
    println("Connected to: " + portName);
  } catch (Exception e) {
    println("Error connecting to port: " + e.getMessage());
    currentUiState = UI_STATE_AWAITING_PROFILE;
  }
}

void sendSetProfileCmd(int profileId) {
  if (serialPort == null) return;

  byte[] pkt = new byte[7];
  pkt[0] = (byte)PREAMBLE_0;
  pkt[1] = (byte)PREAMBLE_1;
  pkt[2] = (byte)MSG_CMD_SET_PROFILE;
  pkt[3] = (byte)1; // Length = 1 byte
  pkt[4] = (byte)profileId;

  // Calculate Fletcher-16 over bytes 2, 3, 4 (msg_id, len, payload)
  int chk = calculateFletcher16(pkt, 2, 3);
  pkt[5] = (byte)(chk & 0xFF);
  pkt[6] = (byte)((chk >> 8) & 0xFF);

  serialPort.write(pkt);
  println("Sent CMD_SET_PROFILE: " + profileId);
}

void sendSystemResetCmd() {
  if (serialPort == null) return;

  byte[] pkt = new byte[6];
  pkt[0] = (byte)PREAMBLE_0;
  pkt[1] = (byte)PREAMBLE_1;
  pkt[2] = (byte)MSG_CMD_SYSTEM_RESET;
  pkt[3] = (byte)0; // Length = 0 bytes

  int chk = calculateFletcher16(pkt, 2, 2);
  pkt[4] = (byte)(chk & 0xFF);
  pkt[5] = (byte)((chk >> 8) & 0xFF);

  serialPort.write(pkt);
  println("Sent CMD_SYSTEM_RESET");
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
  isDemoMode = false; // Real hardware telemetry packet overrides demo

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
    q1 = readFloatLE(payload, 8);  // IMU X (Roll)
    q2 = readFloatLE(payload, 12); // IMU Y (Pitch)
    q3 = readFloatLE(payload, 16); // IMU Z (Yaw)

    rollDeg  = readFloatLE(payload, 20); // Roll (Alabeo lateral)
    pitchDeg = readFloatLE(payload, 24); // Pitch (Cabeceo morro)
    yawDeg   = readFloatLE(payload, 28); // Yaw (Giro brújula / mesa)

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
    println("Received ACK_NACK for Msg 0x" + hex(refMsg, 2) + " Status: " + status);
    if (refMsg == MSG_CMD_SET_PROFILE && status == 0) {
      currentUiState = UI_STATE_BIST_CALIBRATION;
    }
  }
}

// -------------------------------------------------------------
// BINARY DESERIALIZATION HELPERS (LITTLE ENDIAN)
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
  // Normalize quaternion for safe rendering
  float norm = sqrt(w*w + x*x + y*y + z*z);
  if (norm < 1e-6f) return;
  w /= norm; x /= norm; y /= norm; z /= norm;

  // Convert quaternion to standard 3D rotation matrix
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
