/*
 * Maze Robot Firmware — Arduino R4 WiFi
 * 
 * Creates WiFi access point, reads sensors sequentially,
 * streams data over UDP, receives motor commands back.
 * 
 * PIN LAYOUT:
 *   D2  - ENCODER Left (interrupt)
 *   D3  - ENCODER Right (interrupt)
 *   D4  - IN1 (left motor dir)
 *   D5  - EN A (left motor PWM)
 *   D6  - EN B (right motor PWM)
 *   D7  - IN2 (left motor dir)
 *   D8  - TRIG (shared, all HC-SR04)
 *   D9  - IN3 (right motor dir)
 *   D10 - IN4 (right motor dir)
 *   D11 - ECHO HC Front
 *   D12 - ECHO HC Left
 *   D13 - ECHO HC Right
 *   A0  - IR Left (analog)
 *   A1  - IR Right (analog)
 * 
 * COMMUNICATION:
 *   WiFi AP: SSID "MazeBot", password "maze1234"
 *   UDP port 4210
 *   Arduino → Laptop: "S:front,left,right,ir_l,ir_r,heading,speed,dist,elapsed,enc_l,enc_r\n"
 *   Laptop → Arduino: "C:command_id\n"
 */

#include <WiFiS3.h>
#include <WiFiUdp.h>

// ═══════════════════════════════════════════════════════════════════
// CONFIGURATION — adjust these to match your robot
// ═══════════════════════════════════════════════════════════════════

// WiFi AP
const char* AP_SSID     = "MazeBot";
const char* AP_PASSWORD = "maze1234";
const int   UDP_PORT    = 4210;

// Motor speed (0-255)
#define BASE_SPEED 150

// Encoder calibration
#define TICKS_PER_CM     5.0    // encoder ticks per cm of travel
#define WHEELBASE_CM     12.0   // distance between wheels in cm

// Ultrasonic timing
#define US_TIMEOUT_US    25000  // ~430cm max
#define US_DELAY_MS      60     // between sequential readings
#define SPEED_OF_SOUND   0.0343 // cm per microsecond

// Safety
#define COMMAND_TIMEOUT_MS 500  // stop if no command for this long

// ═══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════

// Encoders
#define ENC_LEFT_PIN   2
#define ENC_RIGHT_PIN  3

// Motor driver (L298N)
#define IN1_PIN  4
#define ENA_PIN  5   // PWM
#define ENB_PIN  6   // PWM
#define IN2_PIN  7
#define IN3_PIN  9
#define IN4_PIN  10

// Ultrasonics
#define TRIG_PIN      8
#define ECHO_FRONT    11
#define ECHO_LEFT     12
#define ECHO_RIGHT    13

// IR sensors
#define IR_LEFT_PIN   A0
#define IR_RIGHT_PIN  A1

// ═══════════════════════════════════════════════════════════════════
// GLOBALS
// ═══════════════════════════════════════════════════════════════════

WiFiUDP udp;
IPAddress clientIP(0, 0, 0, 0);
int clientPort = 0;
bool clientConnected = false;

// Encoder counters (volatile for interrupt safety)
volatile long encLeftTicks  = 0;
volatile long encRightTicks = 0;

// Odometry
float heading         = 0.0;   // degrees, cumulative
float distanceTraveled = 0.0;  // cm
float speed_cms       = 0.0;   // cm/s estimate
long  prevEncLeft     = 0;
long  prevEncRight    = 0;
unsigned long prevOdomTime = 0;

// Sensor readings
float usFront = 0, usLeft = 0, usRight = 0;
int   irLeft  = 0, irRight = 0;

// Timing
unsigned long startTime       = 0;
unsigned long lastCommandTime = 0;
unsigned long lastSendTime    = 0;

// Current command
int currentCommand = 5; // start stopped

// ═══════════════════════════════════════════════════════════════════
// ENCODER INTERRUPTS
// ═══════════════════════════════════════════════════════════════════

void encLeftISR()  { encLeftTicks++;  }
void encRightISR() { encRightTicks++; }

// ═══════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { ; }

  Serial.println("=== MazeBot Firmware ===");

  // Motor pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  stopMotors();

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Encoder pins
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN), encLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), encRightISR, RISING);

  // IR pins (analog, no setup needed)

  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Create WiFi AP
  Serial.print("Creating AP: ");
  Serial.println(AP_SSID);

  int status = WiFi.beginAP(AP_SSID, AP_PASSWORD);
  if (status != WL_AP_LISTENING) {
    Serial.println("AP creation failed!");
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  IPAddress ip = WiFi.localIP();
  Serial.print("AP IP: ");
  Serial.println(ip);

  udp.begin(UDP_PORT);
  Serial.print("UDP listening on port ");
  Serial.println(UDP_PORT);

  startTime = millis();
  prevOdomTime = startTime;
  lastCommandTime = startTime;

  Serial.println("Ready. Waiting for laptop...");
}

// ═══════════════════════════════════════════════════════════════════
// MAIN LOOP
// ═══════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // Blink LED while waiting, solid when connected
  if (!clientConnected) {
    digitalWrite(LED_BUILTIN, (now / 300) % 2);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // ── Read sensors ──────────────────────────────────────────────
  readUltrasonics();
  irLeft  = analogRead(IR_LEFT_PIN);
  irRight = analogRead(IR_RIGHT_PIN);

  // ── Update odometry ───────────────────────────────────────────
  updateOdometry();

  // ── Receive commands ──────────────────────────────────────────
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[64];
    int len = udp.read(buf, sizeof(buf) - 1);
    buf[len] = '\0';

    // Remember client address for sending data back
    clientIP = udp.remoteIP();
    clientPort = udp.remotePort();
    clientConnected = true;

    // Parse command: "C:0\n"
    if (buf[0] == 'C' && buf[1] == ':') {
      int cmd = atoi(&buf[2]);
      if (cmd >= 0 && cmd <= 5) {
        currentCommand = cmd;
        lastCommandTime = now;
      }
    }
  }

  // ── Safety: stop if no commands ───────────────────────────────
  if (now - lastCommandTime > COMMAND_TIMEOUT_MS) {
    currentCommand = 5; // stop
  }

  // ── Execute motor command ─────────────────────────────────────
  executeCommand(currentCommand);

  // ── Send sensor data ──────────────────────────────────────────
  if (clientConnected) {
    float elapsed = (now - startTime) / 1000.0;

    char packet[200];
    snprintf(packet, sizeof(packet),
      "S:%.1f,%.1f,%.1f,%d,%d,%.1f,%.1f,%.1f,%.2f,%ld,%ld\n",
      usFront, usLeft, usRight,
      irLeft, irRight,
      heading, speed_cms, distanceTraveled, elapsed,
      encLeftTicks, encRightTicks
    );

    udp.beginPacket(clientIP, clientPort);
    udp.write(packet);
    udp.endPacket();
  }
}

// ═══════════════════════════════════════════════════════════════════
// ULTRASONIC READING (sequential)
// ═══════════════════════════════════════════════════════════════════

float readOneUltrasonic(int echoPin) {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(echoPin, HIGH, US_TIMEOUT_US);
  if (duration == 0) return 200.0; // max range on timeout

  float dist = (duration * SPEED_OF_SOUND) / 2.0;
  return min(dist, 200.0);
}

void readUltrasonics() {
  usFront = readOneUltrasonic(ECHO_FRONT);
  delay(US_DELAY_MS);
  usLeft = readOneUltrasonic(ECHO_LEFT);
  delay(US_DELAY_MS);
  usRight = readOneUltrasonic(ECHO_RIGHT);
  delay(US_DELAY_MS);
}

// ═══════════════════════════════════════════════════════════════════
// ODOMETRY
// ═══════════════════════════════════════════════════════════════════

void updateOdometry() {
  unsigned long now = millis();
  float dt = (now - prevOdomTime) / 1000.0;
  if (dt < 0.01) return; // too soon

  // Read encoder ticks atomically
  noInterrupts();
  long leftTicks  = encLeftTicks;
  long rightTicks = encRightTicks;
  interrupts();

  long dLeft  = leftTicks  - prevEncLeft;
  long dRight = rightTicks - prevEncRight;
  prevEncLeft  = leftTicks;
  prevEncRight = rightTicks;

  float dLeftCm  = dLeft  / TICKS_PER_CM;
  float dRightCm = dRight / TICKS_PER_CM;
  float dCenter  = (dLeftCm + dRightCm) / 2.0;

  // Update distance
  distanceTraveled += abs(dCenter);

  // Update heading from differential
  float dTheta = (dRightCm - dLeftCm) / WHEELBASE_CM;
  heading += dTheta * (180.0 / PI);

  // Normalize heading to [0, 360)
  while (heading < 0)   heading += 360.0;
  while (heading >= 360) heading -= 360.0;

  // Speed estimate
  speed_cms = abs(dCenter) / dt;

  prevOdomTime = now;
}

// ═══════════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════

void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor: IN1/IN2 direction, ENA speed
  if (leftSpeed >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(ENA_PIN, min(leftSpeed, 255));

  // Right motor: IN3/IN4 direction, ENB speed
  if (rightSpeed >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(ENB_PIN, min(rightSpeed, 255));
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

void executeCommand(int cmd) {
  int left, right;

  switch (cmd) {
    case 0: // forward
      left  = BASE_SPEED;
      right = BASE_SPEED;
      break;
    case 1: // slight left
      left  = (int)(BASE_SPEED * 0.6);
      right = BASE_SPEED;
      break;
    case 2: // slight right
      left  = BASE_SPEED;
      right = (int)(BASE_SPEED * 0.6);
      break;
    case 3: // hard left
      left  = -(int)(BASE_SPEED * 0.3);
      right = BASE_SPEED;
      break;
    case 4: // hard right
      left  = BASE_SPEED;
      right = -(int)(BASE_SPEED * 0.3);
      break;
    case 5: // stop
    default:
      stopMotors();
      return;
  }

  setMotors(left, right);
}
