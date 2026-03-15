/*
 * HC-SR04 Calibration Sketch — Arduino R4
 * 
 * Takes rapid-fire distance readings and logs them over Serial
 * for noise profiling. Place the sensor facing a flat wall at a
 * known distance, then press 's' to start a capture burst.
 * 
 * WIRING:
 *   HC-SR04 VCC  → 5V
 *   HC-SR04 GND  → GND
 *   HC-SR04 TRIG → Pin 9
 *   HC-SR04 ECHO → Pin 10
 * 
 * USAGE:
 *   1. Upload this sketch
 *   2. Open Serial Monitor at 115200 baud
 *   3. Place sensor at known distance from a flat wall
 *   4. Type the actual distance in cm and press Enter (e.g. "30")
 *   5. Type "s" and press Enter to start a 100-reading burst
 *   6. Move sensor to next distance, repeat
 *   7. Type "done" when finished — prints a summary
 * 
 * OUTPUT FORMAT (CSV over Serial):
 *   BURST,actual_cm,reading_num,raw_us,distance_cm
 * 
 * The Python calibration tool reads this directly.
 */

#define TRIG_PIN 9
#define ECHO_PIN 10
#define READINGS_PER_BURST 100
#define DELAY_BETWEEN_MS 60    // 60ms between readings (sequential safe)
#define SPEED_OF_SOUND 0.0343  // cm per microsecond (at ~20°C)
#define MAX_DISTANCE_CM 400.0
#define TIMEOUT_US 25000       // ~430cm max

float actualDistance = 0.0;
int burstCount = 0;

// Rolling stats
#define MAX_BURSTS 20
float burstActual[MAX_BURSTS];
float burstMean[MAX_BURSTS];
float burstStd[MAX_BURSTS];
float burstMin[MAX_BURSTS];
float burstMax[MAX_BURSTS];
int burstDropouts[MAX_BURSTS];

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  delay(100);
  
  Serial.println("=== HC-SR04 CALIBRATION ===");
  Serial.println("Commands:");
  Serial.println("  <number>  Set actual distance in cm (e.g. '30')");
  Serial.println("  s         Start a 100-reading burst");
  Serial.println("  t         Single test reading");
  Serial.println("  done      Print summary and finish");
  Serial.println("");
  Serial.println("CSV header: BURST,actual_cm,reading_num,raw_us,distance_cm");
  Serial.println("===========================");
  Serial.println("");
}

float readSensor() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo
  long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  
  if (duration == 0) {
    return -1.0;  // timeout / no echo
  }
  
  float distance = (duration * SPEED_OF_SOUND) / 2.0;
  
  if (distance > MAX_DISTANCE_CM) {
    return -1.0;
  }
  
  return distance;
}

void runBurst() {
  if (actualDistance <= 0) {
    Serial.println("ERROR: Set actual distance first! Type a number (e.g. '30')");
    return;
  }
  
  if (burstCount >= MAX_BURSTS) {
    Serial.println("ERROR: Max bursts reached. Type 'done' for summary.");
    return;
  }
  
  Serial.print("Starting burst at ");
  Serial.print(actualDistance, 1);
  Serial.print("cm (");
  Serial.print(READINGS_PER_BURST);
  Serial.println(" readings)...");
  
  float readings[READINGS_PER_BURST];
  long rawUs[READINGS_PER_BURST];
  int validCount = 0;
  int dropouts = 0;
  float sum = 0;
  
  for (int i = 0; i < READINGS_PER_BURST; i++) {
    // Trigger
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
    float dist;
    
    if (duration == 0) {
      dist = -1.0;
      dropouts++;
    } else {
      dist = (duration * SPEED_OF_SOUND) / 2.0;
      if (dist > MAX_DISTANCE_CM) {
        dist = -1.0;
        dropouts++;
      }
    }
    
    readings[i] = dist;
    rawUs[i] = duration;
    
    // Print CSV line
    Serial.print("BURST,");
    Serial.print(actualDistance, 1);
    Serial.print(",");
    Serial.print(i);
    Serial.print(",");
    Serial.print(duration);
    Serial.print(",");
    Serial.println(dist, 2);
    
    if (dist > 0) {
      sum += dist;
      validCount++;
    }
    
    delay(DELAY_BETWEEN_MS);
  }
  
  // Compute stats
  float mean = (validCount > 0) ? sum / validCount : 0;
  float sumSqDiff = 0;
  float minVal = 9999, maxVal = -1;
  
  for (int i = 0; i < READINGS_PER_BURST; i++) {
    if (readings[i] > 0) {
      float diff = readings[i] - mean;
      sumSqDiff += diff * diff;
      if (readings[i] < minVal) minVal = readings[i];
      if (readings[i] > maxVal) maxVal = readings[i];
    }
  }
  
  float std = (validCount > 1) ? sqrt(sumSqDiff / (validCount - 1)) : 0;
  
  // Store for summary
  burstActual[burstCount] = actualDistance;
  burstMean[burstCount] = mean;
  burstStd[burstCount] = std;
  burstMin[burstCount] = minVal;
  burstMax[burstCount] = maxVal;
  burstDropouts[burstCount] = dropouts;
  burstCount++;
  
  // Print burst summary
  Serial.println("---");
  Serial.print("STATS,");
  Serial.print(actualDistance, 1);
  Serial.print(",mean=");
  Serial.print(mean, 2);
  Serial.print(",std=");
  Serial.print(std, 2);
  Serial.print(",min=");
  Serial.print(minVal, 2);
  Serial.print(",max=");
  Serial.print(maxVal, 2);
  Serial.print(",dropouts=");
  Serial.print(dropouts);
  Serial.print("/");
  Serial.println(READINGS_PER_BURST);
  Serial.println("---");
  Serial.println("Ready for next distance. Set distance then type 's'.");
  Serial.println("");
}

void printSummary() {
  Serial.println("");
  Serial.println("========== CALIBRATION SUMMARY ==========");
  Serial.println("Actual(cm) | Mean(cm) | Std(cm) | Min    | Max    | Dropouts");
  Serial.println("-----------|----------|---------|--------|--------|--------");
  
  for (int i = 0; i < burstCount; i++) {
    char buf[100];
    snprintf(buf, sizeof(buf), "%10s |", "");
    
    Serial.print("SUMMARY,");
    Serial.print(burstActual[i], 1);
    Serial.print(",");
    Serial.print(burstMean[i], 2);
    Serial.print(",");
    Serial.print(burstStd[i], 2);
    Serial.print(",");
    Serial.print(burstMin[i], 2);
    Serial.print(",");
    Serial.print(burstMax[i], 2);
    Serial.print(",");
    Serial.println(burstDropouts[i]);
  }
  
  Serial.println("=========================================");
  Serial.println("Copy everything above and save to a .txt file.");
  Serial.println("Then run: python calibration/analyze.py --input your_file.txt");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "s" || input == "S") {
      runBurst();
    }
    else if (input == "t" || input == "T") {
      float d = readSensor();
      Serial.print("TEST: ");
      if (d < 0) {
        Serial.println("NO ECHO (timeout)");
      } else {
        Serial.print(d, 2);
        Serial.println(" cm");
      }
    }
    else if (input.equalsIgnoreCase("done")) {
      printSummary();
    }
    else {
      // Try to parse as number (actual distance)
      float val = input.toFloat();
      if (val > 0) {
        actualDistance = val;
        Serial.print("Actual distance set to: ");
        Serial.print(actualDistance, 1);
        Serial.println(" cm. Type 's' to start burst.");
      } else {
        Serial.println("Unknown command. Use: <number>, s, t, or done");
      }
    }
  }
}
