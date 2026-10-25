#include <Wire.h>
#include <Adafruit_VEML7700.h>

// ---------------------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------------------
const uint8_t PIN_LDR_ADC = A0;      // buffered photoresistor divider output
const uint8_t PIN_CAP_CHARGE = 7;    // digital output: drives capacitor charge/discharge
const uint8_t PIN_CAP_SENSE = A1;    // analog input: monitors capacitor voltage

// ---------------------------------------------------------------------------
// PCT2075 register map (no dedicated Arduino library needed, it is simple)
// ---------------------------------------------------------------------------
const uint8_t PCT2075_ADDR = 0x48;   // default address, confirm against your wiring (A0 to A2 pins)
const uint8_t PCT2075_REG_TEMP = 0x00;

Adafruit_VEML7700 veml = Adafruit_VEML7700();

// ---------------------------------------------------------------------------
// LDR to lux regression (from the project calibration, R^2 = 0.91)
// RLDR = A * Lux^B, inverted for real time lux from ADC counts
// ---------------------------------------------------------------------------
const float LDR_A = 496.5855f;
const float LDR_B = 0.0403f;
const float DIVIDER_R_OHMS = 10000.0f;  // fixed 10k pulled up to 3.3V LDO rail
const float VCC = 3.3f;

// Valid indoor calibration range from the report. Outside this, trust the
// value less (error grows above about 4000 lux).
const float LDR_LUX_MIN_VALID = 243.0f;
const float LDR_LUX_MAX_VALID = 7178.0f;

// ---------------------------------------------------------------------------
// Capacitive sensor (RC time constant reset trigger)
// ---------------------------------------------------------------------------
const int CAP_THRESHOLD_COUNTS = 646;         // 63.2% of 1023
const float CAP_CHARGE_R_OHMS = 1000000.0f;   // CALIBRATE ME: your actual charging resistor value
unsigned long capTimeoutMicros = 200000UL;    // safety timeout so a bad press cannot hang the loop

// ---------------------------------------------------------------------------
// Kalman filter state (1D scalar, static model with no motion term)
// ---------------------------------------------------------------------------
struct KalmanState {
  float x_est;  // current estimate
  float P;      // current estimate uncertainty
  float Q;      // process noise (tune: higher = trusts new readings more)
  float R;      // measurement noise (tune: higher = trusts prior estimate more)
};

KalmanState opacityFilter = { 0.0f, 1.0f, 0.5f, 4.0f };   // CALIBRATE ME: Q/R for LDR channel
KalmanState tempFilter    = { 0.0f, 1.0f, 0.01f, 1.0f };  // CALIBRATE ME: Q/R for PCT2075 channel
bool filtersInitialized = false;

float kalmanUpdate(KalmanState &kf, float rawMeasurement) {
  // Predict
  float x_pred = kf.x_est;
  float P_pred = kf.P + kf.Q;

  // Update
  float K = P_pred / (P_pred + kf.R);
  kf.x_est = x_pred + K * (rawMeasurement - x_pred);
  kf.P = (1.0f - K) * P_pred;
  return kf.x_est;
}

// ---------------------------------------------------------------------------
// "Go Beyond" fermentation tracking state
// ---------------------------------------------------------------------------
enum FermentState { RISING, PEAK, OVER_PROOFED };

float baselineOpacity = 0.0f;      // set at reset: "flat dough" reference
float fullyRisenOpacity = 0.0f;    // CALIBRATE ME: your fully risen reference reading
float lastRisePercent = 0.0f;
unsigned long lastSampleMs = 0;
float riseSlopePerSec = 0.0f;

const float TEMP_REFERENCE_C = 24.0f;        // room temp reference for slope correction
const float TEMP_CORRECTION_K = 0.06f;       // CALIBRATE ME: fractional rate change per degree C
const float PEAK_RISE_PERCENT = 95.0f;       // considered "at peak" above this
const float OVERPROOF_SLOPE_DROP = -0.5f;   // percent per second, negative slope after peak means collapsing structure

FermentState currentState = RISING;
unsigned long proofStartMillis = 0;

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(PIN_CAP_CHARGE, OUTPUT);
  digitalWrite(PIN_CAP_CHARGE, LOW);

  if (!veml.begin()) {
    Serial.println("ERROR: VEML7700 not found on I2C bus.");
  }

  Serial.println("Sourdough Tracker booted. Press capacitive sensor to start a proof.");
}

// ---------------------------------------------------------------------------
void loop() {
  // 1) Check for a reset or start of proof press (bounded by a timeout)
  if (checkCapacitivePress()) {
    handleProofReset();
  }

  // 2) Regular sampling cycle (runs independent of the capacitive check)
  static unsigned long lastLoopMs = 0;
  unsigned long now = millis();
  if (now - lastLoopMs >= 1000) {   // 1 Hz sample rate, adjust as needed
    lastLoopMs = now;
    runSensorCycle(now);
  }
}

// ---------------------------------------------------------------------------
// Capacitive sensor: RC time constant estimator
// Returns true if a press (higher capacitance) was detected.
// ---------------------------------------------------------------------------
bool checkCapacitivePress() {
  // Fully discharge
  digitalWrite(PIN_CAP_CHARGE, LOW);
  delayMicroseconds(50);

  // Start charging and time until threshold is crossed
  digitalWrite(PIN_CAP_CHARGE, HIGH);
  unsigned long startMicros = micros();
  unsigned long elapsed = 0;
  bool crossed = false;

  while (elapsed < capTimeoutMicros) {
    if (analogRead(PIN_CAP_SENSE) >= CAP_THRESHOLD_COUNTS) {
      crossed = true;
      break;
    }
    elapsed = micros() - startMicros;
  }

  digitalWrite(PIN_CAP_CHARGE, LOW);  // discharge again for next cycle

  if (!crossed) return false;

  float tau_seconds = elapsed / 1e6f;
  float capacitance_F = tau_seconds / CAP_CHARGE_R_OHMS;  // C = tau / R

  // A "press" is a deliberate, distinctly different charge time than the
  // resting baseline predicts. CALIBRATE ME: tune this threshold against your
  // own observed resting (about 3 nF) vs compressed (about 4 nF) range from testing.
  const float PRESS_CAPACITANCE_THRESHOLD_F = 3.5e-9f;
  return capacitance_F >= PRESS_CAPACITANCE_THRESHOLD_F;
}

// ---------------------------------------------------------------------------
void handleProofReset() {
  Serial.println(">>> Reset triggered: zeroing baseline, restarting Go Beyond tracking.");

  float rawLux = readLdrLux();
  baselineOpacity = rawLux;
  lastRisePercent = 0.0f;
  riseSlopePerSec = 0.0f;
  currentState = RISING;
  proofStartMillis = millis();

  // Reseed the Kalman filter so it does not smooth against a stale reference
  opacityFilter.x_est = rawLux;
  opacityFilter.P = 1.0f;
  filtersInitialized = true;

  logProofStart(proofStartMillis);
}

// ---------------------------------------------------------------------------
void runSensorCycle(unsigned long nowMs) {
  // --- Read raw sensors ---
  float rawLux = readLdrLux();
  float ambientLux = veml.readLux();
  float tempC = readPct2075TempC();

  if (!filtersInitialized) {
    opacityFilter.x_est = rawLux;
    tempFilter.x_est = tempC;
    filtersInitialized = true;
  }

  // --- Kalman filter both channels ---
  float filteredOpacity = kalmanUpdate(opacityFilter, rawLux);
  float filteredTempC = kalmanUpdate(tempFilter, tempC);

  // --- Go Beyond: normalize against ambient light to cancel room light drift ---
  float normalizedOpacity = filteredOpacity - ambientLux;

  // --- Convert to rise percentage using baseline and fully risen calibration ---
  float risePercent = 0.0f;
  if (fullyRisenOpacity != baselineOpacity) {
    risePercent = 100.0f * (normalizedOpacity - baselineOpacity) /
                  (fullyRisenOpacity - baselineOpacity);
    risePercent = constrain(risePercent, 0.0f, 150.0f);  // allow some over range for over proof detection
  }

  // --- Rate of change (slope), temperature corrected ---
  float dtSec = (lastSampleMs == 0) ? 1.0f : (nowMs - lastSampleMs) / 1000.0f;
  float rawSlope = (risePercent - lastRisePercent) / dtSec;
  float tempCorrection = 1.0f + TEMP_CORRECTION_K * (filteredTempC - TEMP_REFERENCE_C);
  riseSlopePerSec = rawSlope * tempCorrection;

  lastRisePercent = risePercent;
  lastSampleMs = nowMs;

  // --- Classify fermentation stage ---
  currentState = classifyState(risePercent, riseSlopePerSec);

  logSample(nowMs, rawLux, ambientLux, filteredTempC, risePercent, riseSlopePerSec, currentState);
}

// ---------------------------------------------------------------------------
FermentState classifyState(float risePercent, float slopePerSec) {
  if (risePercent >= PEAK_RISE_PERCENT && slopePerSec <= OVERPROOF_SLOPE_DROP) {
    return OVER_PROOFED;
  }
  if (risePercent >= PEAK_RISE_PERCENT) {
    return PEAK;
  }
  return RISING;
}

// ---------------------------------------------------------------------------
// Sensor read helpers
// ---------------------------------------------------------------------------
float readLdrLux() {
  int adcCount = analogRead(PIN_LDR_ADC);
  float vOut = (adcCount / 1023.0f) * VCC;

  // Guard against divide by zero as vOut approaches VCC
  if (vOut >= VCC - 0.001f) vOut = VCC - 0.001f;

  // Voltage divider: LDR on top, fixed 10k on bottom, output vOut
  float rLdr = DIVIDER_R_OHMS * (VCC - vOut) / vOut;

  // Invert RLDR = A * Lux^B  =>  Lux = (RLDR / A) ^ (1/B)
  float lux = pow(rLdr / LDR_A, 1.0f / LDR_B);
  return lux;  // caller or log can flag if outside LDR_LUX_MIN_VALID / LDR_LUX_MAX_VALID
}

float readPct2075TempC() {
  Wire.beginTransmission(PCT2075_ADDR);
  Wire.write(PCT2075_REG_TEMP);
  Wire.endTransmission(false);
  Wire.requestFrom(PCT2075_ADDR, (uint8_t)2);

  if (Wire.available() < 2) {
    return NAN;  // I2C read failed
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  // 11 bit two's complement, left justified in a 16 bit word, 0.125 C per LSB
  int16_t raw = ((int16_t)(msb << 8 | lsb)) >> 5;
  float tempC = raw * 0.125f;
  return tempC;
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
void logProofStart(unsigned long timestampMs) {
  Serial.print("PROOF_START,");
  Serial.println(timestampMs);
}

void logSample(unsigned long t, float rawLux, float ambientLux, float tempC,
               float risePercent, float slope, FermentState state) {
  Serial.print(t);                Serial.print(",");
  Serial.print(rawLux, 2);        Serial.print(",");
  Serial.print(ambientLux, 2);    Serial.print(",");
  Serial.print(tempC, 2);         Serial.print(",");
  Serial.print(risePercent, 2);   Serial.print(",");
  Serial.print(slope, 3);         Serial.print(",");
  switch (state) {
    case RISING:        Serial.println("RISING"); break;
    case PEAK:          Serial.println("PEAK"); break;
    case OVER_PROOFED:  Serial.println("OVER_PROOFED"); break;
  }
}
