#include "SystemState.h"
#include "Config.h"
#include <Arduino.h>
#include <EEPROM.h>

// ==================== CONFIGURATION LOADING ====================
void loadAdvancedConfig() {
  Serial.println(F("[CONFIG] Loading advanced configuration..."));
  
  // Initialize control state to safe defaults
  controlState.gpsLocked = false;
  controlState.speedValid = false;
  controlState.motorRunning = false;
  controlState.rawSpeedKmh = 0.0f;
  controlState.filteredSpeed = 0.0f;
  controlState.targetRPM = 0.0f;
  controlState.satelliteCount = 0;
  controlState.gpsHDOP = 99.0f;
  controlState.lastGPSUpdate = 0;
  controlState.systemReady = false;
  controlState.calibrationMode = false;
  controlState.lastUpdate = millis();
  
  // Set emergency stop to false initially
  emergencyStop = false;
  
  Serial.println(F("[CONFIG] Configuration loaded successfully"));
}

// ==================== SYSTEM STATE MANAGEMENT ====================
void updateSystemState(float speed, float rpm, bool gpsValid) {
  controlState.rawSpeedKmh = speed;
  controlState.targetRPM = rpm;
  controlState.speedValid = gpsValid;
  
  // Apply simple filtering
  static float lastFilteredSpeed = 0.0f;
  controlState.filteredSpeed = lastFilteredSpeed * (1.0f - SPEED_FILTER_ALPHA) + 
                               (speed / 3.6f) * SPEED_FILTER_ALPHA;
  lastFilteredSpeed = controlState.filteredSpeed;
}

void checkSystemSafety() {
  if (testMode && testSpeedSet) {
        return; // Skip GPS checks in test mode
    }
  if (controlState.rawSpeedKmh > MAX_OPERATING_SPEED) {
    Serial.println(F("[SAFETY] Speed limit exceeded"));
    emergencyStop = true;
  }
  
  // RPM safety check
  if (controlState.targetRPM > MOTOR_RPM_MAX) {
    Serial.println(F("[SAFETY] RPM limit exceeded"));
    controlState.targetRPM = MOTOR_RPM_MAX;
  }
  
  // GPS safety check - warn but don't stop
  if (!controlState.gpsLocked && controlState.motorRunning) {
    Serial.println(F("[SAFETY] GPS lost during operation"));
  }
  
  // Memory safety check
  if (ESP.getFreeHeap() < MIN_FREE_HEAP) {
    Serial.println(F("[SAFETY] Low memory condition"));
  }
}

void resetEmergencyStop() {
  if (emergencyStop) {
    emergencyStop = false;
    Serial.println(F("[SYSTEM] Emergency stop reset"));
  }
}

bool isSystemReady() {
  return controlState.gpsLocked && 
         !emergencyStop && 
         (ESP.getFreeHeap() > MIN_FREE_HEAP);
}

// ==================== SYSTEM DIAGNOSTICS ====================
void performSystemDiagnostic() {
  Serial.println(F("\n=== SYSTEM DIAGNOSTIC ==="));
  
  // GPS Diagnostic
  Serial.printf("GPS Status: %s\n", controlState.gpsLocked ? "LOCKED" : "SEARCHING");
  Serial.printf("Satellites: %d\n", controlState.satelliteCount);
  Serial.printf("HDOP: %.2f\n", controlState.gpsHDOP);
  
  // Motor Diagnostic
  Serial.printf("Motor Running: %s\n", controlState.motorRunning ? "YES" : "NO");
  Serial.printf("Target RPM: %.0f\n", controlState.targetRPM);
  
  // System Health
  Serial.printf("Free Memory: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Uptime: %.1f hours\n", millis() / 3600000.0f);
  
  // Safety Status
  Serial.printf("Emergency Stop: %s\n", emergencyStop ? "ACTIVE" : "CLEAR");
  Serial.printf("System Ready: %s\n", isSystemReady() ? "YES" : "NO");
  
  Serial.println(F("========================\n"));
}

// ==================== ERROR HANDLING ====================
void logSystemError(const char* errorMsg) {
  Serial.printf("[ERROR] %s\n", errorMsg);
}

// ==================== SYSTEM RESET FUNCTIONS ====================
void softSystemReset() {
  Serial.println(F("[SYSTEM] Performing soft reset..."));
  
  // Reset control state
  controlState.motorRunning = false;
  controlState.targetRPM = 0.0f;
  emergencyStop = false;
  
  Serial.println(F("[SYSTEM] Soft reset complete"));
}

void factoryReset() {
  Serial.println(F("[SYSTEM] Performing factory reset..."));
  
  // Clear EEPROM
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  
  // Reset calibration parameters
  calibrated = false;
  calib_a = 1.0f;
  calib_b = 0.0f;
  
  // Reset system state
  softSystemReset();
  
  Serial.println(F("[SYSTEM] Factory reset complete - restart required"));
}

// ==================== STUB FUNCTIONS FOR WEB INTERFACE ====================
// These are placeholder functions to satisfy any remaining dependencies

void updateWebClients() {
  // Web client updates handled in main loop
}

void serializeSystemData(void* response) {
  // Data serialization handled in main web server
}

void checkSystemIntegrity() {
  if (ESP.getFreeHeap() < 8192) {
    Serial.printf("WARNING: Low memory - %u bytes\n", ESP.getFreeHeap());
  }
}

void checkGPSValidity() {
  // GPS validity checking handled in main GPS processing
}

void checkMotorHealth() {
  if (controlState.targetRPM > MOTOR_RPM_MAX) {
    Serial.println(F("WARNING: Motor RPM limit exceeded"));
    controlState.targetRPM = MOTOR_RPM_MAX;
  }
}

void validateCalibrationHealth() {
  if (calibrated && (!isfinite(calib_a) || !isfinite(calib_b) || calib_a <= 0.0f)) {
    Serial.println(F("WARNING: Calibration health check failed"));
    calibrated = false;
  }
}

void triggerEmergencyStop(const char *reason) {
  emergencyStop = true;
  controlState.motorRunning = false;
  controlState.targetRPM = 0.0f;
  Serial.printf("EMERGENCY STOP: %s\n", reason);
}

void checkEmergencyConditions() {
  if (controlState.targetRPM > MOTOR_RPM_MAX * 1.2f) {
    triggerEmergencyStop("RPM limit exceeded");
  }
}

void performSystemHealthCheck() {
  uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < 15000) {
    Serial.printf("System health: %u bytes free\n", freeHeap);
  }
}
