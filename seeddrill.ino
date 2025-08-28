// ==================== INCLUDES ====================
#include <Arduino.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>

// Include enhanced headers
#include "Config.h"
#include "MotorControl.h"
#include "KalmanFilter.h"
#include "Calibration.h"
#include "SystemState.h"
#include "FuzzyPID.h"
#include "Enhanced_WebInterface.h"

// ==================== GLOBAL OBJECTS ====================
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
hd44780_I2Cexp lcd(0x27);
KalmanState kalman;
EEPROMData eepromData;
AsyncWebServer server(80);
DetailedStats detailedStats;

// ==================== SYSTEM STATE ====================
extern ControlState controlState;
extern bool emergencyStop;
extern float calib_a;
extern float calib_b;
extern bool calibrated;
extern bool testMode;
extern float testSpeedKmh;
extern bool testSpeedSet;

// Fixed: Only ONE declaration of verboseMode
bool verboseMode = false; // Set to false for field use
bool fieldMode = true; // Simple field-friendly output

// Enhanced state variables
bool awaitingSpeed = false;
bool systemReady = false;
bool autoMode = false;
bool fuzzyControlEnabled = true;
volatile unsigned long seedsDroppedCount = 0;

// System logs array
String systemLogs[50];
int logIndex = 0;

// Multi-crop variables
CropType currentCrop = CROP_WHEAT;
float customSpacing = 0.15f;
bool cropConfigChanged = false;

// WiFi AP Configuration
const char* ap_ssid = "SeedDrill_Pro";
const char* ap_password = "Agriculture2025";

// Timing variables
unsigned long lastDisplayUpdate = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastSystemCheck = 0;
unsigned long lastEnvironmentalUpdate = 0;
unsigned long lastTestModeMonitor = 0;

// Task handles for dual-core operation
TaskHandle_t taskGPSHandle = NULL;
TaskHandle_t taskMotorHandle = NULL;

// ==================== SYSTEM LOG FUNCTION ====================
void addSystemLog(String message, String level = "INFO") {
    if (logIndex >= 50) { // Fixed: Use correct array size
        // Circular buffer - overwrite oldest logs
        logIndex = 0;
    }
    
    String timestamp = String(millis() / 1000);
    systemLogs[logIndex] = "[" + timestamp + "] " + level + ": " + message;
    logIndex++;
    
    // Always print web-related logs, or if verbose mode is on
    if (verboseMode || message.indexOf("web") >= 0 || message.indexOf("Web") >= 0 ||
        message.indexOf("WEB") >= 0 || level == "ERROR" || level == "WARNING") {
        Serial.println("[LOG] " + message);
    }
    
    // Update statistics for errors
    if (level == "ERROR") {
        detailedStats.lastError = message;
        detailedStats.systemHealth = "Warning";
    }
}

// ==================== ENHANCED TEST MODE MONITORING ====================
void printSimpleFieldStatus() {
    if (!fieldMode) return;
    
    static unsigned long lastSimplePrint = 0;
    if (millis() - lastSimplePrint < 2000) return; // Print every 2 seconds
    lastSimplePrint = millis();
    
    if (testMode && testSpeedSet) {
        // TEST MODE - Simple output
        Serial.printf("TEST | Speed: %.1f km/h | Target RPM: %.0f | PWM: %d | %s\n",
            testSpeedKmh,
            controlState.targetRPM,
            getCurrentPWM(),
            controlState.motorRunning ? "RUN" : "STOP"
        );
    } else {
        // GPS MODE - Simple field output
        Serial.printf("GPS | Raw: %.1f | Filtered: %.1f km/h | Sat: %d | HDOP: %.1f | RPM: %.0f | PWM: %d\n",
            controlState.rawSpeedKmh,
            controlState.filteredSpeed * 3.6f, // Convert m/s to km/h
            controlState.satelliteCount,
            controlState.gpsHDOP,
            controlState.targetRPM,
            getCurrentPWM()
        );
    }
}

// ==================== REPLACE EXISTING VERBOSE FUNCTIONS ====================
void printTestModeStatus() {
    // Replace the existing verbose test mode printing
    if (!testMode || !testSpeedSet) return;
    
    // Only print if verbose mode is enabled
    if (verboseMode) {
        static unsigned long lastTestPrint = 0;
        if (millis() - lastTestPrint < 2000) return;
        lastTestPrint = millis();
        
        Serial.printf("[TEST] Speed: %.1f km/h | RPM: %.0f/%.0f | PWM: %d | %s\n",
            testSpeedKmh,
            controlState.targetRPM,
            controlState.actualRPM,
            getCurrentPWM(),
            controlState.motorRunning ? "RUN" : "STOP"
        );
    }
}

// ==================== SIMPLIFIED SYSTEM HEARTBEAT ====================
void printSimpleHeartbeat() {
    static unsigned long lastHeartbeat = 0;
    // Only print heartbeat every 30 seconds and only if verbose
    if (verboseMode && (millis() - lastHeartbeat > 30000)) {
        Serial.printf("[SYS] %s | %s | %.1fh uptime\n",
            systemReady ? "READY" : "WAIT",
            cropConfigs[currentCrop].name,
            millis() / 3600000.0f
        );
        lastHeartbeat = millis();
    }
}

// ==================== ENHANCED SERIAL COMMANDS ====================
void handleEnhancedSerialCommands() {
    if (!Serial.available()) return;
    
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toLowerCase();
    
    // Toggle verbose mode
    if (line == "verbose" || line == "v") {
        verboseMode = !verboseMode;
        fieldMode = !verboseMode; // Field mode is opposite of verbose
        Serial.printf("Output mode: %s\n", verboseMode ? "VERBOSE" : "FIELD");
        return;
    }
    
    if (line.startsWith("crop ")) {
        String cropName = line.substring(5);
        for (int i = 0; i < CROP_COUNT; i++) {
            String name = String(cropConfigs[i].name);
            name.toLowerCase();
            if (name == cropName) {
                currentCrop = (CropType)i;
                customSpacing = cropConfigs[i].targetSpacing;
                fuzzyController.configure_for_crop_type(currentCrop);
                Serial.printf("Crop: %s (%.1fcm)\n",
                    cropConfigs[i].name, customSpacing * 100);
                addSystemLog("Crop changed via serial: " + String(cropConfigs[i].name));
                return;
            }
        }
        Serial.println("Invalid crop. Type 'crops' to see available options.");
        return;
    }
    
    if (line == "crops") {
        Serial.println("Available crops:");
        for (int i = 0; i < CROP_COUNT; i++) {
            Serial.printf("%s ", cropConfigs[i].name);
        }
        Serial.println();
        return;
    }
    
    // Enhanced Test Mode Commands
    if (line == "t" || line == "test") {
        if (testMode) {
            testMode = false;
            testSpeedSet = false;
            stopMotor();
            Serial.println("Test OFF");
            addSystemLog("Test mode deactivated");
        } else {
            Serial.println("Test mode ON. Enter speed (1-15 km/h):");
            awaitingSpeed = true;
        }
        return;
    }
    
    if (awaitingSpeed) {
        float speed = line.toFloat();
        if (speed >= 1.0f && speed <= 15.0f) {
            testSpeedKmh = speed;
            testSpeedSet = true;
            testMode = true;
            awaitingSpeed = false;
            Serial.printf("Test speed: %.1f km/h\n", testSpeedKmh);
            addSystemLog("Test speed set: " + String(testSpeedKmh) + " km/h");
        } else {
            Serial.println("Enter 1-15 km/h:");
        }
        return;
    }
    
    if (line == "fuzzy") {
        fuzzyControlEnabled = !fuzzyControlEnabled;
        Serial.printf("Fuzzy control: %s\n", fuzzyControlEnabled ? "ENABLED" : "DISABLED");
        addSystemLog("Fuzzy control " + String(fuzzyControlEnabled ? "enabled" : "disabled"));
        return;
    }
    
    if (line == "help" || line == "h") {
        Serial.println("Commands:");
        Serial.println("t, test - Toggle test mode");
        Serial.println("crops - List crops");
        Serial.println("crop <name> - Select crop");
        Serial.println("v, verbose - Toggle output mode");
        Serial.println("fuzzy - Toggle fuzzy control");
        Serial.println("c, calib - Calibrate motor");
        Serial.println("s, status - Show system status");
        return;
    }
    
    // Other existing commands (only show output if verbose)
    if (line == "c" || line == "calib") {
        performEnhancedCalibration();
    } else if (line == "s" || line == "status") {
        printDetailedStatus();
    }
}

// ==================== DETAILED STATUS (ONLY WHEN REQUESTED) ====================
void printDetailedStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.printf("Crop: %s (%.1fcm spacing)\n",
        cropConfigs[currentCrop].name, customSpacing * 100);
    
    if (testMode && testSpeedSet) {
        Serial.println("Mode: TEST");
        Serial.printf("Test Speed: %.1f km/h\n", testSpeedKmh);
        Serial.printf("Target RPM: %.0f\n", controlState.targetRPM);
        Serial.printf("PWM: %d\n", getCurrentPWM());
        Serial.printf("Motor: %s\n", controlState.motorRunning ? "RUNNING" : "STOPPED");
    } else {
        Serial.println("Mode: GPS");
        Serial.printf("Raw Speed: %.1f km/h\n", controlState.rawSpeedKmh);
        Serial.printf("Filtered Speed: %.1f km/h\n", controlState.filteredSpeed * 3.6f);
        Serial.printf("GPS: %s (%d satellites, HDOP: %.1f)\n",
            controlState.gpsLocked ? "LOCKED" : "SEARCHING",
            controlState.satelliteCount, controlState.gpsHDOP);
        Serial.printf("Target RPM: %.0f\n", controlState.targetRPM);
        Serial.printf("PWM: %d\n", getCurrentPWM());
    }
    
    Serial.printf("System: %s\n", systemReady ? "READY" : "STANDBY");
    Serial.printf("Free Memory: %d KB\n", ESP.getFreeHeap() / 1024);
    Serial.println("====================\n");
}

// ==================== FIXED LCD INITIALIZATION ====================
void initializeEnhancedLCD() {
    Wire.begin(LCD_SDA, LCD_SCL);
    Wire.setClock(100000);
    
    int status = lcd.begin(20, 4);
    if (status != 0) { // Fixed: Use return value instead of deprecated method
        Serial.println(F("[ERROR] LCD initialization failed"));
        Serial.printf("[ERROR] LCD status code: %d\n", status);
        addSystemLog("LCD failed", "ERROR");
        return;
    }
    
    // Enhanced startup display optimized for 20x4
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Multi-Crop Drill")); // 19 chars
    lcd.setCursor(0, 1);
    lcd.print(F("System Starting..."));
    
    Serial.println(F("[LCD] Enhanced 20x4 LCD successfully initialized"));
    addSystemLog("Enhanced 20x4 LCD with multi-crop support initialized");
}

// ==================== ENHANCED DUAL-CORE TASKS ====================
void taskEnhancedGPS(void *pvParameters) {
    Serial.println(F("[CORE0] Enhanced GPS/Web task started"));
    while (true) {
        // Process GPS data
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid() && gps.speed.isValid()) {
                    float rawSpeedKmh = gps.speed.kmph();
                    int satellites = gps.satellites.value();
                    float hdop = gps.hdop.hdop();
                    
                    // Fixed: Add proper age calculation
                    unsigned long dataAge = gps.location.age();
                    
                    // Process with Kalman filter
                    processTractorGPSMeasurement(kalman, controlState,
                        rawSpeedKmh, satellites, hdop,
                        gps.location.isValid(), dataAge);
                    applyTractorPostProcessing(kalman, controlState);
                }
            }
        }
        
        // Handle serial commands
        handleEnhancedSerialCommands();
        
        // System safety checks
        checkSystemSafety();
        
        // Test mode monitoring
        if (testMode && testSpeedSet) {
            printTestModeStatus();
        }
        
        // Update environmental sensors
        if (millis() - lastEnvironmentalUpdate > 10000) {
            // Read soil moisture (if sensor connected)
            controlState.soilMoisture = analogRead(SOIL_MOISTURE_PIN) / 40.95f; // Convert to percentage
            
            // Read temperature
            controlState.ambientTemp = temperatureRead();
            controlState.motorTemp = controlState.ambientTemp + (controlState.motorRunning ? 15.0f : 0);
            
            // Update fuzzy controller with environmental data
            fuzzyController.set_environmental_sensors(controlState.soilMoisture, controlState.ambientTemp);
            lastEnvironmentalUpdate = millis();
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void taskEnhancedMotor(void *pvParameters) {
    Serial.println(F("[CORE1] Enhanced motor control task started"));
    while (true) {
        float targetRPM = 0.0f;
        
        // Calculate target RPM based on current crop and speed
        if (testMode && testSpeedSet && !emergencyStop) {
            float speed_ms = testSpeedKmh / 3.6f;
            targetRPM = calculateCropSpecificRPM(speed_ms, currentCrop);
            systemReady = true;
            
            // Debug output
            if (verboseMode) {
                Serial.printf("[TEST] Speed: %.1f km/h -> %.1f m/s -> RPM: %.1f\n", 
                            testSpeedKmh, speed_ms, targetRPM);
            }
        } else if (controlState.speedValid && controlState.filteredSpeed > 0.1f && 
                   systemReady && !emergencyStop) {
            targetRPM = calculateCropSpecificRPM(controlState.filteredSpeed, currentCrop);
            
            // Debug output  
            if (verboseMode) {
                Serial.printf("[GPS] Speed: %.1f m/s -> RPM: %.1f\n", 
                            controlState.filteredSpeed, targetRPM);
            }
        }
        
        // Use fuzzy PID if enabled, otherwise use traditional PID
        if (fuzzyControlEnabled && targetRPM > 0) {
            float actualRPM = readActualRPM();
            float dt = 0.02f; // 20ms update rate
            float fuzzyOutput = fuzzyController.calculate(targetRPM, actualRPM, dt);
            // Apply fuzzy output (simplified)
            int pwm = constrain((int)fuzzyOutput, 0, 255);
            setEnhancedMotorForward(pwm);
            controlState.targetRPM = targetRPM;
            controlState.actualRPM = actualRPM;
        } else {
            // Use traditional enhanced motor control
            updateEnhancedMotorControl(controlState, targetRPM,
                                     controlState.speedValid || (testMode && testSpeedSet));
        }
        
        // Monitor motor health
        monitorMotorHealth();
        
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// ==================== MAIN SETUP FUNCTION ====================
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println(F("\n🌾 ======================================== 🌾"));
    Serial.println(F(" ENHANCED MULTI-CROP PRECISION SEED DRILL"));
    Serial.println(F(" 🇮🇳 Built for Indian Agriculture 🇮🇳"));
    Serial.println(F(" Fuzzy AI • Multi-Crop • GPS Precision"));
    Serial.println(F("🌾 ======================================== 🌾\n"));
    
    Serial.printf("Compilation: %s %s\n", __DATE__, __TIME__);
    Serial.printf("ESP32: %s Rev%d, Free: %dKB\n",
        ESP.getChipModel(), ESP.getChipRevision(), ESP.getFreeHeap()/1024);
    Serial.printf("Supported Crops: %d\n", CROP_COUNT);
    Serial.println(F("Features: Fuzzy PID, Kalman Filter, Dual-Core, Web Interface\n"));
    
    // Initialize EEPROM
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println(F("[ERROR] EEPROM initialization failed"));
        addSystemLog("EEPROM initialization failed", "ERROR");
    } else {
        Serial.println(F("[EEPROM] Initialized with multi-crop support"));
        addSystemLog("Enhanced EEPROM initialized");
    }
    
    // Initialize LCD
    initializeEnhancedLCD();
    delay(1000);
    
    // Initialize GPS
    gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    initTractorKalman(kalman);
    Serial.println(F("[GPS] GPS with Kalman filter initialized"));
    addSystemLog("GPS system initialized");
    
    // Initialize motor system - CRITICAL FIX
    initializeEnhancedMotorSystem();
    delay(500);
    
    // Load calibration and crop configuration
    loadCalibration(eepromData, calib_a, calib_b, calibrated);
    
    // Load saved crop configuration
    if (eepromData.version == EEPROM_VERSION) {
        currentCrop = (CropType)constrain(eepromData.selectedCrop, 0, CROP_COUNT-1);
        if (eepromData.customSpacing[currentCrop] > 0.01f) {
            customSpacing = eepromData.customSpacing[currentCrop];
        }
        fuzzyControlEnabled = eepromData.fuzzyEnabled;
    }
    
    // Initialize fuzzy controller
    fuzzyController.configure_for_crop_type(currentCrop);
    Serial.printf("[FUZZY] Initialized for %s\n", cropConfigs[currentCrop].name);
    
    // Print startup banner
    printStartupBanner(calib_a, calib_b, calibrated);
    Serial.printf("Default Crop: %s %s (%.1fcm spacing)\n",
        cropConfigs[currentCrop].icon,
        cropConfigs[currentCrop].name,
        customSpacing * 100);
    
    // Create enhanced dual-core tasks
    xTaskCreatePinnedToCore(
        taskEnhancedGPS,
        "Enhanced_GPS_Web",
        8192, // Increased stack size
        NULL,
        2,
        &taskGPSHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        taskEnhancedMotor,
        "Enhanced_Motor_AI",
        8192, // Reduced stack size to prevent memory issues
        NULL,
        3,
        &taskMotorHandle,
        1
    );
    
    Serial.println(F("[SYSTEM] Enhanced dual-core system initialized"));
    Serial.println(F("[SYSTEM] Multi-crop precision agriculture system ready"));
    
    // Update LCD to show ready status
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("System Ready"));
    lcd.setCursor(0, 1);
    lcd.print(cropConfigs[currentCrop].name);
    lcd.setCursor(0, 2);
    lcd.print(F("Type 'help' for cmds"));
    
    addSystemLog("Enhanced multi-crop system initialized");
    Serial.println(F("\nType 'help' for available commands\n"));
}

// ==================== MAIN LOOP ====================
void loop() {
    static unsigned long lastMemoryCheck = 0;
    
    // Memory management check every 10 seconds
    if (millis() - lastMemoryCheck > 10000) {
        if (ESP.getFreeHeap() < MIN_FREE_HEAP) {
            Serial.printf("[⚠️] Low memory warning: %d bytes\n", ESP.getFreeHeap());
            addSystemLog("Low memory condition", "WARNING");
            
            // Clear old logs if memory is critically low
            if (ESP.getFreeHeap() < 10000) {
                for (int i = 0; i < 50; i++) {
                    systemLogs[i] = "";
                }
                logIndex = 0;
                Serial.println(F("[🧹] Emergency log cleanup performed"));
            }
        }
        lastMemoryCheck = millis();
    }
    
    // System ready check
    systemReady = (controlState.gpsLocked || (testMode && testSpeedSet)) && 
                  !emergencyStop && 
                  (ESP.getFreeHeap() > MIN_FREE_HEAP);
    
    printSimpleFieldStatus();
    
    delay(10); // Main loop runs at ~100Hz
}
