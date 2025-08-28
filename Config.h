#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==================== HARDWARE PIN DEFINITIONS ====================
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define R_PWM 25 // Right motor PWM
#define L_PWM 26 // Left motor PWM
#define R_EN 27 // Right motor enable
#define L_EN 14 // Left motor enable
#define R_IS 35 // Right motor current sense
#define L_IS 34 // Left motor current sense
#define HALL_SENSOR_PIN 2 // Hall sensor for seed counting
#define CALIB_BUTTON_PIN 0 // Calibration button
#define LCD_SDA 21 // LCD I2C data
#define LCD_SCL 22 // LCD I2C clock
#define SOIL_MOISTURE_PIN 36 // Analog pin for soil moisture sensor
#define TEMPERATURE_PIN 39 // Temperature sensor pin

// ==================== MOTOR CONTROL PARAMETERS ====================
#define MOTOR_PWM_MIN 50
#define MOTOR_PWM_MAX 255
#define MOTOR_RPM_MAX 3000.0f
#define MOTOR_TO_ROLLER_GEAR_RATIO 2.5f
#define SEEDS_PER_REVOLUTION 8.0f

// ==================== SEEDING PARAMETERS ====================
#define TARGET_SEED_SPACING 0.20f // 20m in meters
#define MIN_OPERATING_SPEED 0.5f // 0.5 km/h minimum
#define MAX_OPERATING_SPEED 15.0f // 15 km/h maximum

// ==================== MULTI-CROP CONFIGURATIONS ====================
enum CropType {
    CROP_WHEAT = 0,
    CROP_SOYBEAN = 1,
    CROP_COTTON = 2,
    CROP_GROUNDNUT = 3,
    CROP_MUSTARD = 4,
    CROP_SUNFLOWER = 5,
    CROP_MAIZE = 6,
    CROP_CHICKPEA = 7,
    CROP_COUNT = 8
};

struct CropConfiguration {
    const char* name;           
    float targetSpacing;        
    int seedsPerKg;            
    const char* season;         
    float expectedYield;        // Keep - you mentioned this  
    const char* icon;          // Keep but minimize - single letter
     float seedRate;             // Remove if not essential
    
};

// ==================== GPS PARAMETERS ====================
#define GPS_MIN_SATELLITES 4
#define GPS_MAX_HDOP 5.0f
#define GPS_TIMEOUT_MS 5000

// ==================== CALIBRATION PARAMETERS ====================
#define EEPROM_VERSION 43 // Updated for multi-crop support
#define EEPROM_SIZE 256 // Increased for crop configs
#define NUM_CALIB_POINTS 6
#define MIN_R_SQUARED 0.85f
#define MIN_VALID_RPM 10.0f
#define MAX_VALID_RPM 500.0f

// ==================== TIMING CONSTANTS ====================
#define CALIBRATION_STABILIZATION_DELAY 3000
#define ENHANCED_STABILIZATION_DELAY 5000
#define MOTOR_WARMUP_DELAY 10000
#define COOLDOWN_DELAY 2000
#define AUTO_TUNE_TEST_DURATION 8000
#define AUTO_TUNE_SETTLING_TIME 3000
#define OPTIMAL_PWM_FREQUENCY 25000
#define SERIAL_CHECK_DELAY 100
#define PWM_UPDATE_INTERVAL 50
#define RPM_CALCULATION_INTERVAL 500
#define DIAGNOSTIC_PRINT_INTERVAL 12000
#define HEALTH_CHECK_INTERVAL 30000
#define DISPLAY_UPDATE_INTERVAL 1000
#define STATUS_PRINT_INTERVAL 100
#define SYSTEM_CHECK_INTERVAL 500

// ==================== SYSTEM PARAMETERS ====================
#define MIN_FREE_HEAP 15000 // Increased for robustness
#define WATCHDOG_TIMEOUT 30000

// ==================== FILTERING PARAMETERS ====================
#define SPEED_FILTER_ALPHA 0.3f
#define LOG_BUFFER_SIZE 50 // Increased for better logging
#define SPEED_BUFFER_SIZE 5

// ==================== FUZZY CONTROLLER PARAMETERS ====================
#define FUZZY_RULE_COUNT 27
#define FUZZY_INPUT_COUNT 3
#define FUZZY_OUTPUT_COUNT 3

// ==================== STRUCTURE DEFINITIONS ====================
struct DetailedStats {
    unsigned long loopCount = 0;
    unsigned long totalRuntime = 0;
    float avgLoopTime = 0;
    float maxLoopTime = 0;
    float minLoopTime = 0;
    float cpuUsage = 0;
    uint32_t freeHeap = 0;
    uint32_t minFreeHeap = 0;
    float gpsDataRate = 0;
    float avgRPM = 0;
    float maxRPM = 0;
    float maxSpeed = 0;
    int currentPWM = 0;
    float actualSpacing = 0;
    float spacingError = 0;
    float totalDistance = 0;
    float totalOperatingTime = 0;
    int wifiClients = 0;
    String lastError = "None";
    String systemHealth = "Good";
    int gpsPackets = 0;
    int gpsErrors = 0;
    int gpsTimeouts = 0;
    float fieldEfficiency = 95.0f;
    float fuelConsumption = 0.0f;
    float seedCostPerHa = 0.0f;
};

extern DetailedStats detailedStats;

// Enhanced EEPROM data structure
struct EEPROMData {
    int version;
    float a; // Calibration coefficient A (slope)
    float b; // Calibration coefficient B (intercept)
    int selectedCrop; // Current crop selection
    float soilMoisture; // Soil moisture compensation
    float temperatureCoeff; // Temperature coefficient
    bool fuzzyEnabled; // Fuzzy controller enabled
    float customSpacing[CROP_COUNT]; // Custom spacing per crop
};

// Enhanced Control state structure
struct ControlState {
    // GPS data
    bool gpsLocked;
    bool speedValid;
    bool motorRunning;
    bool systemReady;
    bool calibrationMode;
    float rawSpeedKmh;
    float filteredSpeed; // in m/s
    float acceleration; // in m/s²
    float targetRPM;
    float actualRPM;
    int satelliteCount;
    float gpsHDOP;
    unsigned long lastGPSUpdate;
    unsigned long lastUpdate;
    
    // Multi-crop additions
    CropType currentCrop;
    float currentSpacing;
    float seedRate;
    float soilMoisture;
    float ambientTemp;
    float motorTemp;
    bool fuzzyControlActive;
};

// Global variable declarations
extern float calib_a;
extern float calib_b;
extern bool calibrated;
extern ControlState controlState;
extern bool emergencyStop;
extern bool testMode;
extern float testSpeedKmh;
extern bool testSpeedSet;

// Crop configurations array
extern const CropConfiguration cropConfigs[CROP_COUNT];

#endif // CONFIG_H
